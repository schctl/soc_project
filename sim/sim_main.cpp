// =============================================================================
// sim_main.cpp – Verilator testbench for drowsiness_detection_top
//
// Build flow (see Makefile):
//   verilator --cc --exe --build drowsiness_top.sv ... sim_main.cpp
//   ./obj_dir/Vdrowsiness_detection_top  driver_open.jpg
//
// Image loading uses stb_image (single header, no extra deps).
//   - Accepts JPEG, PNG, BMP, etc.
//   - Converts to grayscale automatically.
//   - Resizes/crops to 512×512 (nearest-neighbour) if the image is a
//     different size.
//
// Frame-buffer model:
//   The DUT drives fb_addr every clock.  We keep a flat uint8_t[512*512]
//   array and return fb_data = framebuf[fb_addr] combinationally (same
//   semantics as the SV testbench assign).
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cassert>
#include <string>
#include <stdexcept>
#include <iostream>

// ---- stb_image (header-only) -----------------------------------------------
// Place stb_image.h in the same directory and run `bash get_stb.sh` once.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STBI_ONLY_BMP
#include "stb_image.h"

// ---- Verilator generated header ---------------------------------------------
#include "Vdrowsiness_detection_top.h"
#include "verilated.h"
#include "verilated_vcd_c.h"   // waveform dump (optional, remove if not needed)

// =============================================================================
// Constants
// =============================================================================
static constexpr int IMG_W = 512;
static constexpr int IMG_H = 512;
static constexpr int MAX_CYCLES = 20'000'000;  // safety timeout

// =============================================================================
// Frame buffer (flat, row-major: addr = row*512 + col)
// =============================================================================
static uint8_t framebuf[IMG_W * IMG_H];

// =============================================================================
// Load an image file → greyscale 512×512 frame buffer
// Supports JPEG, PNG, BMP (stb_image handles decoding).
// If the source image is not 512×512 it is cropped/padded from (0,0).
// =============================================================================
static void load_image(const char* path)
{
    int w, h, channels;
    // Request 1 channel (grayscale).  stb_image converts colour automatically.
    uint8_t* data = stbi_load(path, &w, &h, &channels, /*desired_channels=*/1);
    if (!data) {
        throw std::runtime_error(std::string("stbi_load failed: ") + stbi_failure_reason());
    }

    printf("[TB] Loaded '%s'  size=%dx%d  original_channels=%d\n",
           path, w, h, channels);

    // Copy into framebuf with nearest-neighbour sampling if sizes differ
    for (int row = 0; row < IMG_H; row++) {
        for (int col = 0; col < IMG_W; col++) {
            // Map dest pixel to source pixel (nearest-neighbour)
            int src_col = (int)((float)col / IMG_W * w);
            int src_row = (int)((float)row / IMG_H * h);
            src_col = std::min(src_col, w - 1);
            src_row = std::min(src_row, h - 1);

            framebuf[row * IMG_W + col] = data[src_row * w + src_col];
        }
    }
    stbi_image_free(data);
    printf("[TB] Frame buffer loaded (%dx%d greyscale)\n", IMG_W, IMG_H);
}

// =============================================================================
// Clock helpers
// =============================================================================
static vluint64_t sim_time = 0;

static void tick(Vdrowsiness_detection_top* dut, VerilatedVcdC* trace)
{
    // Combinational frame-buffer read: set fb_data before rising edge
    // so the DUT sees it when it samples on posedge.
    dut->fb_data = framebuf[dut->fb_addr & 0x3FFFF];  // mask to 18 bits

    dut->clk = 0;
    dut->eval();
    if (trace) trace->dump(sim_time++);

    dut->clk = 1;
    dut->eval();
    if (trace) trace->dump(sim_time++);
}

// =============================================================================
// Apply synchronous reset
// =============================================================================
static void apply_reset(Vdrowsiness_detection_top* dut, VerilatedVcdC* trace)
{
    dut->rst_n = 0;
    dut->start = 0;
    for (int i = 0; i < 8; i++) tick(dut, trace);
    dut->rst_n = 1;
    tick(dut, trace);
    printf("[TB] Reset released @ sim_time=%llu\n", (unsigned long long)sim_time);
}

// =============================================================================
// Run one processing frame and return the drowsy decision.
// Returns:  0 = alert,  1 = drowsy,  -1 = timeout
// =============================================================================
static int run_one_frame(Vdrowsiness_detection_top* dut, VerilatedVcdC* trace)
{
    // Pulse start for one clock
    dut->start = 1;
    tick(dut, trace);
    dut->start = 0;

    // Wait for decision_valid
    for (int cycle = 0; cycle < MAX_CYCLES; cycle++) {
        tick(dut, trace);
        if (dut->decision_valid) {
            printf("[TB] decision_valid=1 @ sim_time=%llu  drowsy=%d\n",
                   (unsigned long long)sim_time, (int)dut->drowsy);
            return (int)dut->drowsy;
        }
    }
    fprintf(stderr, "[TB] ERROR: timeout waiting for decision_valid!\n");
    return -1;
}

// =============================================================================
// main
// =============================================================================
int main(int argc, char** argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <image.jpg> [image2.jpg ...]\n", argv[0]);
        fprintf(stderr, "  Feeds each image into the DUT and prints drowsy/alert.\n");
        return 1;
    }

    // ---- Verilator initialisation -------------------------------------------
    Verilated::commandArgs(argc, argv);
    Vdrowsiness_detection_top* dut = new Vdrowsiness_detection_top;

    // ---- Optional VCD waveform dump -----------------------------------------
    VerilatedVcdC* trace = nullptr;
#ifdef TRACE_ENABLE
    Verilated::traceEverOn(true);
    trace = new VerilatedVcdC;
    dut->trace(trace, /*depth=*/99);
    trace->open("drowsiness_sim.vcd");
    printf("[TB] VCD trace: drowsiness_sim.vcd\n");
#endif

    // ---- Reset the DUT -------------------------------------------------------
    apply_reset(dut, trace);

    // ---- Process each image supplied on the command line --------------------
    int total = 0, pass = 0;
    for (int i = 1; i < argc; i++) {
        const char* imgpath = argv[i];
        printf("\n[TB] ---- Processing: %s ----\n", imgpath);

        try {
            load_image(imgpath);
        } catch (const std::exception& e) {
            fprintf(stderr, "[TB] SKIP: %s\n", e.what());
            continue;
        }

        int result = run_one_frame(dut, trace);
        total++;

        if (result < 0) {
            printf("[TB] RESULT: TIMEOUT\n");
        } else if (result == 1) {
            printf("[TB] RESULT: DROWSY  (low edge complexity)\n");
        } else {
            printf("[TB] RESULT: ALERT   (high edge complexity)\n");
            pass++;
        }

        // Allow a few idle cycles between frames
        for (int c = 0; c < 16; c++) tick(dut, trace);
    }

    printf("\n[TB] Done. Processed %d image(s).\n", total);

    // ---- Cleanup ------------------------------------------------------------
    if (trace) { trace->close(); delete trace; }
    dut->final();
    delete dut;
    return 0;
}
