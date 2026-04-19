// =============================================================================
// sim_main.cpp – Verilator testbench with intermediate-stage image capture
//
// Writes these PNG files for each input image named "foo.jpg":
//   foo_1_gray.png          – grayscale input (512×512)
//   foo_2_bounds.png        – grayscale + detected eye-region rectangle overlay
//   foo_3_edge_gx.png       – |Gx| (vertical edge response, 8-bit greyscale)
//   foo_4_edge_gy.png       – |Gy| (horizontal edge response, 8-bit greyscale)
//   foo_5_edge_binary.png   – thresholded binary edge image
//
// Requires:
//   stb_image.h       (run `bash sim/get_stb.sh`)
//   stb_image_write.h (run `bash sim/get_stb.sh`)
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cassert>
#include <string>
#include <stdexcept>
#include <filesystem>
#include <iostream>

// ---- stb_image (decode) -----------------------------------------------------
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STBI_ONLY_BMP
#include "stb_image.h"

// ---- stb_image_write (encode PNG/BMP) ---------------------------------------
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ---- Verilator generated header ---------------------------------------------
#include "Vdrowsiness_detection_top.h"
#include "verilated.h"
#ifdef TRACE_ENABLE
#include "verilated_vcd_c.h"
#endif

// =============================================================================
// Constants
// =============================================================================
static constexpr int IMG_W      = 512;
static constexpr int IMG_H      = 512;
static constexpr int MAX_CYCLES = 20'000'000;

// =============================================================================
// Buffers — all indexed as [row * IMG_W + col]
// =============================================================================
static uint8_t framebuf [IMG_W * IMG_H];   // grayscale input

// Intermediate capture buffers (same spatial extent as framebuf)
// The eye-region outputs are sparse; pixels outside the region stay 0.
static uint8_t gx_buf   [IMG_W * IMG_H];   // |Gx| from prewitt
static uint8_t gy_buf   [IMG_W * IMG_H];   // |Gy| from prewitt
static uint8_t edge_buf [IMG_W * IMG_H];   // binary edge (0 or 255)

// Write diagnostics
static uint8_t gx_written   [IMG_W * IMG_H];
static uint8_t gy_written   [IMG_W * IMG_H];
static uint8_t edge_written [IMG_W * IMG_H];
static uint64_t gx_writes = 0, gy_writes = 0, edge_writes = 0;
static uint64_t gx_nonzero_writes = 0, gy_nonzero_writes = 0, edge_one_writes = 0;
static uint8_t fb_x_seen[IMG_W];
static uint8_t fb_y_seen[IMG_H];

// Bounds (filled when dbg_bounds_valid fires)
static int bounds_xmin = 0, bounds_xmax = IMG_W-1;
static int bounds_ymin = 0, bounds_ymax = IMG_H-1;

// =============================================================================
// Helpers
// =============================================================================

// Clear intermediate buffers before a new frame
static void clear_capture_buffers()
{
    memset(gx_buf,   0, sizeof gx_buf);
    memset(gy_buf,   0, sizeof gy_buf);
    memset(edge_buf, 0, sizeof edge_buf);
    memset(gx_written,   0, sizeof gx_written);
    memset(gy_written,   0, sizeof gy_written);
    memset(edge_written, 0, sizeof edge_written);
    gx_writes = gy_writes = edge_writes = 0;
    gx_nonzero_writes = gy_nonzero_writes = edge_one_writes = 0;
    memset(fb_x_seen, 0, sizeof fb_x_seen);
    memset(fb_y_seen, 0, sizeof fb_y_seen);
}

// Load image → framebuf (nearest-neighbour resize to 512×512, force greyscale)
static void load_image(const char* path)
{
    int w, h, ch;
    uint8_t* data = stbi_load(path, &w, &h, &ch, 1);
    if (!data)
        throw std::runtime_error(std::string("stbi_load: ") + stbi_failure_reason());

    printf("[TB] Loaded '%s'  %dx%d  ch=%d\n", path, w, h, ch);

    for (int r = 0; r < IMG_H; r++) {
        for (int c = 0; c < IMG_W; c++) {
            int sr = (int)((float)r / IMG_H * h);
            int sc = (int)((float)c / IMG_W * w);
            sr = std::min(sr, h - 1);
            sc = std::min(sc, w - 1);
            framebuf[r * IMG_W + c] = data[sr * w + sc];
        }
    }
    stbi_image_free(data);
}

// Derive output filename stem: "dir/foo.jpg" → "dir/foo"
static std::string stem(const char* path)
{
    namespace fs = std::filesystem;
    fs::path p(path);
    return (p.parent_path() / p.stem()).string();
}

// Write a greyscale PNG
static void write_gray_png(const char* path, const uint8_t* buf,
                            int w = IMG_W, int h = IMG_H)
{
    if (!stbi_write_png(path, w, h, 1, buf, w))
        fprintf(stderr, "[TB] WARNING: could not write %s\n", path);
    else
        printf("[TB] Wrote %s\n", path);
}

// Write bounds-overlay PNG: copy gray image then draw a coloured rectangle.
// Output is RGB so the rectangle can be a distinct colour (red).
static void write_bounds_png(const char* path, const uint8_t* gray,
                              int xmin, int xmax, int ymin, int ymax)
{
    // Build an RGB image
    static uint8_t rgb[IMG_W * IMG_H * 3];
    for (int i = 0; i < IMG_W * IMG_H; i++) {
        rgb[i*3+0] = gray[i];
        rgb[i*3+1] = gray[i];
        rgb[i*3+2] = gray[i];
    }

    auto draw_hline = [&](int y, int x0, int x1, uint8_t r, uint8_t g, uint8_t b) {
        if (y < 0 || y >= IMG_H) return;
        for (int x = std::max(0,x0); x <= std::min(IMG_W-1,x1); x++) {
            rgb[(y*IMG_W+x)*3+0] = r;
            rgb[(y*IMG_W+x)*3+1] = g;
            rgb[(y*IMG_W+x)*3+2] = b;
        }
    };
    auto draw_vline = [&](int x, int y0, int y1, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || x >= IMG_W) return;
        for (int y = std::max(0,y0); y <= std::min(IMG_H-1,y1); y++) {
            rgb[(y*IMG_W+x)*3+0] = r;
            rgb[(y*IMG_W+x)*3+1] = g;
            rgb[(y*IMG_W+x)*3+2] = b;
        }
    };

    // Draw eye-region rectangle in green (3-pixel thick for visibility)
    for (int t = -1; t <= 1; t++) {
        draw_hline(ymin+t, xmin, xmax, 0,   255, 0);
        draw_hline(ymax+t, xmin, xmax, 0,   255, 0);
        draw_vline(xmin+t, ymin, ymax, 0,   255, 0);
        draw_vline(xmax+t, ymin, ymax, 0,   255, 0);
    }

    if (!stbi_write_png(path, IMG_W, IMG_H, 3, rgb, IMG_W*3))
        fprintf(stderr, "[TB] WARNING: could not write %s\n", path);
    else
        printf("[TB] Wrote %s  bounds=(%d,%d)–(%d,%d)\n",
               path, xmin, ymin, xmax, ymax);
}

// =============================================================================
// Clock / simulation state
// =============================================================================
static vluint64_t sim_time = 0;
#ifdef TRACE_ENABLE
static VerilatedVcdC* g_trace = nullptr;
#endif

static void tick(Vdrowsiness_detection_top* dut)
{
    // Combinational frame-buffer read before rising edge
    dut->fb_data = framebuf[dut->fb_addr & 0x3FFFF];

    // Falling edge
    dut->clk = 0;
    dut->eval();
#ifdef TRACE_ENABLE
    if (g_trace) g_trace->dump(sim_time);
#endif
    sim_time++;

    // Rising edge — sample outputs here
    dut->clk = 1;
    dut->eval();
#ifdef TRACE_ENABLE
    if (g_trace) g_trace->dump(sim_time);
#endif
    sim_time++;

    // ---- Capture intermediate outputs ----------------------------------------

    // Track framebuffer address coverage (helps infer edge-stage scan region)
    {
        uint32_t a = dut->fb_addr & 0x3FFFF;
        uint32_t x = a & 0x1FF;
        uint32_t y = (a >> 9) & 0x1FF;
        if (x < IMG_W) fb_x_seen[x] = 1;
        if (y < IMG_H) fb_y_seen[y] = 1;
    }

    // Bounds: latch when bounds_valid pulses
    if (dut->dbg_bounds_valid) {
        bounds_xmin = dut->dbg_xmin;
        bounds_xmax = dut->dbg_xmax;
        bounds_ymin = dut->dbg_ymin;
        bounds_ymax = dut->dbg_ymax;
    }

    // Gx image
    if (dut->dbg_gx_wen) {
        uint32_t addr = dut->dbg_gx_waddr & 0x3FFFF;
        if (addr < IMG_W * IMG_H) {
            gx_buf[addr] = dut->dbg_gx_wdata;
            gx_written[addr] = 1;
            gx_writes++;
            if (dut->dbg_gx_wdata != 0) gx_nonzero_writes++;
        }
    }

    // Gy image
    if (dut->dbg_gy_wen) {
        uint32_t addr = dut->dbg_gy_waddr & 0x3FFFF;
        if (addr < IMG_W * IMG_H) {
            gy_buf[addr] = dut->dbg_gy_wdata;
            gy_written[addr] = 1;
            gy_writes++;
            if (dut->dbg_gy_wdata != 0) gy_nonzero_writes++;
        }
    }

    // Binary edge image (1 bit → 0 or 255)
    if (dut->dbg_edge_wen) {
        uint32_t addr = dut->dbg_edge_waddr & 0x3FFFF;
        if (addr < IMG_W * IMG_H) {
            edge_buf[addr] = dut->dbg_edge_wbit ? 255 : 0;
            edge_written[addr] = 1;
            edge_writes++;
            if (dut->dbg_edge_wbit) edge_one_writes++;
        }
    }
}

// =============================================================================
// Reset
// =============================================================================
static void apply_reset(Vdrowsiness_detection_top* dut)
{
    dut->rst_n = 0;
    dut->start = 0;
    for (int i = 0; i < 8; i++) tick(dut);
    dut->rst_n = 1;
    tick(dut);
    printf("[TB] Reset released @ sim_time=%llu\n", (unsigned long long)sim_time);
}

// =============================================================================
// Run one frame, wait for decision_valid, then dump all intermediate PNGs.
// Returns: 0=alert, 1=drowsy, -1=timeout
// =============================================================================
static int run_one_frame(Vdrowsiness_detection_top* dut, const char* imgpath)
{
    clear_capture_buffers();

    const std::string pfx = "data_out/";

    // --- 1. Save grayscale input immediately ---
    write_gray_png((pfx + "_1_gray.png").c_str(), framebuf);

    // --- Start the DUT ---
    dut->start = 1;
    tick(dut);
    dut->start = 0;

    // --- Run until decision_valid ---
    bool bounds_written = false;
    uint32_t last_compl_scaled = 0;
    bool compl_seen = false;

    for (int cycle = 0; cycle < MAX_CYCLES; cycle++) {
        tick(dut);

        if (dut->dbg_cmplx_done) {
            last_compl_scaled = dut->dbg_compl_scaled;
            compl_seen = true;
        }

        // --- 2. Bounds overlay — write once, the cycle bounds_valid fires ---
        if (!bounds_written && dut->dbg_bounds_valid) {
            // Bounds are stable for rest of simulation from this point
            bounds_written = true;
            // We write the PNG immediately so we capture the exact cycle
            write_bounds_png((pfx + "_2_bounds.png").c_str(),
                             framebuf,
                             bounds_xmin, bounds_xmax,
                             bounds_ymin, bounds_ymax);
        }

        if (dut->decision_valid) {
            int result = (int)dut->drowsy;

             uint64_t gx_unique = 0, gy_unique = 0, edge_unique = 0;
             uint8_t gx_max = 0, gy_max = 0;
            int gx_xmin = IMG_W, gx_xmax = -1, gx_ymin = IMG_H, gx_ymax = -1;
            int gy_xmin = IMG_W, gy_xmax = -1, gy_ymin = IMG_H, gy_ymax = -1;
            int fb_xmin = IMG_W, fb_xmax = -1, fb_ymin = IMG_H, fb_ymax = -1;
            uint64_t fb_x_cov = 0, fb_y_cov = 0;
             for (int i = 0; i < IMG_W * IMG_H; i++) {
              gx_unique += gx_written[i] ? 1 : 0;
              gy_unique += gy_written[i] ? 1 : 0;
              edge_unique += edge_written[i] ? 1 : 0;
              if (gx_buf[i] > gx_max) gx_max = gx_buf[i];
              if (gy_buf[i] > gy_max) gy_max = gy_buf[i];
                if (gx_written[i]) {
                    int y = i / IMG_W;
                    int x = i % IMG_W;
                    if (x < gx_xmin) gx_xmin = x;
                    if (x > gx_xmax) gx_xmax = x;
                    if (y < gx_ymin) gx_ymin = y;
                    if (y > gx_ymax) gx_ymax = y;
                }
                if (gy_written[i]) {
                    int y = i / IMG_W;
                    int x = i % IMG_W;
                    if (x < gy_xmin) gy_xmin = x;
                    if (x > gy_xmax) gy_xmax = x;
                    if (y < gy_ymin) gy_ymin = y;
                    if (y > gy_ymax) gy_ymax = y;
                }
             }
            for (int x = 0; x < IMG_W; x++) {
                if (fb_x_seen[x]) {
                    fb_x_cov++;
                    if (x < fb_xmin) fb_xmin = x;
                    if (x > fb_xmax) fb_xmax = x;
                }
            }
            for (int y = 0; y < IMG_H; y++) {
                if (fb_y_seen[y]) {
                    fb_y_cov++;
                    if (y < fb_ymin) fb_ymin = y;
                    if (y > fb_ymax) fb_ymax = y;
                }
            }

             printf("[TB] BOUNDS: xmin=%d xmax=%d ymin=%d ymax=%d area=%d\n",
                 bounds_xmin, bounds_xmax, bounds_ymin, bounds_ymax,
                 (bounds_xmax - bounds_xmin + 1) * (bounds_ymax - bounds_ymin + 1));
             printf("[TB] EDGE_DIAG: gx_writes=%llu gx_unique=%llu gx_nonzero_writes=%llu gx_max=%u\n",
                 (unsigned long long)gx_writes,
                 (unsigned long long)gx_unique,
                 (unsigned long long)gx_nonzero_writes,
                 (unsigned)gx_max);
            if (gx_unique)
                printf("[TB] EDGE_DIAG: gx_bbox=(%d,%d)-(%d,%d)\n", gx_xmin, gx_ymin, gx_xmax, gx_ymax);
             printf("[TB] EDGE_DIAG: gy_writes=%llu gy_unique=%llu gy_nonzero_writes=%llu gy_max=%u\n",
                 (unsigned long long)gy_writes,
                 (unsigned long long)gy_unique,
                 (unsigned long long)gy_nonzero_writes,
                 (unsigned)gy_max);
            if (gy_unique)
                printf("[TB] EDGE_DIAG: gy_bbox=(%d,%d)-(%d,%d)\n", gy_xmin, gy_ymin, gy_xmax, gy_ymax);
                 printf("[TB] EDGE_DIAG: fb_addr_x_cov=%llu x_range=(%d,%d) fb_addr_y_cov=%llu y_range=(%d,%d)\n",
                     (unsigned long long)fb_x_cov, fb_xmin, fb_xmax,
                     (unsigned long long)fb_y_cov, fb_ymin, fb_ymax);
             printf("[TB] EDGE_DIAG: edge_writes=%llu edge_unique=%llu edge_one_writes=%llu\n",
                 (unsigned long long)edge_writes,
                 (unsigned long long)edge_unique,
                 (unsigned long long)edge_one_writes);

                 printf("[TB] COMPLEXITY: compl_scaled=%u%s\n",
                   compl_seen ? last_compl_scaled : dut->dbg_compl_scaled,
                     compl_seen ? "" : " (sampled at decision)");

            // --- 3. Edge images (fully populated after edge_done) ---
            write_gray_png((pfx + "_3_edge_gx.png").c_str(),  gx_buf);
            write_gray_png((pfx + "_4_edge_gy.png").c_str(),  gy_buf);
            write_gray_png((pfx + "_5_edge_binary.png").c_str(), edge_buf);

            printf("[TB] RESULT: %s  (complexity proxy ready)\n",
                   result ? "DROWSY" : "ALERT");
            return result;
        }
    }

    fprintf(stderr, "[TB] ERROR: timeout!\n");
    return -1;
}

// =============================================================================
// main
// =============================================================================
int main(int argc, char** argv)
{
    if (argc < 2) {
        fprintf(stderr,
            "Usage: %s <image.jpg|png|bmp> [...]\n"
            "\n"
            "Outputs per image (e.g. 'driver.jpg'):\n"
            "  driver_1_gray.png         grayscale input\n"
            "  driver_2_bounds.png       eye-region rectangle overlay\n"
            "  driver_3_edge_gx.png      |Gx| vertical edge response\n"
            "  driver_4_edge_gy.png      |Gy| horizontal edge response\n"
            "  driver_5_edge_binary.png  thresholded binary edge map\n",
            argv[0]);
        return 1;
    }

    Verilated::commandArgs(argc, argv);
    auto* dut = new Vdrowsiness_detection_top;

#ifdef TRACE_ENABLE
    Verilated::traceEverOn(true);
    g_trace = new VerilatedVcdC;
    dut->trace(g_trace, 99);
    g_trace->open("drowsiness_sim.vcd");
    printf("[TB] VCD: drowsiness_sim.vcd\n");
#endif

    apply_reset(dut);

    for (int i = 1; i < argc; i++) {
        printf("\n[TB] ======== %s ========\n", argv[i]);
        try {
            load_image(argv[i]);
        } catch (const std::exception& e) {
            fprintf(stderr, "[TB] SKIP: %s\n", e.what());
            continue;
        }

        run_one_frame(dut, argv[i]);

        // Idle between frames
        for (int c = 0; c < 16; c++) tick(dut);
    }

    printf("\n[TB] Simulation complete.\n");

#ifdef TRACE_ENABLE
    if (g_trace) { g_trace->close(); delete g_trace; }
#endif
    dut->final();
    delete dut;
    return 0;
}
