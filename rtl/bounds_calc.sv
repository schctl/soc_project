//=============================================================================
// bounds_calc.sv  –  Face / eye-region bound extraction
//
// Bug fixes in this version:
//
// PRIMARY BUG (timing race — this was the actual root cause):
//   bounds_calc is enabled the moment S3_GRAD starts, running concurrently
//   with u_sg_ph.  u_sg_ph needs ~1027 cycles to finish writing phg_bram
//   (IDLE + PASS1 + PASS2).  bounds_calc only spends ~512 cycles in XSCAN
//   before entering YSCAN_GRAD and reading phg_bram — while u_sg_ph is
//   still partway through PASS2.  The sdp_bram returns the OLD value
//   (zeros from initialization) for any address that hasn't been written
//   yet.  Every phg_bram read in YSCAN_GRAD therefore returns 0, so no
//   gradient crossing is ever detected, y_grad_found stays false, and the
//   FSM falls through to the PHSCAN raw-Ph fallback unconditionally.
//
//   Fix: add a WAIT state between XSCAN and YSCAN_GRAD and a new input
//   ph_grad_ready.  The WAIT state holds until ph_grad_ready pulses
//   (driven by grad_done from u_sg_ph in the top module), guaranteeing
//   phg_bram is fully populated before YSCAN_GRAD begins.
//
// SECONDARY BUG (PHSCAN fallback → nose):
//   Because of the timing bug above, PHSCAN was the only path that ever
//   ran.  For this close-up dark-skinned face the absolute minimum raw-Ph
//   value in [130, 280] lands near the nose bridge rather than the eyes.
//
//   Fix (still relevant as a safety fallback):
//   • SEARCH_TOP raised to 150, SEARCH_BOT raised to 290 (eyes in a
//     512-px upscaled 227-px image are at ~y=200-250; nose bridge starts
//     ~y=300).
//   • Removed the destructive 50/50 blend with y_grad_idx (which combined
//     a correct eye reading with an eyebrow/forehead Ph-minimum and pulled
//     the centre into the wrong region).
//
// PREVIOUS BUG FIX (retained):
//   YSCAN_GRAD now only counts gradient crossings within [SRCH_TOP,
//   SRCH_BOT] and triggers on the 2nd crossing in that window, not the
//   3rd globally.  This prevents the nostril local-minimum from being
//   counted when the hair band is absent in close-up images.
//
// FSM: IDLE → XSCAN → WAIT → YSCAN_GRAD → PHSCAN → FINISH
//=============================================================================
`timescale 1ns/1ps

module bounds_calc #(
    parameter LEN           = 512,
    parameter GW            = 18,
    parameter PW            = 17,
    parameter ABITS         = 9,

    parameter EYE_HALF      = 60,
    parameter MIN_FACE_PIX  = 64,
    parameter GUARD_Y       = 60,

    // Eye search window (for 512-px image upscaled from 227-px close-up):
    //   hair/forehead ends  ≈ y=140
    //   eyebrow minimum     ≈ y=170–185   ← 1st crossing in window
    //   eye minimum         ≈ y=230–250   ← 2nd crossing in window (captured)
    //   nose bridge begins  ≈ y=295+
    parameter SEARCH_TOP    = 150,
    parameter SEARCH_BOT    = 290,

    parameter EYE_FALLBACK_Y = 210
)(
    input  logic              clk,
    input  logic              rst_n,
    input  logic              en,

    // NEW: pulse high for one cycle when u_sg_ph has finished writing
    //      phg_bram.  Driven by grad_done in drowsiness_detection_top.
    input  logic              ph_grad_ready,

    // Pv gradient BRAM
    output logic [ABITS-1:0]  pvg_raddr,
    input  logic [GW-1:0]     pvg_rdata,

    // Ph gradient BRAM (read only after ph_grad_ready)
    output logic [ABITS-1:0]  phg_raddr,
    input  logic [GW-1:0]     phg_rdata,

    // Ph projection BRAM (raw sums, PHSCAN fallback)
    output logic [ABITS-1:0]  ph_raddr,
    input  logic [PW-1:0]     ph_rdata,

    output logic [ABITS-1:0]  xmin, xmax,
    output logic [ABITS-1:0]  ymin, ymax,
    output logic              done
);
    localparam [ABITS-1:0] LEN_M1   = ABITS'(LEN - 1);
    localparam [ABITS-1:0] LEN_EYE  = ABITS'(LEN - EYE_HALF);
    localparam [ABITS-1:0] EYE_H    = ABITS'(EYE_HALF);
    localparam [ABITS-1:0] MIN_FACE = ABITS'(MIN_FACE_PIX);
    localparam [ABITS-1:0] SRCH_TOP = ABITS'(SEARCH_TOP);
    localparam [ABITS-1:0] SRCH_BOT = ABITS'(SEARCH_BOT);
    localparam [ABITS-1:0] EYE_FB_Y = ABITS'(EYE_FALLBACK_Y);

    // WAIT is new — 6 states still fits in 3 bits
    typedef enum logic [2:0] {
        IDLE, XSCAN, WAIT, YSCAN_GRAD, PHSCAN, FINISH
    } state_t;
    state_t state;

    logic [ABITS-1:0] idx;

    // X-scan
    logic        prev_pos_x;
    logic [1:0]  x_peak_cnt;

    // Y gradient scan
    logic [2:0]        y_min_cnt;
    logic [GW-1:0]     prev_grad_y;
    logic [ABITS-1:0]  y_grad_idx;
    logic              y_grad_found;

    // Ph minimum scan (fallback)
    logic [PW-1:0]    ph_min_val;
    logic [ABITS-1:0] ph_min_idx;

    // Sign helpers
    logic gx_positive, prev_neg_y, curr_pos_y;
    assign gx_positive = !pvg_rdata[GW-1] && (pvg_rdata != '0);
    assign prev_neg_y  =  prev_grad_y[GW-1];
    assign curr_pos_y  = !phg_rdata[GW-1] && (phg_rdata != '0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            idx          <= '0;
            xmin         <= '0;    xmax         <= LEN_M1;
            ymin         <= '0;    ymax         <= LEN_M1;
            done         <= '0;
            prev_pos_x   <= '0;   x_peak_cnt   <= '0;
            y_min_cnt    <= '0;   prev_grad_y  <= '0;
            y_grad_idx   <= '0;   y_grad_found <= '0;
            ph_min_val   <= '1;   ph_min_idx   <= EYE_FB_Y;
        end else begin
            done <= '0;

            case (state)

                // -------------------------------------------------------
                IDLE: if (en) begin
                    state        <= XSCAN;
                    idx          <= '0;
                    prev_pos_x   <= '0;   x_peak_cnt   <= '0;
                    y_min_cnt    <= '0;   prev_grad_y  <= '0;
                    y_grad_found <= '0;
                    ph_min_val   <= '1;   ph_min_idx   <= EYE_FB_Y;
                    xmin <= '0;  xmax <= LEN_M1;
                end

                // -------------------------------------------------------
                // XSCAN: detect face x-bounds from Pv gradient peaks.
                // Runs for exactly 512 cycles.  phg_bram is NOT read here;
                // u_sg_ph is still writing it during this window.
                // -------------------------------------------------------
                XSCAN: begin
                    pvg_raddr <= idx;

                    if (idx >= 9'd1) begin
                        if (prev_pos_x && !gx_positive) begin
                            case (x_peak_cnt)
                                2'd0: begin xmin <= idx - 9'd1; x_peak_cnt <= 2'd1; end
                                2'd1: begin xmax <= idx - 9'd1; x_peak_cnt <= 2'd2; end
                                default: ;
                            endcase
                        end
                    end
                    prev_pos_x <= gx_positive;

                    if (idx == LEN_M1) begin
                        // phg_bram is NOT ready yet — go to WAIT first
                        state       <= WAIT;
                        idx         <= '0;
                        prev_grad_y <= '0;
                    end else begin
                        idx <= idx + 9'd1;
                    end
                end

                // -------------------------------------------------------
                // WAIT: hold here until u_sg_ph signals it has finished
                // writing all 512 gradient values into phg_bram.
                //
                // ph_grad_ready is driven by grad_done (= u_sg_ph.done)
                // in drowsiness_detection_top.  It pulses for exactly one
                // cycle, which is guaranteed to arrive while we are in
                // this state because:
                //   u_sg_ph finishes at ≈ cycle 1027 of S3_GRAD
                //   XSCAN ends at       ≈ cycle  512 of S3_GRAD
                //   → we wait ≈ 515 cycles, well within the pulse window.
                // -------------------------------------------------------
                WAIT: begin
                    if (ph_grad_ready) begin
                        state       <= YSCAN_GRAD;
                        idx         <= '0;
                        prev_grad_y <= '0;
                        y_min_cnt   <= '0;
                    end
                end

                // -------------------------------------------------------
                // YSCAN_GRAD: count Ph gradient crossings (neg→pos = local
                // minimum in the smoothed Ph curve) strictly within the
                // eye search window [SRCH_TOP, SRCH_BOT].
                //
                // 2nd crossing in window = eye row:
                //   y_min_cnt==0 → 1st crossing (eyebrows, ≈y=175)
                //   y_min_cnt==1 → 2nd crossing (eyes,     ≈y=240) ← capture
                //
                // Crossings outside [SRCH_TOP, SRCH_BOT] are ignored,
                // so the hair minimum (y<150) and nostril minimum (y>290)
                // are never counted.
                // -------------------------------------------------------
                YSCAN_GRAD: begin
                    phg_raddr <= idx;

                    if (idx >= 9'd1
                        && idx >= SRCH_TOP
                        && idx <= SRCH_BOT
                        && prev_neg_y && curr_pos_y) begin

                        y_min_cnt <= y_min_cnt + 3'd1;

                        if (y_min_cnt == 3'd1 && !y_grad_found) begin
                            y_grad_idx   <= idx - 9'd1;
                            y_grad_found <= 1'b1;
                        end
                    end
                    prev_grad_y <= phg_rdata;

                    if (idx == LEN_M1) begin
                        state <= PHSCAN;
                        idx   <= SRCH_TOP;
                    end else begin
                        idx <= idx + 9'd1;
                    end
                end

                // -------------------------------------------------------
                // PHSCAN: fallback — find the row with the minimum raw Ph
                // value in [SRCH_TOP, SRCH_BOT].  Only used when
                // y_grad_found is false (fewer than 2 clear crossings in
                // the search window).
                // -------------------------------------------------------
                PHSCAN: begin
                    ph_raddr <= idx;

                    if (idx > SRCH_TOP) begin
                        if (ph_rdata < ph_min_val) begin
                            ph_min_val <= ph_rdata;
                            ph_min_idx <= idx - 9'd1;
                        end
                    end

                    if (idx == SRCH_BOT) begin
                        state <= FINISH;
                    end else begin
                        idx <= idx + 9'd1;
                    end
                end

                // -------------------------------------------------------
                // FINISH
                // -------------------------------------------------------
                FINISH: begin
                    // X fallback
                    if (xmax <= xmin || (xmax - xmin) < MIN_FACE) begin
                        xmin <= '0;
                        xmax <= LEN_M1;
                    end

                    // Y window — prefer gradient result; fallback to
                    // Ph minimum scan only when gradient method found nothing.
                    // The old 50/50 blend is removed: blending a correct
                    // y_grad_idx with an incorrect ph_min_idx (forehead/
                    // eyebrow level) pulled the eye centre into the wrong
                    // region.
                    begin
                        logic [ABITS-1:0] eye_centre;

                        if (y_grad_found)
                            eye_centre = y_grad_idx;
                        else
                            eye_centre = ph_min_idx;

                        ymin <= (eye_centre > EYE_H)   ? eye_centre - EYE_H : '0;
                        ymax <= (eye_centre < LEN_EYE) ? eye_centre + EYE_H : LEN_M1;
                    end

                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
