//=============================================================================
// prewitt_edge_unit.sv  –  3×3 Prewitt edge detector (streaming, 1 px/cycle)
//
// Root cause of the black-output bug in v3/v4:
// ------------------------------------------------
// The previous design read line-buffer address xn = x+1 (look-ahead) from
// whichever buffer was currently being WRITTEN (current row).  Because the
// write pointer is still at x, position xn hasn't been written yet — so the
// read returns data from the PREVIOUS TIME that buffer was used (3 rows ago).
// This corrupted the right-column of the 3×3 window and made absGx ≈ 0.
//
// New design — no look-ahead, process center at (x-1):
// -------------------------------------------------------
//  • 3 line buffers using DISTRIBUTED RAM (async / 0-latency read).
//    Reading any address from the previously-completed rows is safe because
//    those buffers are not being written during the current row.
//  • lb_cur_idx tracks which LB holds the current (write) row.
//    lb_mid and lb_old point to the two preceding complete rows.
//  • At each cycle (reading I[y][x]):
//      right_col = { lb_old[x], lb_mid[x], fb_data }   ← async read at x
//      center_col = { old_c1, mid_c1, cur0 }           ← 1-cycle-old reads
//      left_col   = { old_c2, mid_c2, cur1 }           ← 2-cycle-old reads
//    where cur0/cur1 are a 2-deep shift register of past fb_data values.
//  • Output pixel centre is at (x-1, y); guarded by x >= xmin+2 and
//    row_cnt >= 2 (two complete previous rows available).
//  • Combinational Prewitt, registered 1 cycle, then written to output BRAMs.
//  • done fires aligned with the last output pixel (1 cycle after last valid).
//=============================================================================
`timescale 1ns/1ps

module prewitt_edge_unit #(
    parameter IMG_W    = 512,
    parameter IMG_H    = 512,
    parameter PIX_W    = 8,
    parameter THRESH   = 9'd30,
    parameter ABITS    = 9
)(
    input  logic              clk,
    input  logic              rst_n,
    input  logic              en,

    input  logic [ABITS-1:0]  xmin, xmax,
    input  logic [ABITS-1:0]  ymin, ymax,

    output logic [17:0]       fb_addr,
    input  logic [PIX_W-1:0]  fb_data,

    output logic [17:0]       edge_waddr,
    output logic              edge_wbit,
    output logic              edge_wen,

    output logic [17:0]       gx_waddr,
    output logic [7:0]        gx_wdata,
    output logic              gx_wen,

    output logic [17:0]       gy_waddr,
    output logic [7:0]        gy_wdata,
    output logic              gy_wen,

    output logic              done
);

    // =========================================================================
    // 3 line buffers — distributed RAM for 0-latency async read
    // (each IMG_W × 8-bit = 4 kB, fits in LUTRAM on Zynq 7020)
    // =========================================================================
    (* ram_style = "distributed" *) logic [PIX_W-1:0] lb0 [0:IMG_W-1];
    (* ram_style = "distributed" *) logic [PIX_W-1:0] lb1 [0:IMG_W-1];
    (* ram_style = "distributed" *) logic [PIX_W-1:0] lb2 [0:IMG_W-1];

    // lb_cur_idx: which LB is being written (current row)
    // (lb_cur_idx+1)%3 = mid (y-1), (lb_cur_idx+2)%3 = old (y-2)
    logic [1:0] lb_cur_idx;

    // Precompute mid and old indices combinationally
    logic [1:0] lb_mid_idx, lb_old_idx;
    always_comb begin
        case (lb_cur_idx)
            2'd0: begin lb_mid_idx = 2'd2; lb_old_idx = 2'd1; end
            2'd1: begin lb_mid_idx = 2'd0; lb_old_idx = 2'd2; end
            2'd2: begin lb_mid_idx = 2'd1; lb_old_idx = 2'd0; end
            default: begin lb_mid_idx = 2'd0; lb_old_idx = 2'd1; end
        endcase
    end

    // Async (0-latency) read of mid and old at current column x
    logic [PIX_W-1:0] mid_c, old_c;
    always_comb begin
        case (lb_mid_idx)
            2'd0: mid_c = lb0[x];
            2'd1: mid_c = lb1[x];
            default: mid_c = lb2[x];
        endcase
        case (lb_old_idx)
            2'd0: old_c = lb0[x];
            2'd1: old_c = lb1[x];
            default: old_c = lb2[x];
        endcase
    end

    // =========================================================================
    // Column shift registers
    // cur0 = fb_data one cycle ago = I[y][x-1]      (center col, current row)
    // cur1 = fb_data two cycles ago = I[y][x-2]     (left col, current row)
    // mid_c1 = lb_mid[x-1], mid_c2 = lb_mid[x-2]   (center and left of mid row)
    // old_c1 = lb_old[x-1], old_c2 = lb_old[x-2]   (center and left of old row)
    // =========================================================================
    logic [PIX_W-1:0] cur0, cur1;
    logic [PIX_W-1:0] mid_c1, mid_c2;
    logic [PIX_W-1:0] old_c1, old_c2;

    // =========================================================================
    // Prewitt kernels (combinational, computed BEFORE registering)
    //
    // Window (top=old=y-2, bot=cur=y, left=x-2, right=x):
    //
    //   old_c2  old_c1  old_c      ← row y-2 (top)
    //   mid_c2  mid_c1  mid_c      ← row y-1 (middle)
    //   cur1    cur0    fb_data    ← row y   (bottom / current)
    //
    // Kx (vertical edges):  [-1 0 +1; -1 0 +1; -1 0 +1]
    //   Gx = (old_c+mid_c+fb_data) - (old_c2+mid_c2+cur1)
    //
    // Ky (horizontal edges): [+1 +1 +1; 0 0 0; -1 -1 -1]
    //   Gy = (old_c2+old_c1+old_c) - (cur1+cur0+fb_data)
    // =========================================================================
    logic signed [10:0] Gx_c, Gy_c;
    logic [10:0]        absGx_c, absGy_c, estr_c;
    logic [7:0]         gx_clip_c, gy_clip_c;
    logic               edge_c;

    assign Gx_c = ($signed({3'b0, old_c})  + $signed({3'b0, mid_c})  + $signed({3'b0, fb_data}))
                - ($signed({3'b0, old_c2}) + $signed({3'b0, mid_c2}) + $signed({3'b0, cur1}));

    assign Gy_c = ($signed({3'b0, old_c2}) + $signed({3'b0, old_c1}) + $signed({3'b0, old_c}))
                - ($signed({3'b0, cur1})   + $signed({3'b0, cur0})   + $signed({3'b0, fb_data}));

    assign absGx_c  = Gx_c[10] ? 11'(-Gx_c) : 11'(Gx_c);
    assign absGy_c  = Gy_c[10] ? 11'(-Gy_c) : 11'(Gy_c);
    assign estr_c   = absGx_c + absGy_c;
    assign edge_c   = (estr_c > {2'b0, THRESH});

    // Saturate to 8-bit (top 3 bits non-zero → clamp to 255)
    assign gx_clip_c = (|absGx_c[10:8]) ? 8'hFF : absGx_c[7:0];
    assign gy_clip_c = (|absGy_c[10:8]) ? 8'hFF : absGy_c[7:0];

    // =========================================================================
    // Counters / control
    // =========================================================================
    logic [ABITS-1:0] x, y;
    logic             active;
    logic [1:0]       row_cnt;       // saturates at 2 (two complete prior rows)
    logic             done_pending;  // fires done 1 cycle after last valid output

    // Output centre pixel address: (x-1, y)
    assign fb_addr = {y, x};

    // Valid: need >= 2 complete prior rows and column position x >= xmin+2
    logic valid_now;
    assign valid_now = active && (row_cnt >= 2'd2) && (x >= (xmin + 9'd2));

    // Pipeline stage: register all combinational results + coords
    logic             valid_r;
    logic [ABITS-1:0] out_col_r, out_y_r;
    logic [10:0]      absGx_r, absGy_r, estr_r;
    logic [7:0]       gx_r, gy_r;
    logic             edge_r;

    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x           <= '0;   y           <= '0;
            active      <= '0;   lb_cur_idx  <= '0;
            row_cnt     <= '0;   done_pending <= '0;
            cur0 <= '0; cur1 <= '0;
            mid_c1 <= '0; mid_c2 <= '0;
            old_c1 <= '0; old_c2 <= '0;
            valid_r <= '0;
            out_col_r <= '0; out_y_r <= '0;
            absGx_r <= '0; absGy_r <= '0; estr_r <= '0;
            gx_r <= '0; gy_r <= '0; edge_r <= '0;
            edge_wen <= '0; gx_wen <= '0; gy_wen <= '0;
            done <= '0;
        end else begin
            edge_wen <= '0; gx_wen <= '0; gy_wen <= '0;
            done     <= '0;

            // ---- Flush pipeline output (1 cycle after computation) ----------
            valid_r <= valid_now;
            if (valid_now) begin
                // Latch compute results
                out_col_r <= x - 9'd1;   // centre pixel column
                out_y_r   <= y;
                absGx_r   <= absGx_c;
                absGy_r   <= absGy_c;
                estr_r    <= estr_c;
                gx_r      <= gx_clip_c;
                gy_r      <= gy_clip_c;
                edge_r    <= edge_c;
            end

            if (valid_r) begin
                edge_wen   <= 1'b1;
                edge_waddr <= {out_y_r, out_col_r};
                edge_wbit  <= edge_r;

                gx_wen     <= 1'b1;
                gx_waddr   <= {out_y_r, out_col_r};
                gx_wdata   <= gx_r;

                gy_wen     <= 1'b1;
                gy_waddr   <= {out_y_r, out_col_r};
                gy_wdata   <= gy_r;
            end

            // ---- Fire done one cycle after last valid_r output --------------
            if (done_pending) begin
                done         <= 1'b1;
                done_pending <= '0;
            end

            // ---- Pixel pipeline --------------------------------------------
            if (en && !active) begin
                active      <= 1'b1;
                x           <= xmin;
                y           <= ymin;
                lb_cur_idx  <= '0;
                row_cnt     <= '0;
                cur0 <= '0; cur1 <= '0;
                mid_c1 <= '0; mid_c2 <= '0;
                old_c1 <= '0; old_c2 <= '0;
            end else if (active) begin

                // Write current pixel to the current line buffer
                case (lb_cur_idx)
                    2'd0: lb0[x] <= fb_data;
                    2'd1: lb1[x] <= fb_data;
                    default: lb2[x] <= fb_data;
                endcase

                // Shift column history registers
                cur1  <= cur0;    cur0  <= fb_data;   // current row
                mid_c2 <= mid_c1; mid_c1 <= mid_c;   // mid row
                old_c2 <= old_c1; old_c1 <= old_c;   // old row

                // ---- Counter advance ---------------------------------------
                if (x == xmax) begin
                    x <= xmin;
                    // Reset column shift registers for new row
                    cur0 <= '0; cur1 <= '0;
                    mid_c1 <= '0; mid_c2 <= '0;
                    old_c1 <= '0; old_c2 <= '0;

                    // Rotate line buffer write index
                    lb_cur_idx <= (lb_cur_idx == 2'd2) ? 2'd0 : lb_cur_idx + 2'd1;

                    // Count up to 2 completed rows
                    if (row_cnt < 2'd2) row_cnt <= row_cnt + 2'd1;

                    if (y == ymax) begin
                        active       <= '0;
                        done_pending <= 1'b1;   // fire done after last valid_r
                    end else begin
                        y <= y + 9'd1;
                    end
                end else begin
                    x <= x + 9'd1;
                end
            end
        end
    end

endmodule
