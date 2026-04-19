//=============================================================================
// prewitt_edge_unit.sv  –  3x3 Prewitt edge detector (streaming, 1 px/cycle)
//=============================================================================
`timescale 1ns/1ps

module prewitt_edge_unit #(
    parameter int IMG_W    = 512,
    parameter int IMG_H    = 512,
    parameter int PIX_W    = 8,
    parameter int THRESH   = 30,
    parameter int ABITS    = 9
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

    (* ram_style = "distributed" *) logic [PIX_W-1:0] lb0 [0:IMG_W-1];
    (* ram_style = "distributed" *) logic [PIX_W-1:0] lb1 [0:IMG_W-1];
    (* ram_style = "distributed" *) logic [PIX_W-1:0] lb2 [0:IMG_W-1];

    logic [ABITS-1:0] x, y;
    localparam [ABITS-1:0] X_MIN = '0;
    localparam [ABITS-1:0] Y_MIN = '0;
    localparam [ABITS-1:0] X_MAX = ABITS'(IMG_W - 1);
    localparam [ABITS-1:0] Y_MAX = ABITS'(IMG_H - 1);
    logic             active;
    logic [1:0]       lb_cur_idx;
    logic [1:0]       row_cnt;
    logic             done_pending;

    logic [1:0] lb_mid_idx, lb_old_idx;

    // Use bounds_calc region only (no fallback to full frame)
    logic [ABITS-1:0] xmin_eff, xmax_eff, ymin_eff, ymax_eff;
    always_comb begin
        xmin_eff = xmin;
        xmax_eff = xmax;
        ymin_eff = ymin;
        ymax_eff = ymax;
    end
    always_comb begin
        unique case (lb_cur_idx)
            2'd0   : begin lb_mid_idx = 2'd2; lb_old_idx = 2'd1; end
            2'd1   : begin lb_mid_idx = 2'd0; lb_old_idx = 2'd2; end
            2'd2   : begin lb_mid_idx = 2'd1; lb_old_idx = 2'd0; end
            default: begin lb_mid_idx = 2'd0; lb_old_idx = 2'd0; end
        endcase
    end

    logic [PIX_W-1:0] mid_px, old_px;
    always_comb begin
        unique case (lb_mid_idx)
            2'd0   : mid_px = lb0[x];
            2'd1   : mid_px = lb1[x];
            2'd2   : mid_px = lb2[x];
            default: mid_px = '0;
        endcase
        unique case (lb_old_idx)
            2'd0   : old_px = lb0[x];
            2'd1   : old_px = lb1[x];
            2'd2   : old_px = lb2[x];
            default: old_px = '0;
        endcase
    end

    assign fb_addr = {{(18-2*ABITS){1'b0}}, y, x};

    logic [PIX_W-1:0] cur0, cur1;
    logic [PIX_W-1:0] mid_c1, mid_c2;
    logic [PIX_W-1:0] old_c1, old_c2;

    logic [9:0]         sumL, sumM, sumR;
    logic [9:0]         sumT, sumB;
    logic signed [11:0] gx_s, gy_s;
    logic        [10:0] abs_gx, abs_gy;
    logic        [11:0] estr;
    logic        [7:0]  gx_clip, gy_clip;
    logic               edge_bit;

    always_comb begin
        sumL = {2'b0, old_c2} + {2'b0, mid_c2} + {2'b0, cur1};
        sumM = {2'b0, old_c1} + {2'b0, mid_c1} + {2'b0, cur0};
        sumR = {2'b0, old_px} + {2'b0, mid_px} + {2'b0, fb_data};

        sumT = {2'b0, old_c2} + {2'b0, old_c1} + {2'b0, old_px};
        sumB = {2'b0, cur1  } + {2'b0, cur0  } + {2'b0, fb_data};

        gx_s = $signed({2'b0, sumR}) - $signed({2'b0, sumL});
        gy_s = $signed({2'b0, sumB}) - $signed({2'b0, sumT});

        abs_gx = gx_s[11] ? 11'($unsigned(-gx_s)) : 11'($unsigned(gx_s));
        abs_gy = gy_s[11] ? 11'($unsigned(-gy_s)) : 11'($unsigned(gy_s));

        estr = {1'b0, abs_gx} + {1'b0, abs_gy};

        gx_clip = (abs_gx > 11'd255) ? 8'hFF : abs_gx[7:0];
        gy_clip = (abs_gy > 11'd255) ? 8'hFF : abs_gy[7:0];

        edge_bit = (estr > 12'(THRESH));
    end

    logic [ABITS-1:0] out_x_calc, out_y_calc;
    always_comb begin
        out_x_calc = x - 9'd1;
        out_y_calc = y - 9'd1;
    end

    // Pipeline-valid for gradient computation (independent of ROI)
    logic valid_now;
    assign valid_now = active
                     && (row_cnt >= 2'd2)
                     && (x >= 9'd2);

    // ROI check is done at output stage using centred coords
    logic out_in_roi;
    assign out_in_roi = (out_x >= xmin_eff)
                     && (out_x <= xmax_eff)
                     && (out_y >= ymin_eff)
                     && (out_y <= ymax_eff);

    logic             out_v;
    logic [ABITS-1:0] out_x, out_y;
    logic [7:0]       out_gx, out_gy;
    logic             out_edge;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur0 <= '0; cur1 <= '0;
            mid_c1 <= '0; mid_c2 <= '0;
            old_c1 <= '0; old_c2 <= '0;
            x <= '0; y <= '0;
            active       <= 1'b0;
            lb_cur_idx   <= 2'd0;
            row_cnt      <= 2'd0;
            done_pending <= 1'b0;
            out_v    <= 1'b0;
            out_x    <= '0;   out_y <= '0;
            out_gx   <= '0;   out_gy <= '0;
            out_edge <= 1'b0;
            edge_waddr <= '0; edge_wbit <= 1'b0; edge_wen <= 1'b0;
            gx_waddr   <= '0; gx_wdata  <= '0;   gx_wen   <= 1'b0;
            gy_waddr   <= '0; gy_wdata  <= '0;   gy_wen   <= 1'b0;
            done       <= 1'b0;
        end else begin
            edge_wen <= 1'b0;
            gx_wen   <= 1'b0;
            gy_wen   <= 1'b0;
            done     <= 1'b0;

            if (out_v && out_in_roi) begin
                edge_waddr <= {{(18-2*ABITS){1'b0}}, out_y, out_x};
                edge_wbit  <= out_edge;
                edge_wen   <= 1'b1;

                gx_waddr   <= {{(18-2*ABITS){1'b0}}, out_y, out_x};
                gx_wdata   <= out_gx;
                gx_wen     <= 1'b1;

                gy_waddr   <= {{(18-2*ABITS){1'b0}}, out_y, out_x};
                gy_wdata   <= out_gy;
                gy_wen     <= 1'b1;
            end

            if (done_pending && !out_v) begin
                done         <= 1'b1;
                done_pending <= 1'b0;
            end

            if (en && !active) begin
                active     <= 1'b1;
                x          <= X_MIN;
                y          <= Y_MIN;
                lb_cur_idx <= 2'd0;
                row_cnt    <= 2'd0;
                cur0 <= '0; cur1 <= '0;
                mid_c1 <= '0; mid_c2 <= '0;
                old_c1 <= '0; old_c2 <= '0;
                out_v <= 1'b0;
            end else if (active) begin

                unique case (lb_cur_idx)
                    2'd0   : lb0[x] <= fb_data;
                    2'd1   : lb1[x] <= fb_data;
                    2'd2   : lb2[x] <= fb_data;
                    default: ;
                endcase

                cur1   <= cur0;    cur0   <= fb_data;
                mid_c2 <= mid_c1;  mid_c1 <= mid_px;
                old_c2 <= old_c1;  old_c1 <= old_px;

                if (valid_now) begin
                    out_v    <= 1'b1;
                    out_x    <= out_x_calc;
                    out_y    <= out_y_calc;
                    out_gx   <= gx_clip;
                    out_gy   <= gy_clip;
                    out_edge <= edge_bit;
                end else begin
                    out_v <= 1'b0;
                end

                if (x == X_MAX) begin
                    x <= X_MIN;

                    cur0 <= '0; cur1 <= '0;
                    mid_c1 <= '0; mid_c2 <= '0;
                    old_c1 <= '0; old_c2 <= '0;

                    lb_cur_idx <= (lb_cur_idx == 2'd2) ? 2'd0 : lb_cur_idx + 2'd1;

                    if (row_cnt < 2'd2) row_cnt <= row_cnt + 2'd1;

                    if (y == Y_MAX) begin
                        active       <= 1'b0;
                        done_pending <= 1'b1;
                    end else begin
                        y <= y + 9'd1;
                    end
                end else begin
                    x <= x + 9'd1;
                end
            end else begin
                out_v <= 1'b0;
            end
        end
    end

endmodule
