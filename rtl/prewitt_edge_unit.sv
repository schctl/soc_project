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

    output logic              done
);
    // -----------------------------------------------------------------------
    // Line buffers
    // -----------------------------------------------------------------------
    (* ram_style = "block" *) logic [PIX_W-1:0] lb0 [0:IMG_W-1];
    (* ram_style = "block" *) logic [PIX_W-1:0] lb1 [0:IMG_W-1];
    (* ram_style = "block" *) logic [PIX_W-1:0] lb2 [0:IMG_W-1];

    // -----------------------------------------------------------------------
    // Counters / control
    // -----------------------------------------------------------------------
    logic [ABITS-1:0] x, y;
    logic             active;
    logic [1:0]       buf_sel;

    // -----------------------------------------------------------------------
    // 3×3 pixel window (shift registers)
    // -----------------------------------------------------------------------
    logic [PIX_W-1:0] w00,w01,w02;
    logic [PIX_W-1:0] w10,w11,w12;
    logic [PIX_W-1:0] w20,w21,w22;

    // -----------------------------------------------------------------------
    // Prewitt - registered Gx/Gy, then combinational abs + threshold
    // -----------------------------------------------------------------------
    logic signed [10:0] Gx_r, Gy_r;
    logic [ABITS-1:0]   x_d1, y_d1;
    logic               valid_d1;

    // Combinational abs and threshold (fixes BLKSEQ)
    logic [10:0] absGx, absGy, edge_strength;
    logic        edge_comb;

    assign absGx        = Gx_r[10] ? 11'(-Gx_r) : 11'(Gx_r);
    assign absGy        = Gy_r[10] ? 11'(-Gy_r) : 11'(Gy_r);
    assign edge_strength = absGx + absGy;
    assign edge_comb    = (edge_strength > {2'b00, THRESH});

    // -----------------------------------------------------------------------
    // Look-ahead read column index (module level - fixes LOCALVAR warning)
    // -----------------------------------------------------------------------
    logic [ABITS-1:0] xn;
    assign xn = (x < xmax) ? x + 9'd1 : xmax;

    assign fb_addr = {y, x};

    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= '0;  y <= '0;  active <= '0;  buf_sel <= '0;
            Gx_r <= '0;  Gy_r <= '0;
            x_d1 <= '0;  y_d1 <= '0;  valid_d1 <= '0;
            edge_wen <= '0;  done <= '0;
        end else begin
            edge_wen <= '0;
            done     <= '0;
            valid_d1 <= '0;

            if (en && !active) begin
                active  <= 1'b1;
                x       <= xmin;
                y       <= ymin;
                buf_sel <= '0;
            end else if (active) begin

                // Write current pixel into newest line buffer
                case (buf_sel)
                    2'd0: lb0[x] <= fb_data;
                    2'd1: lb1[x] <= fb_data;
                    default: lb2[x] <= fb_data;
                endcase

                // Shift column window; load right column from look-ahead
                w00 <= w01;  w01 <= w02;
                w10 <= w11;  w11 <= w12;
                w20 <= w21;  w21 <= w22;

                // Read right column (xn) from the three rotating buffers
                case (buf_sel)
                    2'd0: begin w02 <= lb0[xn]; w12 <= lb1[xn]; w22 <= lb2[xn]; end
                    2'd1: begin w02 <= lb1[xn]; w12 <= lb2[xn]; w22 <= lb0[xn]; end
                    default: begin w02 <= lb2[xn]; w12 <= lb0[xn]; w22 <= lb1[xn]; end
                endcase

                // Prewitt computation (valid after 2 warm-up rows)
                if (y > ymin + 9'd1) begin
                    valid_d1 <= 1'b1;
                    x_d1     <= x;
                    y_d1     <= y;

                    Gx_r <= $signed({2'b00,w02}) - $signed({2'b00,w00})
                           + $signed({2'b00,w12}) - $signed({2'b00,w10})
                           + $signed({2'b00,w22}) - $signed({2'b00,w20});

                    Gy_r <= $signed({2'b00,w00}) + $signed({2'b00,w01}) + $signed({2'b00,w02})
                           - $signed({2'b00,w20}) - $signed({2'b00,w21}) - $signed({2'b00,w22});
                end

                // Register edge output one cycle after Prewitt (comb results ready)
                if (valid_d1) begin
                    edge_wen   <= 1'b1;
                    edge_waddr <= {y_d1, x_d1};
                    edge_wbit  <= edge_comb;
                end

                // Advance counters
                if (x == xmax) begin
                    x       <= xmin;
                    buf_sel <= (buf_sel == 2'd2) ? 2'd0 : buf_sel + 2'd1;
                    if (y == ymax) begin
                        active <= '0;
                        done   <= 1'b1;
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
