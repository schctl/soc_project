`timescale 1ns/1ps

module bounds_calc #(
    parameter LEN      = 512,
    parameter GW       = 18,
    parameter ABITS    = 9,
    parameter EYE_HALF = 20
)(
    input  logic              clk,
    input  logic              rst_n,
    input  logic              en,

    output logic [ABITS-1:0]  pvg_raddr,
    input  logic [GW-1:0]     pvg_rdata,

    output logic [ABITS-1:0]  phg_raddr,
    input  logic [GW-1:0]     phg_rdata,

    output logic [ABITS-1:0]  xmin, xmax,
    output logic [ABITS-1:0]  ymin, ymax,
    output logic              done
);
    localparam [ABITS-1:0] LEN_M1     = ABITS'(LEN - 1);
    localparam [ABITS-1:0] LEN_EYE    = ABITS'(LEN - EYE_HALF);
    localparam [ABITS-1:0] EYE_H      = ABITS'(EYE_HALF);

    typedef enum logic [1:0] { IDLE, XSCAN, YSCAN, FINISH } state_t;
    state_t state;

    logic [ABITS-1:0] idx;

    // X-scan
    logic              prev_pos_x;
    logic [1:0]        x_peak_cnt;

    // Y-scan - module-level declarations (avoids LOCALVAR-in-always_ff)
    logic [1:0]        y_min_cnt;
    logic [GW-1:0]     prev_grad_y;
    logic [ABITS-1:0]  y_min_idx;
    logic              prev_neg_y;   // sign of previous Ph gradient sample
    logic              curr_pos_y;   // sign of current  Ph gradient sample

    // Pv gradient: positive = non-negative and non-zero
    logic gx_positive;
    assign gx_positive = !pvg_rdata[GW-1] && (pvg_rdata != '0);

    // Ph sign bits - combinational from registered prev_grad_y / current read
    assign prev_neg_y = prev_grad_y[GW-1];
    assign curr_pos_y = !phg_rdata[GW-1] && (phg_rdata != '0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            idx        <= '0;
            xmin       <= '0;    xmax       <= LEN_M1;
            ymin       <= '0;    ymax       <= LEN_M1;
            done       <= '0;
            prev_pos_x <= '0;
            x_peak_cnt <= '0;
            y_min_cnt  <= '0;
            prev_grad_y <= '0;
            y_min_idx  <= '0;
        end else begin
            done <= '0;

            case (state)
                // ----------------------------------------------------------------
                IDLE: if (en) begin
                    state      <= XSCAN;
                    idx        <= '0;
                    prev_pos_x <= '0;
                    x_peak_cnt <= '0;
                    y_min_cnt  <= '0;
                    prev_grad_y <= '0;
                end

                // ----------------------------------------------------------------
                // Scan Pv gradient: two positive→non-positive crossings = face edges
                // ----------------------------------------------------------------
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
                        state <= YSCAN;
                        idx   <= '0;
                        prev_grad_y <= '0;
                    end else begin
                        idx <= idx + 9'd1;
                    end
                end

                // ----------------------------------------------------------------
                // Scan Ph gradient: 3rd negative→positive crossing = eye row
                // ----------------------------------------------------------------
                YSCAN: begin
                    phg_raddr <= idx;

                    // Detect trough (neg→pos crossing)
                    if (idx >= 9'd1 && prev_neg_y && curr_pos_y) begin
                        y_min_cnt <= y_min_cnt + 2'd1;
                        y_min_idx <= idx - 9'd1;
                    end
                    prev_grad_y <= phg_rdata;

                    if (idx == LEN_M1) begin
                        state <= FINISH;
                    end else begin
                        idx <= idx + 9'd1;
                    end
                end

                // ----------------------------------------------------------------
                FINISH: begin
                    ymin  <= (y_min_idx > EYE_H)    ? y_min_idx - EYE_H : '0;
                    ymax  <= (y_min_idx < LEN_EYE)  ? y_min_idx + EYE_H : LEN_M1;
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
