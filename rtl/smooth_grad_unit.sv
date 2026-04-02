`timescale 1ns/1ps

module smooth_grad_unit #(
    parameter LEN   = 512,
    parameter DW    = 17,
    parameter GW    = 18,
    parameter ABITS = 9
)(
    input  logic              clk,
    input  logic              rst_n,
    input  logic              en,

    output logic [ABITS-1:0]  src_raddr,
    input  logic [DW-1:0]     src_rdata,

    output logic [ABITS-1:0]  sm_waddr,
    output logic [DW-1:0]     sm_wdata,
    output logic              sm_wen,

    output logic [ABITS-1:0]  sm_raddr,
    input  logic [DW-1:0]     sm_rdata,

    output logic [ABITS-1:0]  grd_waddr,
    output logic [GW-1:0]     grd_wdata,
    output logic              grd_wen,

    output logic              done
);
    // idx goes up to LEN+1 = 513 for pipeline flush → needs 10 bits
    localparam CBITS = 10;
    localparam [CBITS-1:0] FLUSH_VAL = CBITS'(LEN + 1);
    localparam [CBITS-1:0] LEN_M1    = CBITS'(LEN - 1);

    typedef enum logic [1:0] { IDLE, PASS1, PASS2, FINISH } state_t;
    state_t state;

    logic [CBITS-1:0] idx;

    // div3: input is DW+2 bits wide (max sum = 3 * 130560 = 391680 < 2^19)
    function automatic [DW-1:0] div3 (input [DW+1:0] v);
        // 43691 = ceil(2^17/3); exact for v < 3*2^17
        logic [37:0] tmp;
        tmp  = {19'b0, v} * 37'd43691;
        div3 = DW'(tmp >> 17);
    endfunction

    logic [DW-1:0] p_prev, p_curr, p_next;
    logic [DW-1:0] sm_prev, sm_curr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            idx     <= '0;
            sm_wen  <= '0;  grd_wen <= '0;  done <= '0;
        end else begin
            sm_wen  <= '0;  grd_wen <= '0;  done <= '0;

            case (state)
                // ----------------------------------------------------------------
                IDLE: if (en) begin
                    state <= PASS1;
                    idx   <= '0;
                    p_prev <= '0;  p_curr <= '0;  p_next <= '0;
                end

                // ----------------------------------------------------------------
                // Pass 1: smooth[i] = (P[i-1] + P[i] + P[i+1]) / 3
                // Pipeline depth 1: read issued at idx, data arrives next cycle.
                // We write smooth[idx-1] using data that arrived this cycle.
                // ----------------------------------------------------------------
                PASS1: begin
                    // Issue read for next element (boundary: clamp to LEN-1)
                    src_raddr <= ABITS'(idx < LEN_M1 ? idx + CBITS'(1) : LEN_M1);

                    // Shift window
                    p_prev <= p_curr;
                    p_curr <= p_next;
                    p_next <= src_rdata;   // arrived for last cycle's read

                    // Write smooth value (available from cycle 2 onward)
                    if (idx >= CBITS'(2)) begin
                        sm_wen   <= 1'b1;
                        sm_waddr <= ABITS'(idx - CBITS'(2));
                        sm_wdata <= div3((DW+2)'(p_prev) +
                                        (DW+2)'(p_curr) +
                                        (DW+2)'(p_next));
                    end

                    if (idx == FLUSH_VAL) begin
                        state <= PASS2;
                        idx   <= '0;
                        sm_prev <= '0;  sm_curr <= '0;
                    end else begin
                        idx <= idx + CBITS'(1);
                    end
                end

                // ----------------------------------------------------------------
                // Pass 2: gradient[i] = smooth[i] - smooth[i-1]
                // ----------------------------------------------------------------
                PASS2: begin
                    sm_raddr <= ABITS'(idx);

                    sm_prev <= sm_curr;
                    sm_curr <= sm_rdata;

                    if (idx >= CBITS'(2)) begin
                        grd_wen   <= 1'b1;
                        grd_waddr <= ABITS'(idx - CBITS'(2));
                        grd_wdata <= GW'($signed({1'b0, sm_curr}) -
                                         $signed({1'b0, sm_prev}));
                    end

                    if (idx == FLUSH_VAL) begin
                        state <= FINISH;
                    end else begin
                        idx <= idx + CBITS'(1);
                    end
                end

                // ----------------------------------------------------------------
                FINISH: begin
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
