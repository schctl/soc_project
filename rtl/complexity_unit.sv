`timescale 1ns/1ps

module complexity_unit #(
    parameter IMG_W   = 512,
    parameter COMP_W  = 26,
    parameter ABITS   = 9
)(
    input  logic              clk,
    input  logic              rst_n,
    input  logic              en,

    input  logic [ABITS-1:0]  xmin, xmax,
    input  logic [ABITS-1:0]  ymin, ymax,

    output logic [17:0]       edge_raddr,
    input  logic              edge_rbit,

    output logic [COMP_W-1:0] compl_scaled,
    output logic              done
);
    logic [ABITS-1:0] m_val;   // eye-region height
    assign m_val = ymax - ymin + 9'd1;

    typedef enum logic [1:0] { IDLE, SCAN, FINISH, UNUSED } state_t;
    state_t state;

    logic [ABITS-1:0] xi, yi;
    logic              prev_bit;
    logic [ABITS-1:0]  trans_cnt;
    logic [ABITS-1:0]  row_idx;

    // Triangular weight (module-level, combinational - avoids any BLKSEQ risk)
    logic [ABITS-1:0] half_m;
    logic [9:0]       w_i;
    assign half_m = m_val >> 1;
    assign w_i    = (row_idx <= half_m)
                  ? {1'b0, row_idx} << 1
                  : {1'b0, (m_val - row_idx)} << 1;

    // Delayed row accumulation
    logic [ABITS-1:0] row_idx_d1, trans_cnt_d1;
    logic             row_end_d1;
    logic [COMP_W-1:0] acc;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            xi           <= '0;  yi <= '0;
            acc          <= '0;
            done         <= '0;
            compl_scaled <= '0;
            trans_cnt    <= '0;
            prev_bit     <= '0;
            row_idx      <= '0;
            row_end_d1   <= '0;
        end else begin
            done       <= '0;
            row_end_d1 <= '0;

            case (state)
                // ------------------------------------------------------------
                IDLE: if (en) begin
                    state     <= SCAN;
                    xi        <= xmin;
                    yi        <= ymin;
                    row_idx   <= '0;
                    acc       <= '0;
                    trans_cnt <= '0;
                    prev_bit  <= '0;
                end

                // ------------------------------------------------------------
                SCAN: begin
                    edge_raddr <= {yi, xi};

                    // Count horizontal transitions
                    if (xi > xmin)
                        if (edge_rbit ^ prev_bit)
                            trans_cnt <= trans_cnt + 9'd1;
                    prev_bit <= edge_rbit;

                    if (xi == xmax) begin
                        // Capture final transition for this row
                        row_idx_d1  <= row_idx;
                        trans_cnt_d1 <= trans_cnt +
                                        ABITS'(edge_rbit ^ prev_bit ? 1 : 0);
                        row_end_d1  <= 1'b1;

                        xi        <= xmin;
                        trans_cnt <= '0;
                        prev_bit  <= '0;

                        if (yi == ymax) begin
                            state <= FINISH;
                        end else begin
                            yi      <= yi + 9'd1;
                            row_idx <= row_idx + 9'd1;
                        end
                    end else begin
                        xi <= xi + 9'd1;
                    end

                    // Accumulate delayed row (w_i is combinational from row_idx)
                    if (row_end_d1)
                        acc <= acc + COMP_W'(trans_cnt_d1) * COMP_W'(w_i);
                end

                // ------------------------------------------------------------
                FINISH: begin
                    // Flush last delayed accumulation
                    if (row_end_d1)
                        acc <= acc + COMP_W'(trans_cnt_d1) * COMP_W'(w_i);
                    compl_scaled <= acc;
                    done         <= 1'b1;
                    state        <= IDLE;
                end

                // Covers the UNUSED enum value - fixes CASEINCOMPLETE
                default: state <= IDLE;
            endcase
        end
    end

endmodule
