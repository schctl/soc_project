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
    logic [ABITS-1:0] m_val;   // eye-region height in rows
    assign m_val = ymax - ymin + 9'd1;

    typedef enum logic [1:0] { IDLE, SCAN, FINISH, UNUSED } state_t;
    state_t state;

    // Read-address generator (row-major scan over ROI)
    logic [ABITS-1:0] xi, yi;
    logic             issue_done;

    // Response metadata to align returned edge_rbit with coordinates
    logic             samp_valid;
    logic [ABITS-1:0] samp_x, samp_y;

    // Transition-count state for current row
    logic             prev_bit;
    logic [ABITS-1:0] trans_cnt;

    // Accumulator
    logic [COMP_W-1:0] acc;

    // Triangular row weight (paper-style weighted transition sum)
    function automatic [9:0] row_weight(
        input logic [ABITS-1:0] row_idx,
        input logic [ABITS-1:0] height
    );
        logic [ABITS-1:0] half_h;
        begin
            half_h = height >> 1;
            if (row_idx <= half_h)
                row_weight = {1'b0, row_idx} << 1;
            else
                row_weight = {1'b0, (height - row_idx)} << 1;
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            xi           <= '0;
            yi           <= '0;
            issue_done   <= 1'b0;
            samp_valid   <= 1'b0;
            samp_x       <= '0;
            samp_y       <= '0;
            acc          <= '0;
            done         <= '0;
            compl_scaled <= '0;
            trans_cnt    <= '0;
            prev_bit     <= '0;
            edge_raddr   <= '0;
        end else begin
            logic [ABITS-1:0] row_idx_now;
            logic [ABITS-1:0] row_trans_total;
            logic [ABITS-1:0] add_trans;
            logic [9:0]       w_row;

            done <= '0;

            // Derived values for currently returned sample (if valid)
            row_idx_now     = samp_y - ymin;
            add_trans       = ((samp_x > xmin) && (edge_rbit ^ prev_bit)) ? ABITS'(1) : '0;
            row_trans_total = trans_cnt + add_trans;
            w_row           = row_weight(row_idx_now, m_val);

            case (state)
                // ------------------------------------------------------------
                IDLE: if (en) begin
                    state      <= SCAN;
                    xi         <= xmin;
                    yi         <= ymin;
                    issue_done <= 1'b0;
                    samp_valid <= 1'b0;
                    acc        <= '0;
                    trans_cnt  <= '0;
                    prev_bit   <= '0;
                end

                // ------------------------------------------------------------
                SCAN: begin
                    // 1) Process returned sample from previous cycle
                    if (samp_valid) begin
                        if (samp_x == xmin) begin
                            // First pixel in row: establish baseline
                            trans_cnt <= '0;
                            prev_bit  <= edge_rbit;
                        end else begin
                            trans_cnt <= row_trans_total;
                            prev_bit  <= edge_rbit;
                        end

                        // End of row: commit weighted transitions and reset
                        if (samp_x == xmax) begin
                            acc       <= acc + COMP_W'(row_trans_total) * COMP_W'(w_row);
                            trans_cnt <= '0;
                            prev_bit  <= '0;
                        end
                    end

                    // 2) Issue next read address (until full ROI issued)
                    if (!issue_done) begin
                        edge_raddr <= {yi, xi};

                        // Save metadata for returned sample in next cycle
                        samp_x     <= xi;
                        samp_y     <= yi;
                        samp_valid <= 1'b1;

                        if (xi == xmax) begin
                            xi <= xmin;
                            if (yi == ymax) begin
                                issue_done <= 1'b1;
                            end else begin
                                yi <= yi + ABITS'(1);
                            end
                        end else begin
                            xi <= xi + ABITS'(1);
                        end
                    end else begin
                        // No further issues; drain one last sample
                        if (samp_valid)
                            samp_valid <= 1'b0;
                        else
                            state <= FINISH;
                    end
                end

                // ------------------------------------------------------------
                FINISH: begin
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
