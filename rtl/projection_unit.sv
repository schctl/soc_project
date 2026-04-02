`timescale 1ns/1ps

module projection_unit #(
    parameter IMG_W  = 512,
    parameter IMG_H  = 512,
    parameter PIX_W  = 8,
    parameter SUM_W  = 17
)(
    input  logic              clk,
    input  logic              rst_n,
    input  logic              en,

    output logic [17:0]       fb_addr,
    input  logic [PIX_W-1:0]  fb_data,

    output logic [8:0]        pv_waddr,
    output logic [SUM_W-1:0]  pv_wdata,
    output logic              pv_wen,

    output logic [8:0]        ph_waddr,
    output logic [SUM_W-1:0]  ph_wdata,
    output logic              ph_wen,

    output logic              done
);
    localparam [8:0] W_MAX = 9'(IMG_W - 1);
    localparam [8:0] H_MAX = 9'(IMG_H - 1);

    // Pv LUTRAM (async read - no BLKSEQ issue; write is registered below)
    (* ram_style = "distributed" *)
    logic [SUM_W-1:0] pv_ram [0:IMG_W-1];

    // Initialise for simulation (Xilinx LUTRAM powers up to 0 anyway)
    initial begin : sim_init
        integer i;
        for (i = 0; i < IMG_W; i++) pv_ram[i] = '0;
    end

    logic [SUM_W-1:0] pv_cur;
    assign pv_cur  = pv_ram[x];
    assign fb_addr = {y, x};

    logic [8:0]       x, y;
    logic             active;
    logic [SUM_W-1:0] ph_acc;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= '0;  y <= '0;  active <= '0;
            ph_acc <= '0;
            pv_wen <= '0;  ph_wen <= '0;  done <= '0;
        end else begin
            pv_wen <= '0;  ph_wen <= '0;  done <= '0;

            if (en && !active) begin
                active <= 1'b1;  x <= '0;  y <= '0;  ph_acc <= '0;
            end else if (active) begin

                // Pv: reset accumulator on first row (implicit clear)
                pv_ram[x] <= (y == '0) ? SUM_W'(fb_data)
                                       : pv_cur + SUM_W'(fb_data);

                if (y == H_MAX) begin
                    pv_wen   <= 1'b1;
                    pv_waddr <= x;
                    pv_wdata <= (y == '0) ? SUM_W'(fb_data)
                                          : pv_cur + SUM_W'(fb_data);
                end

                // Ph: running accumulator, written at end of each row
                ph_acc <= (x == '0) ? SUM_W'(fb_data)
                                    : ph_acc + SUM_W'(fb_data);

                if (x == W_MAX) begin
                    ph_wen   <= 1'b1;
                    ph_waddr <= y;
                    ph_wdata <= ph_acc + SUM_W'(fb_data);
                end

                // Counter advance
                if (x == W_MAX) begin
                    x <= '0;
                    if (y == H_MAX) begin
                        active <= '0;  done <= 1'b1;  y <= '0;
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
