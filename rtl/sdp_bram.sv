//=============================================================================
// sdp_bram.sv – Simple Dual-Port Block RAM (1 write port, 1 read port)
// Read has 1-cycle registered latency (infers as BRAM on Xilinx 7-series)
//=============================================================================
`timescale 1ns/1ps

module sdp_bram #(
    parameter DEPTH = 512,
    parameter WIDTH = 17,
    parameter ABITS = 9       // must satisfy 2^ABITS >= DEPTH
)(
    input  logic              clk,
    // Write port
    input  logic              wr_en,
    input  logic [ABITS-1:0]  wr_addr,
    input  logic [WIDTH-1:0]  wr_data,
    // Read port (1-cycle latency)
    input  logic [ABITS-1:0]  rd_addr,
    output logic [WIDTH-1:0]  rd_data
);
    (* ram_style = "block" *) logic [WIDTH-1:0] mem [0:DEPTH-1];

    // Zero-initialise (avoids X-propagation in simulation)
    initial begin : init_mem
        integer i;
        for (i = 0; i < DEPTH; i++) mem[i] = '0;
    end

    always_ff @(posedge clk) begin
        if (wr_en) mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];          // registered read
    end
endmodule
