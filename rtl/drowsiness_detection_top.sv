`timescale 1ns/1ps

module drowsiness_detection_top #(
    parameter IMG_W         = 512,
    parameter IMG_H         = 512,
    parameter PIX_W         = 8,
    parameter SUM_W         = 17,
    parameter GRAD_W        = 18,
    parameter COMP_W        = 26,
    parameter DROWSY_THRESH = 26'd18
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,

    output logic [17:0] fb_addr,
    input  logic [7:0]  fb_data,

    output logic        drowsy,
    output logic        decision_valid
);

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [2:0] {
        S0_IDLE  = 3'd0,
        S1_FRAME = 3'd1,
        S2_PROJ  = 3'd2,
        S3_GRAD  = 3'd3,
        S6_EDGE  = 3'd4,
        S5_CMPLX = 3'd5,
        S4_DECID = 3'd6
    } state_e;

    state_e state, nstate;

    logic frame_done, proj_done, grad_done, bounds_done, edge_done, cmplx_done;

    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) state <= S0_IDLE;
        else        state <= nstate;

    always_comb begin
        nstate = state;
        case (state)
            S0_IDLE  : if (start)       nstate = S1_FRAME;
            S1_FRAME : if (frame_done)  nstate = S2_PROJ;
            S2_PROJ  : if (proj_done)   nstate = S3_GRAD;
            // Wait for bounds_calc to finish before starting edge detection
            S3_GRAD  : if (bounds_done) nstate = S6_EDGE;
            S6_EDGE  : if (edge_done)   nstate = S5_CMPLX;
            S5_CMPLX : if (cmplx_done)  nstate = S4_DECID;
            S4_DECID :                  nstate = S1_FRAME;
            default  :                  nstate = S0_IDLE;
        endcase
    end

    // =========================================================================
    // BRAMs
    // =========================================================================
    logic [SUM_W-1:0] pv_wdata, pv_rdata;
    logic [8:0]       pv_waddr, pv_raddr;
    logic             pv_wen;
    sdp_bram #(.DEPTH(512),.WIDTH(SUM_W),.ABITS(9)) u_pv_bram (
        .clk(clk),.wr_en(pv_wen),.wr_addr(pv_waddr),.wr_data(pv_wdata),
        .rd_addr(pv_raddr),.rd_data(pv_rdata));

    logic [SUM_W-1:0] ph_wdata, ph_rdata;
    logic [8:0]       ph_waddr, ph_raddr;
    logic             ph_wen;
    sdp_bram #(.DEPTH(512),.WIDTH(SUM_W),.ABITS(9)) u_ph_bram (
        .clk(clk),.wr_en(ph_wen),.wr_addr(ph_waddr),.wr_data(ph_wdata),
        .rd_addr(ph_raddr),.rd_data(ph_rdata));

    logic [SUM_W-1:0] sm_wdata, sm_rdata;
    logic [8:0]       sm_waddr, sm_raddr;
    logic             sm_wen;
    sdp_bram #(.DEPTH(512),.WIDTH(SUM_W),.ABITS(9)) u_sm_bram (
        .clk(clk),.wr_en(sm_wen),.wr_addr(sm_waddr),.wr_data(sm_wdata),
        .rd_addr(sm_raddr),.rd_data(sm_rdata));

    logic [GRAD_W-1:0] pvg_wdata, pvg_rdata;
    logic [8:0]        pvg_waddr, pvg_raddr;
    logic              pvg_wen;
    sdp_bram #(.DEPTH(512),.WIDTH(GRAD_W),.ABITS(9)) u_pvg_bram (
        .clk(clk),.wr_en(pvg_wen),.wr_addr(pvg_waddr),.wr_data(pvg_wdata),
        .rd_addr(pvg_raddr),.rd_data(pvg_rdata));

    logic [GRAD_W-1:0] phg_wdata, phg_rdata;
    logic [8:0]        phg_waddr, phg_raddr;
    logic              phg_wen;
    sdp_bram #(.DEPTH(512),.WIDTH(GRAD_W),.ABITS(9)) u_phg_bram (
        .clk(clk),.wr_en(phg_wen),.wr_addr(phg_waddr),.wr_data(phg_wdata),
        .rd_addr(phg_raddr),.rd_data(phg_rdata));

    logic [17:0] edge_waddr, edge_raddr;
    logic        edge_wbit,  edge_rbit,  edge_wen;
    sdp_bram #(.DEPTH(262144),.WIDTH(1),.ABITS(18)) u_edge_bram (
        .clk(clk),.wr_en(edge_wen),.wr_addr(edge_waddr),.wr_data(edge_wbit),
        .rd_addr(edge_raddr),.rd_data(edge_rbit));

    // =========================================================================
    // Submodules
    // =========================================================================

    // --- S1: Projection ---
    logic [17:0] proj_fb_addr;
    projection_unit #(.IMG_W(IMG_W),.IMG_H(IMG_H)) u_proj (
        .clk(clk),.rst_n(rst_n),.en(state == S1_FRAME),
        .fb_addr(proj_fb_addr),.fb_data(fb_data),
        .pv_waddr(pv_waddr),.pv_wdata(pv_wdata),.pv_wen(pv_wen),
        .ph_waddr(ph_waddr),.ph_wdata(ph_wdata),.ph_wen(ph_wen),
        .done(frame_done));

    // --- S2: Smooth + grad Pv ---
    logic [8:0]   sg_src_raddr;
    logic [SUM_W-1:0] sg_src_rdata;
    logic         sg_done_pv, sg_done_ph;

    assign pv_raddr    = (state == S2_PROJ) ? sg_src_raddr : '0;
    assign ph_raddr    = (state == S3_GRAD) ? sg_src_raddr : '0;
    assign sg_src_rdata = (state == S2_PROJ) ? pv_rdata : ph_rdata;

    smooth_grad_unit #(.LEN(512),.DW(SUM_W),.GW(GRAD_W)) u_sg_pv (
        .clk(clk),.rst_n(rst_n),.en(state == S2_PROJ),
        .src_raddr(sg_src_raddr),.src_rdata(sg_src_rdata),
        .sm_waddr(sm_waddr),.sm_wdata(sm_wdata),.sm_wen(sm_wen),
        .sm_raddr(sm_raddr),.sm_rdata(sm_rdata),
        .grd_waddr(pvg_waddr),.grd_wdata(pvg_wdata),.grd_wen(pvg_wen),
        .done(sg_done_pv));

    smooth_grad_unit #(.LEN(512),.DW(SUM_W),.GW(GRAD_W)) u_sg_ph (
        .clk(clk),.rst_n(rst_n),.en(state == S3_GRAD),
        .src_raddr(sg_src_raddr),.src_rdata(sg_src_rdata),
        .sm_waddr(sm_waddr),.sm_wdata(sm_wdata),.sm_wen(sm_wen),
        .sm_raddr(sm_raddr),.sm_rdata(sm_rdata),
        .grd_waddr(phg_waddr),.grd_wdata(phg_wdata),.grd_wen(phg_wen),
        .done(sg_done_ph));

    assign proj_done = sg_done_pv;
    assign grad_done = sg_done_ph;

    // --- Bounds calc: runs during S3_GRAD, starts when Ph gradient is ready ---
    // Enabled at start of S3_GRAD; finishes (bounds_done) before FSM advances
    logic [8:0] xmin, xmax, ymin, ymax;

    bounds_calc u_bounds (
        .clk(clk),.rst_n(rst_n),.en(state == S3_GRAD),
        .pvg_raddr(pvg_raddr),.pvg_rdata(pvg_rdata),
        .phg_raddr(phg_raddr),.phg_rdata(phg_rdata),
        .xmin(xmin),.xmax(xmax),.ymin(ymin),.ymax(ymax),
        .done(bounds_done));   // << was empty in v1

    // --- S6: Prewitt edge detection ---
    logic [17:0] edge_fb_addr;
    prewitt_edge_unit u_prewitt (
        .clk(clk),.rst_n(rst_n),.en(state == S6_EDGE),
        .xmin(xmin),.xmax(xmax),.ymin(ymin),.ymax(ymax),
        .fb_addr(edge_fb_addr),.fb_data(fb_data),
        .edge_waddr(edge_waddr),.edge_wbit(edge_wbit),.edge_wen(edge_wen),
        .done(edge_done));

    // --- S5: Complexity ---
    logic [COMP_W-1:0] compl_scaled;
    complexity_unit #(.IMG_W(IMG_W),.COMP_W(COMP_W)) u_compl (
        .clk(clk),.rst_n(rst_n),.en(state == S5_CMPLX),
        .xmin(xmin),.xmax(xmax),.ymin(ymin),.ymax(ymax),
        .edge_raddr(edge_raddr),.edge_rbit(edge_rbit),
        .compl_scaled(compl_scaled),.done(cmplx_done));

    // =========================================================================
    // fb_addr mux
    // =========================================================================
    always_comb begin
        case (state)
            S1_FRAME : fb_addr = proj_fb_addr;
            S6_EDGE  : fb_addr = edge_fb_addr;
            default  : fb_addr = '0;
        endcase
    end

    // =========================================================================
    // Decision
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drowsy         <= '0;
            decision_valid <= '0;
        end else if (state == S4_DECID) begin
            drowsy         <= (compl_scaled < DROWSY_THRESH);
            decision_valid <= 1'b1;
        end else begin
            decision_valid <= '0;
        end
    end

endmodule
