`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/12 20:51:20
// Design Name: 
// Module Name: KalmanFilterTop_bd
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module KalmanFilterTop_bd (
  input  wire clk,
  input  wire rst_n,
  input  wire start,
  output wire filter_done,

  // m0 read
  output wire [31:0]  m0_axi_araddr,
  output wire [7:0]   m0_axi_arlen,
  output wire [2:0]   m0_axi_arsize,
  output wire [1:0]   m0_axi_arburst,
  output wire         m0_axi_arvalid,
  input  wire         m0_axi_arready,
  input  wire [511:0] m0_axi_rdata,
  input  wire         m0_axi_rvalid,
  output wire         m0_axi_rready,

  // m1 read
  output wire [31:0]  m1_axi_araddr,
  output wire [7:0]   m1_axi_arlen,
  output wire [2:0]   m1_axi_arsize,
  output wire [1:0]   m1_axi_arburst,
  output wire         m1_axi_arvalid,
  input  wire         m1_axi_arready,
  input  wire [511:0] m1_axi_rdata,
  input  wire         m1_axi_rvalid,
  output wire         m1_axi_rready,

  // m2 write
  output wire [31:0]  m2_axi_awaddr,
  output wire [7:0]   m2_axi_awlen,
  output wire [2:0]   m2_axi_awsize,
  output wire [1:0]   m2_axi_awburst,
  output wire         m2_axi_awvalid,
  input  wire         m2_axi_awready,
  output wire [511:0] m2_axi_wdata,
  output wire [63:0]  m2_axi_wstrb,
  output wire         m2_axi_wvalid,
  input  wire         m2_axi_wready,
  output wire         m2_axi_wlast,
  input  wire [1:0]   m2_axi_bresp,
  input  wire         m2_axi_bvalid,
  output wire         m2_axi_bready,

  // legacy (unused)
  input  wire [63:0]  z_data,
  input  wire         z_valid
);

  KalmanFilterTop u_top (
    .clk(clk), .rst_n(rst_n), .start(start), .filter_done(filter_done),

    .m0_axi_araddr(m0_axi_araddr), .m0_axi_arlen(m0_axi_arlen),
    .m0_axi_arsize(m0_axi_arsize), .m0_axi_arburst(m0_axi_arburst),
    .m0_axi_arvalid(m0_axi_arvalid), .m0_axi_arready(m0_axi_arready),
    .m0_axi_rdata(m0_axi_rdata), .m0_axi_rvalid(m0_axi_rvalid), .m0_axi_rready(m0_axi_rready),

    .m1_axi_araddr(m1_axi_araddr), .m1_axi_arlen(m1_axi_arlen),
    .m1_axi_arsize(m1_axi_arsize), .m1_axi_arburst(m1_axi_arburst),
    .m1_axi_arvalid(m1_axi_arvalid), .m1_axi_arready(m1_axi_arready),
    .m1_axi_rdata(m1_axi_rdata), .m1_axi_rvalid(m1_axi_rvalid), .m1_axi_rready(m1_axi_rready),

    .m2_axi_awaddr(m2_axi_awaddr), .m2_axi_awlen(m2_axi_awlen),
    .m2_axi_awsize(m2_axi_awsize), .m2_axi_awburst(m2_axi_awburst),
    .m2_axi_awvalid(m2_axi_awvalid), .m2_axi_awready(m2_axi_awready),
    .m2_axi_wdata(m2_axi_wdata), .m2_axi_wstrb(m2_axi_wstrb),
    .m2_axi_wvalid(m2_axi_wvalid), .m2_axi_wready(m2_axi_wready),
    .m2_axi_wlast(m2_axi_wlast),
    .m2_axi_bresp(m2_axi_bresp), .m2_axi_bvalid(m2_axi_bvalid), .m2_axi_bready(m2_axi_bready),

    .z_data(z_data), .z_valid(z_valid)
  );

endmodule
