`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/03/13 10:10:53
// Design Name: 
// Module Name: MIBus_if
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


interface MIBus_if #(parameter WIDTH=64);
    logic [WIDTH-1:0] data;
    logic             valid;
    logic             ctrl_flag;
    logic [3:0]       adjust_term;
    
    modport master (output data, valid, input ctrl_flag);
    modport slave  (input  data, valid, output ctrl_flag);
endinterface 
