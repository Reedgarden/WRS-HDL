`timescale 1ns/1ps


module main;

   reg clk_ref=0;
   reg clk_sys=0;
   reg clk_swc_mpm_core=0;
   reg rst_n=0;
   parameter g_num_ports = 18;
   parameter g_top_in_bits   = 30;
   parameter g_top_out_bits  = 30;

   always #2.5ns clk_swc_mpm_core <=~clk_swc_mpm_core;
   always #8ns clk_sys <= ~clk_sys;
   always #8ns clk_ref <= ~clk_ref;
   
   reg [g_top_in_bits -1:0] input_vector;
   reg [g_top_out_bits-1:0] output_vector;
   
   initial begin
      repeat(100) @(posedge clk_sys);
      rst_n <= 1;
   end
/*
 *  wait ncycles
 */
    task automatic wait_cycles;
       input [31:0] ncycles;
       begin : wait_body
	  integer i;
 
	  for(i=0;i<ncycles;i=i+1) @(posedge clk_sys);
 
       end
    endtask // wait_cycles   
    
   module_test_top_bare
     #(
       .g_num_ports(g_num_ports),
       .g_top_in_bits(g_top_in_bits),
       .g_top_out_bits(g_top_out_bits)
       ) DUT (
              .sys_rst_n_i    (rst_n),
              .clk_startup_i  (clk_ref),
              .clk_ref_i      (clk_ref),
              .clk_dmtd_i     (clk_ref),
              .clk_aux_i      (clk_swc_mpm_core),

              .input_vector_i (input_vector),
              .output_vector_o(output_vector)
              );

  initial begin
    input_vector <= 0;
    wait_cycles(22);
    input_vector <= 1;
    wait_cycles(30);
    input_vector <= 10;

  end
endmodule // main

