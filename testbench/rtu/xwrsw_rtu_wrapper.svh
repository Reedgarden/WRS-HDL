`include "if_rtu_port.svh"
`include "if_wb_master.svh"
`include "if_wb_slave.svh"

// `define `c_mac_addr_width 48
// `define `c_vid_width      3
// `define `c_prio_width     3
// `define `c_port_mask_bits 15
// `define `c_prio_num       8
// `define `c_num_ports      16


`define array_assign(a, ah, al, b, bl) \
   for (k=al; k<=ah; k=k+1) begin assign a[k] = b[bl+k-al]; end



module swc_core_wrapper_generic
  (
   clk_i,
   rst_n_i
//    port,
//    rtu_wb
   );
   parameter g_prio_num       = 8;
   parameter g_prio_num_width = 3;
   parameter g_vid_num_width  = 3;
   parameter g_mac_addr_width = 48;
   parameter g_num_ports      = 16;  
   parameter g_port_mask_bits = 15;
   
   input clk_i;
   input rst_n_i;
   
   IWishboneMaster  #(32,32) rtu_wb (clk_i,rst_n_i);

   IRTU  #(g_mac_addr_width,
           g_vid_num_width,
           g_prio_num_width, 
           g_port_mask_bits) port[g_num_ports] (clk_i,rst_n_i);
   
   wire [g_num_ports                   -1 :0] rtu_idle;
   wire [g_num_ports                   -1 :0] rq_strobe_p;
   wire [g_num_ports*g_mac_addr_width  -1 :0] rq_smac;
   wire [g_num_ports*g_mac_addr_width  -1 :0] rq_dmac;
   wire [g_num_ports*g_vid_num_width   -1 :0] rq_vid;
   wire [g_num_ports                   -1 :0] rq_has_vid;
   wire [g_num_ports*g_prio_num_width  -1 :0] rq_prio;
   wire [g_num_ports                   -1 :0] rq_has_prio;

   wire [g_num_ports                   -1 :0] rsp_valid;
   wire [g_num_ports*g_port_mask_bits  -1 :0] rsp_dst_port_mask;
   wire [g_num_ports                   -1 :0] rsp_drop;
   wire [g_num_ports*g_prio_num_width  -1 :0] rsp_prio;
   wire [g_num_ports                   -1 :0] rsp_ack;

   wire [g_num_ports                   -1 :0] port_almost_full;
   wire [g_num_ports                   -1 :0] port_full;

   
   xwrsw_rtu_wrapper
     #(
       .g_prio_num                         (g_prio_num),
       .g_prio_num_width                   (g_prio_num_width),
       .g_vid_num_width                    (g_vid_num_width),
       .g_mac_addr_width                   (g_mac_addr_width),
       .g_num_ports                        (g_num_ports),
       .g_port_mask_bits                   (g_port_mask_bits)
       )DUT_RTU(
              .clk_i               (clk_i),
              .rst_n_i             (rst_n_i),

              .rtu_idle_o          (rtu_idle),
              .rq_strobe_p_i       (rq_strobe_p),
              .rq_smac_i           (rq_smac),
              .rq_dmac_i           (rq_dmac),
              .rq_vid_i            (rq_vid),
              .rq_has_vid_i        (rq_has_vid),
              .rq_prio_i           (rq_prio),
              .rq_has_prio_i       (rq_has_prio),

              .rsp_valid_o         (rsp_valid),
              .rsp_dst_port_mask_o (rsp_dst_port_mask),
              .rsp_drop_o          (rsp_drop),
              .rsp_prio_o          (rsp_prio),
              .rsp_ack_i           (rsp_ack),
              .port_almost_full_o  (port_almost_full),
              .port_full_o         (port_full),
              .wb_addr_i           (rtu_wb.master.adr),
              .wb_data_i           (rtu_wb.master.dat_o),
              .wb_data_o           (rtu_wb.master.dat_i),
              .wb_sel_i            (rtu_wb.master.sel),
              .wb_cyc_i            (rtu_wb.master.cyc),
              .wb_stb_i            (rtu_wb.master.stb),
              .wb_ack_o            (rtu_wb.master.ack),
//              .wb_irq_o            (),
              .wb_we_i             (rtu_wb.master.we)
              );
   genvar i, k;
   generate
     for(i=0;i<g_num_ports;i=i+1)
     begin

      `array_assign(rq_smac,(i+1)*g_mac_addr_width-1, i*g_mac_addr_width,port[i].rq_smac,0);
      `array_assign(rq_dmac,(i+1)*g_mac_addr_width-1, i*g_mac_addr_width,port[i].rq_dmac,0);
      `array_assign(rq_vid, (i+1)*g_vid_num_width -1, i*g_vid_num_width, port[i].rq_vid, 0);
      `array_assign(rq_prio,(i+1)*g_prio_num_width-1, i*g_prio_num_width,port[i].rq_prio,0);

      assign port[i].rtu_idle    = rtu_idle[i];
      assign rq_strobe_p[i]      = port[i].rq_strobe_p;
      assign rq_has_vid[i]       = port[i].rq_has_vid;
      assign rq_has_prio[i]      = port[i].rq_has_prio;

      assign port[i].rsp_valid        = rsp_valid[i];
      assign port[i].rsp_drop         = rsp_drop[i];
      assign rsp_ack[i]               = port[i].rsp_ack;
      assign port[i].port_almost_full = port_almost_full[i];
      assign port[i].port_full        = port_full[i];
      
      `array_assign(port[i].rsp_dst_port_mask,g_port_mask_bits-1,0,rsp_dst_port_mask,i*g_port_mask_bits);
      `array_assign(port[i].rsp_prio,         g_prio_num_width-1,0,rsp_prio,         i*g_prio_num_width);

     end //for(i=0;i<g_num_ports;i=i+1)
   endgenerate

endmodule 
