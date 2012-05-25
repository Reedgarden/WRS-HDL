//
// Title          : RTU port interface for testebch
//
// File           : if_rtu.sv
// Author         : Maciej Lipinski
// Created        : 2012-05-23
// Standard       : SystemVerilog
//




interface IRTU
  (
   input clk_i,
   input rst_n_i
   );

   parameter g_mac_addr_width = 48;
   parameter g_vid_width      = 3;
   parameter g_prio_width     = 3;
   parameter g_rtu_num_ports  = 20;

   logic                         rtu_idle;      
   logic                         rq_strobe_p;
   logic [g_mac_addr_width-1:0 ] rq_smac;
   logic [g_mac_addr_width-1:0 ] rq_dmac;
   logic [g_vid_width     -1:0 ] rq_vid;
   logic                         rq_has_vid;
   logic [g_prio_width    -1:0 ] rq_prio;
   logic                         rq_has_prio;
   logic                         rsp_valid;
   logic [g_rtu_num_ports -1:0 ] rsp_dst_port_mask;
   logic                         rsp_drop;
   logic [g_prio_width    -1:0 ] rsp_prio;
   logic                         rsp_ack;
   logic                         port_almost_full;
   logic                         port_full;
   
   int cnt;
   int port_nr;
   
   task automatic init_port(input int portNum);
   
     port_nr     = portNum;
     rq_smac     <= 0;
     rq_dmac     <= 0;
     rq_vid      <= 0;
     rq_has_vid  <= 0;
     rq_prio     <= 0;
     rq_has_prio <= 0;
     rq_strobe_p <= 0;
     rq_strobe_p <= 1'b0;   
     rsp_ack     <= 1'b0;   
     @(posedge clk_i);
      
   endtask
   
   task automatic send_req
     (
       input  bit[7:0] dmac[],
       input  bit[7:0] smac[],
       input  bit[7:0] vid,
       input  bit      has_vid,
       input  bit[7:0] prio,
       input  bit      has_prio,
       input  bit      drop
     );
   
     rq_strobe_p <= 1'b0;
     @(posedge clk_i);
     
     rq_smac[47:40]     <= smac[0];
     rq_smac[39:32]     <= smac[1];
     rq_smac[31:24]     <= smac[2];
     rq_smac[23:16]     <= smac[3];
     rq_smac[15: 8]     <= smac[4];
     rq_smac[ 7: 0]     <= smac[5];

     rq_dmac[47:40]     <= dmac[0];
     rq_dmac[39:32]     <= dmac[1];
     rq_dmac[31:24]     <= dmac[2];
     rq_dmac[23:16]     <= dmac[3];
     rq_dmac[15: 8]     <= dmac[4];
     rq_dmac[ 7: 0]     <= dmac[5];   
     
     rq_vid      <= vid[g_vid_width  -1:0 ];
     rq_has_vid  <= has_vid;
     rq_prio     <= prio[g_prio_width-1:0 ];
     rq_has_prio <= has_prio;
     rq_strobe_p <= 1'b1;
     @(posedge clk_i);
     rq_strobe_p <= 1'b0;
   
   endtask   
   
   always@(posedge clk_i)
   begin
     //$display("rsp_valid = %d",rsp_valid);
     if(rsp_valid == 1'b1)
     begin
       $display("RTU.port[%2d] rsp: dmac=0x%x, drop=%d, prio=%d [time=%4d cycles, %4d ns]",
                 port_nr, rsp_dst_port_mask,rsp_drop,rsp_prio, cnt, cnt*16);
       rsp_ack <= 1'b1;
       @(posedge clk_i);
       rsp_ack <= 1'b0;
     end
   end
   
   initial forever
   begin
     @(posedge clk_i);

     //count how long a request is handled
     if(rq_strobe_p == 1) cnt = 0;
     else if(rtu_idle == 0) cnt++;
   
   end
   
endinterface // IRTU
