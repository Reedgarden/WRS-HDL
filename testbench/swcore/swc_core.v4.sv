// Fabric emulator example, showing 2 fabric emulators connected together and exchanging packets.

`define c_clock_period        8
`define c_swc_page_addr_width 10
`define c_swc_usecount_width  4 
`define c_wrsw_prio_width     3
`define c_swc_ctrl_width      4
`define c_swc_data_width      16
`define c_wrsw_num_ports      11
//`define c_wrsw_num_ports      7


`timescale 1ns / 1ps

`include "fabric_emu.sv"


`define array_copy(a, ah, al, b, bl) \
   for (k=al; k<=ah; k=k+1) a[k] <= b[bl+k-al];


typedef struct {
   int cnt;
   int usecnt[10];
   int port[10];

} alloc_info_t;

alloc_info_t alloc_table[1024];
alloc_info_t dealloc_table[1024];

int stack_bastard = 0;

int pg_alloc_cnt[1024][20];
int pg_dealloc_cnt[1024][20];

module main;

   
   reg clk 		       = 0;
   reg rst_n 		     = 0;


   wire [`c_wrsw_num_ports * 16- 1:0] a_to_input_block_data;
   wire [`c_wrsw_num_ports * 4 - 1:0] a_to_input_block_ctrl;
   wire [`c_wrsw_num_ports-1:0]       a_to_input_block_bytesel;
   wire [`c_wrsw_num_ports-1:0]       a_to_input_block_dreq;
   wire [`c_wrsw_num_ports-1:0]       a_to_input_block_valid;
   wire [`c_wrsw_num_ports-1:0]       a_to_input_block_sof_p1;
   wire [`c_wrsw_num_ports-1:0]       a_to_input_block_eof_p1;
   wire [`c_wrsw_num_ports-1:0]       a_to_input_block_rerror_p1;
   wire [`c_wrsw_num_ports-1:0]       a_to_input_block_abort_p1;
   wire [`c_wrsw_num_ports-1:0]       a_to_input_block_terror_p1;
   wire [`c_wrsw_num_ports-1:0]       a_to_input_block_tabort_p1;


   wire [`c_wrsw_num_ports * 16- 1:0] input_block_to_a_data;
   wire [`c_wrsw_num_ports * 4 - 1:0] input_block_to_a_ctrl;
   wire [`c_wrsw_num_ports-1:0]       input_block_to_a_bytesel;
   wire [`c_wrsw_num_ports-1:0]       input_block_to_a_dreq;
   wire [`c_wrsw_num_ports-1:0]       input_block_to_a_valid;
   wire [`c_wrsw_num_ports-1:0]       input_block_to_a_sof_p1;
   wire [`c_wrsw_num_ports-1:0]       input_block_to_a_eof_p1;
   wire [`c_wrsw_num_ports-1:0]       input_block_to_a_rerror_p1;
   wire [`c_wrsw_num_ports-1:0]       input_block_to_a_idle;
   wire [`c_wrsw_num_ports-1:0]       input_block_to_a_terror_p1;
   wire [`c_wrsw_num_ports-1:0]       input_block_to_a_tabort_p1;
  
   `WRF_WIRES(ab); // Emu A to B fabric
   `WRF_WIRES(ba); // And the other way around
    
   reg  [`c_wrsw_num_ports-1:0]                         rtu_rsp_valid        = 0;     
   wire [`c_wrsw_num_ports-1:0]                         rtu_rsp_ack;       
   reg  [`c_wrsw_num_ports * `c_wrsw_num_ports - 1 : 0] rtu_dst_port_mask    = 0; 
   reg  [`c_wrsw_num_ports-1:0]                         rtu_drop             = 0;          
   reg  [`c_wrsw_num_ports * `c_wrsw_prio_width -1 : 0] rtu_prio             = 0;     

   // generate clock and reset signals
   always #(`c_clock_period/2) clk <= ~clk;
   
   initial begin 
      repeat(3) @(posedge clk);
      rst_n  = 1;
   end
 
   int tx_cnt_0[11];
   int tx_cnt_1[11];
   int tx_cnt_2[11];
   int tx_cnt_3[11];
   int tx_cnt_4[11];
   int tx_cnt_5[11];
   int tx_cnt_6[11];
   int tx_cnt_7[11];
   int tx_cnt_8[11];
   int tx_cnt_9[11];
   int tx_cnt_10[11];

   int rx_cnt[11];
   
   int tx_cnt_by_port[11][11];
   int rx_cnt_by_port[11][11];
   
   int rx_cnt_0[11];
   int rx_cnt_1[11];
   int rx_cnt_2[11];
   int rx_cnt_3[11];
   int rx_cnt_4[11];
   int rx_cnt_5[11];
   int rx_cnt_6[11];
   int rx_cnt_7[11];
   int rx_cnt_8[11];
   int rx_cnt_9[11];
   int rx_cnt_10[11];
 
 
   bit [10:0] tx_port_finished = 0;//{0,0,0,0,0,0,0,0,0,0,0};
 
   integer ports_read = 0;

   
   swc_core 
    DUT (
    .clk_i                 (clk),
    .rst_n_i               (rst_n),
//-------------------------------------------------------------------------------
//-- Fabric I/F  
//-------------------------------------------------------------------------------  
    .tx_sof_p1_i         (a_to_input_block_sof_p1),
    .tx_eof_p1_i         (a_to_input_block_eof_p1),
    .tx_data_i           (a_to_input_block_data),
    .tx_ctrl_i           (a_to_input_block_ctrl),
    .tx_valid_i          (a_to_input_block_valid),
    .tx_bytesel_i        (a_to_input_block_bytesel),
    .tx_dreq_o           (a_to_input_block_dreq),
    .tx_abort_p1_i       (a_to_input_block_abort_p1),
    .tx_rerror_p1_i      (a_to_input_block_rerror_p1),

//-------------------------------------------------------------------------------
//-- Fabric I/F : output (goes to the Endpoint)
//-------------------------------------------------------------------------------  

    .rx_sof_p1_o         (input_block_to_a_sof_p1),
    .rx_eof_p1_o         (input_block_to_a_eof_p1),
    .rx_dreq_i           (input_block_to_a_dreq),
    .rx_ctrl_o           (input_block_to_a_ctrl),
    .rx_data_o           (input_block_to_a_data),
    .rx_valid_o          (input_block_to_a_valid),
    .rx_bytesel_o        (input_block_to_a_bytesel),
    .rx_idle_o           (input_block_to_a_idle),
    .rx_rerror_p1_o      (input_block_to_a_rerror_p1),
    .rx_terror_p1_i      (input_block_to_a_terror_p1),
    .rx_tabort_p1_i      (input_block_to_a_tabort_p1),// tx_rabort_p1_i ????????

       

//-------------------------------------------------------------------------------
//-- I/F with Routing Table Unit (RTU)
//-------------------------------------------------------------------------------      
    
    .rtu_rsp_valid_i       (rtu_rsp_valid),
    .rtu_rsp_ack_o         (rtu_rsp_ack),
    .rtu_dst_port_mask_i   (rtu_dst_port_mask),
    .rtu_drop_i            (rtu_drop),
    .rtu_prio_i            (rtu_prio)
    );
    
    task automatic wait_cycles;
       input [31:0] ncycles;
       begin : wait_body
    integer i;
 
    for(i=0;i<ncycles;i=i+1) @(posedge clk);
 
       end
    endtask // wait_cycles
    


   fabric_emu test_input_block_0
     (
      .clk_i(clk),
      .rst_n_i(rst_n),
      `WRF_CONNECT_SOURCE_ML(rx, a_to_input_block,0),

/*
      .rx_data_o      (a_to_input_block_data[15:0]),
      .rx_ctrl_o      (a_to_input_block_ctrl[15:0]),
      .rx_bytesel_o   (a_to_input_block_bytesel[0]),
      .rx_dreq_i      (a_to_input_block_dreq[0]),
      .rx_valid_o     (a_to_input_block_valid[0]),
      .rx_sof_p1_o    (a_to_input_block_sof_p1[0]),
      .rx_eof_p1_o    (a_to_input_block_eof_p1[0]),
      .rx_rerror_p1_o (a_to_input_block_rerror_p1[0]),
      .rx_terror_p1_i (1'b0),
      .rx_tabort_p1_o (),
      .rx_rabort_p1_i (1'b0),
      .rx_idle_o      (),
 */
      `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,0)

/*      
      .tx_data_i      (input_block_to_a_data[15:0]),
      .tx_ctrl_i      (input_block_to_a_ctrl[15:0]),
      .tx_bytesel_i   (input_block_to_a_bytesel[0]),
      .tx_dreq_o      (input_block_to_a_dreq[0]),
      .tx_valid_i     (input_block_to_a_valid[0]),
      .tx_sof_p1_i    (input_block_to_a_sof_p1[0]),
      .tx_eof_p1_i    (input_block_to_a_eof_p1[0]),
      .tx_rerror_p1_i (input_block_to_a_rerror_p1[0]),
      .tx_terror_p1_o (input_block_to_a_terror_p1[0]),
      .tx_rabort_p1_o (input_block_to_a_rabort_p1[0]),
      .tx_idle_i      (input_block_to_a_idle[0])
*/
      );
   fabric_emu test_input_block_1
     (
      .clk_i(clk),
      .rst_n_i(rst_n),
      `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,1),
      `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,1)
      );
   fabric_emu test_input_block_2
        (
         .clk_i(clk),
         .rst_n_i(rst_n),
         `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,2),
         `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,2)
         );
   fabric_emu test_input_block_3
     (
      .clk_i(clk),
      .rst_n_i(rst_n),
      `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,3),
      `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,3)
      );
   fabric_emu test_input_block_4
        (
         .clk_i(clk),
         .rst_n_i(rst_n),
         `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,4),
         `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,4)
         );  
   fabric_emu test_input_block_5
     (
      .clk_i(clk),
      .rst_n_i(rst_n),
      `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,5),
      `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,5)
      );
   fabric_emu test_input_block_6
        (
         .clk_i(clk),
         .rst_n_i(rst_n),
         `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,6),
         `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,6)
         );
   fabric_emu test_input_block_7
     (
      .clk_i(clk),
      .rst_n_i(rst_n),
      `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,7),
      `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,7)
      );
   fabric_emu test_input_block_8
        (
         .clk_i(clk),
         .rst_n_i(rst_n),
         `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,8),
         `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,8)
         );   
         
       
   fabric_emu test_input_block_9
     (
      .clk_i(clk),
      .rst_n_i(rst_n),
      `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,9),
      `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,9)
      );
   fabric_emu test_input_block_10
        (
         .clk_i(clk),
         .rst_n_i(rst_n),
         `WRF_FULL_CONNECT_SOURCE_ML(rx, a_to_input_block,10),
         `WRF_FULL_CONNECT_SINK_ML(tx, input_block_to_a,10)
         );     
         
         
    task automatic set_rtu_rsp;
       input [31:0]                    chan;
       input                           valid;
       input                           drop;
       input [`c_wrsw_num_ports - 1:0] prio;
       input [`c_wrsw_num_ports - 1:0] mask;
       
       begin : wait_body
    integer i;
    integer k; // for the macro array_copy()

    `array_copy(rtu_dst_port_mask,(chan+1)*`c_wrsw_num_ports  - 1, chan*`c_wrsw_num_ports,  mask ,0); 
    `array_copy(rtu_prio         ,(chan+1)*`c_wrsw_prio_width - 1, chan*`c_wrsw_prio_width, prio, 0); 
    
    rtu_drop         [ chan ]                                                = drop;          
    rtu_rsp_valid    [ chan ]                                                = valid;
 
       end
    endtask // wait_cycles         
       
   task automatic send_pck;
      input ether_header_t            hdr; 
      input int                       payload[]; 
      input int                       length;
      input [31:0]                    port;
      input                           drop;
      input [`c_wrsw_num_ports - 1:0] prio;
      input [`c_wrsw_num_ports - 1:0] mask;    
      input int                       frame_number;
      
      begin : send_pck_body            
        int j;
        set_rtu_rsp(port,1,drop,prio,mask);  
        
    //    $display("Sending [f_nr: %2d]: src = %x, port = %d, len = %d, drop = %d, prio = %d, mask = %x",frame_number,hdr.src, port,length, drop, prio, mask);
    
    
      case(port)
         0: test_input_block_0.send(hdr, payload, length);
         1: test_input_block_1.send(hdr, payload, length);
         2: test_input_block_2.send(hdr, payload, length);
         3: test_input_block_3.send(hdr, payload, length);
         4: test_input_block_4.send(hdr, payload, length);
         5: test_input_block_5.send(hdr, payload, length);
         6: test_input_block_6.send(hdr, payload, length);
         7: test_input_block_7.send(hdr, payload, length);
         8: test_input_block_8.send(hdr, payload, length);
         9: test_input_block_9.send(hdr, payload, length);
         10:test_input_block_10.send(hdr, payload, length);
         default: $display("ERROR: Wrong port number !!!");
       endcase   
       
       if(drop == 0 && mask != 0)
       begin
         for(j=0;j<11;j++)
         begin
           if(mask[j]) 
           begin 
               
             tx_cnt_by_port[port][j]++; 
             case(port)
                0: tx_cnt_0[j]++;
                1: tx_cnt_1[j]++;
                2: tx_cnt_2[j]++;
                3: tx_cnt_3[j]++;
                4: tx_cnt_4[j]++;
                5: tx_cnt_5[j]++;
                6: tx_cnt_6[j]++;
                7: tx_cnt_7[j]++;
                8: tx_cnt_8[j]++;
                9: tx_cnt_9[j]++;
                10:tx_cnt_10[j]++;
                default: $display("ERROR: Wrong port number !!!");
              endcase 
               
           end
         end
       end
       
      end
   endtask
           
           
   task automatic load_port;
      input [31:0]              port;
      
      begin : load_port_body
                 
        ether_header_t hdr;
        int buffer[2024];
        int i,j;
        int cnt =0;
        bit [10:0] mask ;
        int drop;

        hdr.src 	       = 'h123456789abcdef;
        hdr.dst 	       = 'hcafeb1badeadbef;
        hdr.ethertype    = 1234;
        hdr.is_802_1q    = 0;
        hdr.oob_type     = `OOB_TYPE_RXTS;
        hdr.timestamp_r  = 10000;
        hdr.timestamp_f  = 4;
        
        for(i=0;i<2000;i++)
            buffer[i]      = i;
        

        
        $display("Initial waiting: %d cycles",((port*50)%11)*50);
        wait_cycles(((port*50)%11)*50);
        
        for(i=400;i<1250;i=i+25)
        begin
          hdr.src          = port << 44 | cnt ;
          hdr.port_id      = port;
          hdr.ethertype    = i;

       //  mask = 'h7FF;;
       //   mask = 'h00f;;
          mask = (port*cnt + 3*cnt + 2*cnt + cnt)%2047; 
          if( (i/50)%20 > 10) drop = 1; else drop = 0;
       //   drop = 0;
          
          
          send_pck(hdr,buffer, i, port, drop,  ((port*i)/50)%7, mask, cnt);  
       //   if(port == 7) $display("====>>> Port 7: send pck nr %d",cnt);
          //if(port == 6) $display("====>>> Port 6: send pck nr %d",cnt);
          if(i > 600) 
          begin
            
            test_input_block_0.simulate_rx_abort(0,10);
            test_input_block_1.simulate_rx_abort(0,20);
            test_input_block_2.simulate_rx_abort(0,30);
            test_input_block_3.simulate_rx_abort(0,40);
            test_input_block_4.simulate_rx_abort(0,50);
            test_input_block_5.simulate_rx_abort(0,60);
            test_input_block_6.simulate_rx_abort(0,70);
            test_input_block_7.simulate_rx_abort(0,80);
            test_input_block_8.simulate_rx_abort(0,90);
            test_input_block_9.simulate_rx_abort(0,100);
            test_input_block_10.simulate_rx_abort(0,110);
            
         end 
          
          cnt ++;    
        end

        $display(" ");
        $display("==>> FINISHED: %2d  !!!!!",port);
        $display(" ");        
      end
   endtask      
                
   initial begin
     
      ports_read = 0;
      wait(test_input_block_0.ready);
      wait(test_input_block_1.ready);
      wait(test_input_block_2.ready);
      wait(test_input_block_3.ready);
      wait(test_input_block_4.ready);
      wait(test_input_block_5.ready);
      wait(test_input_block_6.ready);
      wait(test_input_block_7.ready);
      wait(test_input_block_8.ready);
      wait(test_input_block_9.ready);
      wait(test_input_block_10.ready);

 
    //////////////// input thottling ///////////////
    
     test_input_block_0.simulate_tx_throttling(1, 50);
     test_input_block_1.simulate_tx_throttling(1, 20);
     test_input_block_2.simulate_tx_throttling(1, 30);
     test_input_block_3.simulate_tx_throttling(1, 40);
     test_input_block_4.simulate_tx_throttling(1, 10);
     test_input_block_5.simulate_tx_throttling(1, 20);
     test_input_block_6.simulate_tx_throttling(1, 30);
     test_input_block_7.simulate_tx_throttling(1, 40);
     test_input_block_8.simulate_tx_throttling(1, 10);
     test_input_block_9.simulate_tx_throttling(1, 20);
     test_input_block_10.simulate_tx_throttling(1,40);



     //////////////// output thottling ///////////////
     
     test_input_block_0.simulate_rx_throttling(1, 10);
     test_input_block_1.simulate_rx_throttling(1, 20);
     test_input_block_2.simulate_rx_throttling(1, 10);
     test_input_block_3.simulate_rx_throttling(1, 20);
     test_input_block_4.simulate_rx_throttling(1, 10);
     test_input_block_5.simulate_rx_throttling(1, 20);
     test_input_block_6.simulate_rx_throttling(1, 10);
     test_input_block_7.simulate_rx_throttling(1, 20);
     test_input_block_8.simulate_rx_throttling(1, 10);
     test_input_block_9.simulate_rx_throttling(1, 20);
     test_input_block_10.simulate_rx_throttling(1, 10);


     test_input_block_0.simulate_rx_abort(1,10);
     test_input_block_1.simulate_rx_abort(1,20);
     test_input_block_2.simulate_rx_abort(1,30);
     test_input_block_3.simulate_rx_abort(1,40);
     test_input_block_4.simulate_rx_abort(1,50);
     test_input_block_5.simulate_rx_abort(1,60);
     test_input_block_6.simulate_rx_abort(1,70);
     test_input_block_7.simulate_rx_abort(1,80);
     test_input_block_8.simulate_rx_abort(1,90);
     test_input_block_9.simulate_rx_abort(1,100);
     test_input_block_10.simulate_rx_abort(1,110);



    //////////////// input error ///////////////
 /*   
     test_input_block_0.simulate_tx_error(1,90);
     test_input_block_1.simulate_tx_error(1,110);
     test_input_block_2.simulate_tx_error(1,130);
     test_input_block_3.simulate_tx_error(1,140);
     test_input_block_4.simulate_tx_error(1,150);
     test_input_block_5.simulate_tx_error(1,160);
     test_input_block_6.simulate_tx_error(1,170);
     test_input_block_7.simulate_tx_error(1,180);
     test_input_block_8.simulate_tx_error(1,190);
     test_input_block_9.simulate_tx_error(1,100);
     test_input_block_10.simulate_tx_error(1,800);
 */
 

      
      
      ports_read = 1;
      
      $display("=======ALL PORTS READY======");

      
    
   end
   
   initial begin
      int i;
      wait(ports_read);
      load_port(0);
      tx_port_finished[0] = 1;
   end
   
   initial begin
      wait(ports_read);
      load_port(1);
      tx_port_finished[1] = 1;
   end
   
   initial begin
      wait(ports_read);
      load_port(2);
      tx_port_finished[2] = 1;
   end

   initial begin
      wait(ports_read);
      load_port(3);
      tx_port_finished[3] = 1;
   end

   initial begin
      wait(ports_read);
      load_port(4);
      tx_port_finished[4] = 1;
   end
   initial begin
     wait(ports_read);
     load_port(5);
     tx_port_finished[5] = 1;
   end
   initial begin
       wait(ports_read);
      load_port(6);
      tx_port_finished[6] = 1;
   end
   initial begin
      wait(ports_read);
      load_port(7);
      tx_port_finished[7] = 1;
   end
   initial begin
      wait(ports_read);
      load_port(8);
      tx_port_finished[8] = 1;
   end
   initial begin
      wait(ports_read);
      load_port(9);
      tx_port_finished[9] = 1;
   end
   initial begin
      wait(ports_read);
     load_port(10);
      tx_port_finished[10] = 1;
   end  
        
           
   initial begin
     int i,j, cnt;
     int sum_rx, sum_tx, sum_tx_by_port[11],sum_rx_by_port[11];

     wait_cycles(100000);
     
//     wait(tx_port_finished[0] & tx_port_finished[1] & tx_port_finished[2] & tx_port_finished[3] & tx_port_finished[4] & 
//          tx_port_finished[5] & tx_port_finished[6] & tx_port_finished[7] & tx_port_finished[8] & tx_port_finished[9] & tx_port_finished[10]);

//     wait_cycles(1000000);
     
     $display("=============================================== DBG =================================================");
     $display("Rx Ports   :  P 0  |  P 1  |  P 2  |  P 3  |  P 4  |  P 5  |  P 6  |  P 7  |  P 8  |  P 9  |  P10  | ");
     $display("-----------------------------------------------------------------------------------------------------");
     $display(" (number of pcks sent from port Rx to port Tx) > (number of pcks received on port Tx from port Rx) | ");
     $display("-----------------------------------------------------------------------------------------------------");
     for(i=0;i<11;i++)

        $display("TX Port %2d : %2d>%2d | %2d>%2d | %2d>%2d | %2d>%2d | %2d>%2d | %2d>%2d | %2d>%2d | %2d>%2d | %2d>%2d | %2d>%2d | %2d>%2d |",i,
        tx_cnt_by_port[i][0],rx_cnt_by_port[i][0],tx_cnt_by_port[i][1],rx_cnt_by_port[i][1],tx_cnt_by_port[i][2],rx_cnt_by_port[i][2],tx_cnt_by_port[i][3],rx_cnt_by_port[i][3],
        tx_cnt_by_port[i][4],rx_cnt_by_port[i][4],tx_cnt_by_port[i][5],rx_cnt_by_port[i][5],tx_cnt_by_port[i][6],rx_cnt_by_port[i][6],tx_cnt_by_port[i][7],rx_cnt_by_port[i][7],
        tx_cnt_by_port[i][8],rx_cnt_by_port[i][8],tx_cnt_by_port[i][9],rx_cnt_by_port[i][9],tx_cnt_by_port[i][10],rx_cnt_by_port[i][10]);
        
     
     $display("=============================================== DBG =================================================");
     
     for(i=0;i<11;i++)
       begin
         for(j=0;j<11;j++) sum_tx_by_port[i] += tx_cnt_by_port[j][i];

         $display("Tx Port %2d : pcks sent to P%2d = %2d, pcks received on P%2d = %2d",i,i, sum_tx_by_port[i],i ,rx_cnt[i]);
       end

     for(i=0;i<11;i++) sum_tx += sum_tx_by_port[i];
     for(i=0;i<11;i++) sum_rx += rx_cnt[i];

     $display("=======================================================================");
     $display("SUM    :  sent pcks = %2d, received pcks = %2d", sum_tx,sum_rx);
     $display("=================================== DBG ===============================");

/*
     cnt =0;
     for(i=0;i<1024;i++)
       if(pg_dealloc_cnt[i][0] != pg_alloc_cnt[i][0])
         begin
           $display("Page %3d: allocated = %2d [%1d|%1d|%1d|%1d|%1d|%1d]  <=|=>   deallocated = %2d [%1d|%1d|%1d|%1d|%1d|%1d] <- lost page !!!!",i,pg_alloc_cnt[i][0],
           pg_alloc_cnt[i][1], pg_alloc_cnt[i][2],pg_alloc_cnt[i][3],pg_alloc_cnt[i][4],pg_alloc_cnt[i][5],pg_alloc_cnt[i][6], pg_dealloc_cnt[i][0],
           pg_dealloc_cnt[i][1],pg_dealloc_cnt[i][2],pg_dealloc_cnt[i][3],pg_dealloc_cnt[i][4],pg_dealloc_cnt[i][5],pg_dealloc_cnt[i][6]);
           cnt++;
         end
        
     $display("=======================================================================");
     $display("MEM LEAKGE Report:  number of lost pages = %2d", cnt);
     $display("=================================== DBG ===============================");

*/
     cnt =0;
     for(i=0;i<1024;i++)
       if(dealloc_table[i].cnt!= alloc_table[i].cnt)
         begin
           $display("Page %4d: alloc = %2d [%2d:%2d|%2d:%2d|%2d:%2d|%2d:%2d|%2d:%2d|%2d:%2d]<=|=>dealloc = %2d [%2d:%11b|%2d:%11b|%2d:%11b|%2d:%11b|%2d:%11b|%2d:%11b]  ",
           i,
           alloc_table[i].cnt,
           alloc_table[i].usecnt[0], alloc_table[i].port[0], 
           alloc_table[i].usecnt[1], alloc_table[i].port[1],
           alloc_table[i].usecnt[2], alloc_table[i].port[2],
           alloc_table[i].usecnt[3], alloc_table[i].port[3],
           alloc_table[i].usecnt[4], alloc_table[i].port[4],
           alloc_table[i].usecnt[5], alloc_table[i].port[5],
           dealloc_table[i].cnt,
           dealloc_table[i].usecnt[0], dealloc_table[i].port[0],
           dealloc_table[i].usecnt[1], dealloc_table[i].port[1],
           dealloc_table[i].usecnt[2], dealloc_table[i].port[2],
           dealloc_table[i].usecnt[3], dealloc_table[i].port[3],
           dealloc_table[i].usecnt[4], dealloc_table[i].port[4],
           dealloc_table[i].usecnt[5], dealloc_table[i].port[5]);
           cnt++;
         end
        
     $display("=======================================================================");
     if(cnt == 22)
       $display("%4d pages allocated in advance (port X start_of_pck + port X pck_internal pages)", cnt);
     else
       $display("MEM LEAKGE Report:  number of lost pages = %2d", (cnt - 22));
     $display("=================================== DBG ===============================");


 $fatal("dupa");
   end                     
//////////////////////////////////////////////////////////


   always @(posedge clk) 
     begin
       

       
       int i;
       for(i = 0;i<11;i++)
       begin
         rtu_rsp_valid[i] = rtu_rsp_valid[i] & !rtu_rsp_ack[i];
         rtu_drop[i]      = rtu_drop[i]      & !rtu_rsp_ack[i];
       end
       
     end
   
   
   // Check if there's anything received by EMU B
   always @(posedge clk) if (test_input_block_0.poll())
     begin
    	ether_frame_t frame;
 
	   test_input_block_0.receive(frame);
	   rx_cnt[0]++;
	   rx_cnt_0[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][0]++;
//	   dump_frame_header("Receiving RX_0: ", frame);
	   end
	   
   always @(posedge clk) if (test_input_block_1.poll())
     begin
    	ether_frame_t frame;
 
	   test_input_block_1.receive(frame);
	   rx_cnt[1]++;
	   rx_cnt_1[frame.hdr.src >> 44]++;
     rx_cnt_by_port[frame.hdr.src >> 44][1]++;;
//	   dump_frame_header("Receiving RX_1: ", frame);
	   end      
   always @(posedge clk) if (test_input_block_2.poll())
     begin
    	ether_frame_t frame;
 
	   test_input_block_2.receive(frame);
	   rx_cnt[2]++;
	   rx_cnt_2[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][2]++;
	//   dump_frame_header("Receiving RX_2: ", frame);
	   end
	   
   always @(posedge clk) if (test_input_block_3.poll())
     begin
    	ether_frame_t frame;
 
	   test_input_block_3.receive(frame);
	   rx_cnt[3]++;
	   rx_cnt_3[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][3]++;
//	   dump_frame_header("Receiving RX_3: ", frame);
	   end      
   always @(posedge clk) if (test_input_block_4.poll())
     begin
    	ether_frame_t frame;
 
	   test_input_block_4.receive(frame);
	   rx_cnt[4]++;
	   rx_cnt_4[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][4]++;
//	   dump_frame_header("Receiving RX_4: ", frame);
	   end
	   
   always @(posedge clk) if (test_input_block_5.poll())
     begin
    	ether_frame_t frame;

 
	   test_input_block_5.receive(frame);
	   rx_cnt[5]++;
	   rx_cnt_5[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][5]++;
//	   dump_frame_header("Receiving RX_5: ", frame);
	   end      
   always @(posedge clk) if (test_input_block_6.poll())
     begin
    	ether_frame_t frame;

 
	   test_input_block_6.receive(frame);
	   rx_cnt[6]++;
	   rx_cnt_6[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][6]++;
//	   dump_frame_header("Receiving RX_6: ", frame);
	   end
	   
   always @(posedge clk) if (test_input_block_7.poll())
     begin
    	ether_frame_t frame;
 
	   test_input_block_7.receive(frame);
	   rx_cnt[7]++;
	   rx_cnt_7[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][7]++;
	//   dump_frame_header("Receiving RX_7: ", frame);
	   end  
      
     always @(posedge clk) if (test_input_block_8.poll())
     begin
    	ether_frame_t frame;
 
	   test_input_block_8.receive(frame);
	   rx_cnt[8]++;
	   rx_cnt_8[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][8]++;
	//   dump_frame_header("Receiving RX_8: ", frame);
	   end
   
   always @(posedge clk) if (test_input_block_9.poll() )
     begin
    	ether_frame_t frame;

 
	   test_input_block_9.receive(frame);
	   rx_cnt[9]++;
	   rx_cnt_9[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][9]++;
//	   dump_frame_header("Receiving RX_9: ", frame);
	   end      
   always @(posedge clk) if (test_input_block_10.poll() )
     begin
    	ether_frame_t frame;
 
	   test_input_block_10.receive(frame);
	   rx_cnt[10]++;
	   rx_cnt_10[frame.hdr.src >> 44]++;
	   rx_cnt_by_port[frame.hdr.src >> 44][10]++; 
//	   dump_frame_header("Receiving RX_10: ", frame);
	   end      

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////// Monitoring allocation of pages  /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
//   always @(posedge clk) if(DUT.memory_management_unit.pg_addr_valid)
   always @(posedge clk) if(DUT.memory_management_unit.pg_alloc & DUT.memory_management_unit.pg_done)

     begin
     int address;  
     int usecnt;
     
     usecnt = DUT.memory_management_unit.pg_usecnt;
     
     wait(DUT.memory_management_unit.pg_addr_valid);
     
     address =  DUT.memory_management_unit.pg_addr_alloc;
     pg_alloc_cnt[address][pg_alloc_cnt[address][0]+1]= usecnt;
     pg_alloc_cnt[address][0]++;
     
     alloc_table[address].usecnt[alloc_table[address].cnt]   = usecnt;
     alloc_table[address].port[alloc_table[address].cnt]     = DUT.memory_management_unit.in_sel;
     alloc_table[address].cnt++;


     
	   end   


///////////////////////////////////////////////////////////////////////////////////////////////////////
///////// Monitoring deallocation of pages  /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
   	   
   always @(posedge clk) if(DUT.memory_management_unit.alloc_core.tmp_dbg_dealloc)
     begin
     int address;  
    
     address =  DUT.memory_management_unit.alloc_core.tmp_page;  

     pg_dealloc_cnt[address][0]++;
       
     dealloc_table[address].cnt++;  
       
     end 	   



///////////////////////////////////////////////////////////////////////////////////////////////////////
///////// Monitoring freeing of pages  /////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////// 
          
   always @(posedge clk) if(DUT.memory_management_unit.pg_free & DUT.memory_management_unit.pg_done)
     begin
     int address;  
     int port_mask;
     int port;
     
     port      = DUT.memory_management_unit.in_sel;    
     address   = DUT.memory_management_unit.pg_addr;  
     port_mask = dealloc_table[address].port[dealloc_table[address].cnt ] ;
     
     pg_dealloc_cnt[address][pg_dealloc_cnt[address][0] + 1]++;
     
     dealloc_table[address].port[dealloc_table[address].cnt ] = ((1 << port) | port_mask) & 'h7FF;     
     dealloc_table[address].usecnt[dealloc_table[address].cnt ]++;
     
     
       
     end 	      
 
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////// Monitoring setting of pages' usecnt /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////     
     
   always @(posedge clk) if(DUT.memory_management_unit.pg_set_usecnt & DUT.memory_management_unit.pg_done)
     begin
     int address;  

     address =  DUT.memory_management_unit.pg_addr;  
       
     pg_alloc_cnt[address][pg_alloc_cnt[address][0] + 1] =  DUT.memory_management_unit.pg_usecnt;
     
     alloc_table[address].usecnt[alloc_table[address].cnt - 1]   = DUT.memory_management_unit.pg_usecnt;;

       
     end 	      
        
endmodule // main