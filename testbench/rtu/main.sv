/*-------------------------------------------------------------------------------
-- Title      : RTU Testbench 
-- Project    : White Rabbit switch
-------------------------------------------------------------------------------
-- File       : main.sv
-- Author     : Maciej Lipinski
-- Company    : CERN BE-Co-HT
-- Created    : 2012-05-24
-- Last update: 2012-05-24
-- Platform   : SystemVerilog
-- Standard   : 
-------------------------------------------------------------------------------
-- Description: 
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
-- 
-------------------------------------------------------------------------------
--
-- Copyright (c) 2012 Maciej Lipinski / CERN
--
-- This source file is free software; you can redistribute it   
-- and/or modify it under the terms of the GNU Lesser General   
-- Public License as published by the Free Software Foundation; 
-- either version 2.1 of the License, or (at your option) any   
-- later version.                                               
--
-- This source is distributed in the hope that it will be       
-- useful, but WITHOUT ANY WARRANTY; without even the implied   
-- warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      
-- PURPOSE.  See the GNU Lesser General Public License for more 
-- details.                                                     
--
-- You should have received a copy of the GNU Lesser General    
-- Public License along with this source; if not, download it   
-- from http://www.gnu.org/licenses/lgpl-2.1.html
--
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author   Description
-- 2012-05-24  1.0      mlipinsk Created
------------------------------------------------------------------------------*/

// `include "xwrsw_rtu_wrapper.svh"
// `include "if_rtu_port.svh"
`include "xwrsw_rtu_wrapper.svh"
`include "if_rtu_port.svh"
`include "if_wb_master.svh"
`include "if_wb_slave.svh"
`include "simdrv_rtu.sv"

`define c_mac_addr_width  48
`define c_vid_num_width   12
`define c_prio_num_width  3
`define c_prio_num        8
`define c_num_ports       8 
`define c_port_mask_bits (`c_num_ports + 1)

module main;
  
/*  parameter g_mac_addr_width = 48;
  parameter g_vid_width      = 3;
  parameter g_prio_width     = 3;
  parameter g_port_mask_bits = 15;
  parameter g_prio_num       = 8;
  parameter g_num_ports      = 16; */ 
 
  reg clk_i=0;
  reg rst_n=0;
  
  always #8ns clk_i <= ~clk_i;
  CRTUSimDriver rtu;
  
  IRTU  #(`c_mac_addr_width,
          `c_vid_num_width,
          `c_prio_num_width, 
          `c_port_mask_bits) port[`c_num_ports] (clk_i,rst_n_i);  
  
  swc_core_wrapper_generic
    #(
      .g_prio_num                         (`c_prio_num),
      .g_prio_num_width                   (`c_prio_num_width),    
      .g_vid_num_width                    (`c_vid_num_width),
      .g_mac_addr_width                   (`c_mac_addr_width),
      .g_num_ports                        (`c_num_ports),
      .g_port_mask_bits                   (`c_port_mask_bits)
      ) 
  DUT (
      .clk_i(clk_i),
      .rst_n_i(rst_n)
/*      .port(port),
      .rtu_wb(rtu_wb)   */ 
      );
  
  initial begin
    repeat(3) @(posedge clk_i);
    rst_n <= 1;
  end  
  initial begin
    
    rtu_vlan_entry_t def_vlan; 
    CWishboneAccessor rtu_acc = DUT.rtu_wb.get_accessor();
    bit[7:0] dmac[];
    bit[7:0] smac[];
    
    repeat(200) @(posedge clk_i);
    
    rtu = new;
    rtu_acc.set_mode(PIPELINED);
    DUT.rtu_wb.settings.addr_gran = WORD;    
    rtu.set_bus(rtu_acc, 'h00000);
    
    for (int dd=0;dd<`c_num_ports;dd++)
      begin
        rtu.set_port_config(dd, 1, 1, 1);
//        DUT.port[dd].init_port();
      end

    def_vlan.port_mask      = 32'hffffffff;
    def_vlan.fid            = 0;
    def_vlan.drop           = 0;
    def_vlan.has_prio       = 0;
    def_vlan.prio_override  = 0;

    rtu.add_vlan_entry(0, def_vlan);
    rtu.add_static_rule('{5, 'h50, 'hca, 'hfe, 'hba, 'hbe}, (1<<2 ));
    rtu.add_static_rule('{6, 'h50, 'hca, 'hfe, 'hba, 'hbe}, (1<<1 ));    
    rtu.enable();

    DUT.port[0].init_port(0);
    DUT.port[1].init_port(1);
    DUT.port[2].init_port(2);
    DUT.port[3].init_port(3);
    DUT.port[4].init_port(4);
    DUT.port[5].init_port(5);
    DUT.port[6].init_port(6);
    DUT.port[7].init_port(7);

      
    repeat(200) @(posedge clk_i);
    smac = '{5, 'h50, 'hca, 'hfe, 'hba, 'hbe};
    dmac = '{6, 'h50, 'hca, 'hfe, 'hba, 'hbe};

/*
    DUT.port[0].send_req('{5, 'h50, 'hca, 'hfe, 'hba, 'hbe};, //dmac
                         '{6, 'h50, 'hca, 'hfe, 'hba, 'hbe}, //smac
                           0,                                //vid
                           0,                                //has_vid
                           0,                                //prio
                           0,                                //has_prio
                           0);                               //drop
*/
    DUT.port[0].send_req(dmac, smac, 0,0,0,0,0);

    dmac[0]=0;
    DUT.port[0].send_req(dmac, smac, 0,0,0,0,0);
    dmac[0]=1;
    DUT.port[1].send_req(dmac, smac, 0,0,0,0,0);
    dmac[0]=2;
    DUT.port[2].send_req(dmac, smac, 0,0,0,0,0);
    dmac[0]=3;
    DUT.port[3].send_req(dmac, smac, 0,0,0,0,0);

  
  end
  
endmodule //main