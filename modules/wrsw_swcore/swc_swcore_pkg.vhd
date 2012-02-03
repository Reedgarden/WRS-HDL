-------------------------------------------------------------------------------
-- Title      : Switching Core Package
-- Project    : WhiteRabbit switch
-------------------------------------------------------------------------------
-- File       : swc_swcore_pkg.vhd
-- Author     : Tomasz Wlostowski
-- Company    : CERN BE-Co-HT
-- Created    : 2010-04-08
-- Last update: 2012-01-24
-- Platform   : FPGA-generic
-- Standard   : VHDL'87
-------------------------------------------------------------------------------
-- Description: 
-------------------------------------------------------------------------------
--
-- Copyright (c) 2010 Tomasz Wlostowski, Maciej Lipinski / CERN
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
-- 2010-04-08  1.0      twlostow Created
-- 2010-11-22  2.0      mlipinsk added staff
-- 2012-02-02  3.0      mlipinsk generic-azed
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.math_real.CEIL;
use ieee.math_real.log2;

library work;
use work.wr_fabric_pkg.all;
use work.wrsw_shared_types_pkg.all;

package swc_swcore_pkg is

  type t_swcore_gen_parameters is record
    num_ports: integer;
    mem_pages: integer;
    page_size: integer;
  end record;

  type t_slv_array is array(integer range <>, integer range <>) of std_logic;

  component swc_prio_encoder
    generic (
      g_num_inputs  : integer range 2 to 64;
      g_output_bits : integer range 1 to 6);
    port (
      in_i     : in  std_logic_vector(g_num_inputs-1 downto 0);
      out_o    : out std_logic_vector(g_output_bits-1 downto 0);
      onehot_o : out std_logic_vector(g_num_inputs-1 downto 0);
      mask_o   : out std_logic_vector(g_num_inputs-1 downto 0);
      zero_o   : out std_logic);
  end component;

  component swc_page_allocator
    generic (
      g_num_pages      : integer;
      g_page_addr_width : integer;
      g_num_ports      : integer ;
      g_usecount_width : integer);
    port (
      clk_i          : in  std_logic;
      rst_n_i        : in  std_logic;
      alloc_i        : in  std_logic;
      free_i         : in  std_logic;
      force_free_i   : in std_logic;
      set_usecnt_i   : in std_logic;
      usecnt_i       : in  std_logic_vector(g_usecount_width-1 downto 0);
      pgaddr_i       : in  std_logic_vector(g_page_addr_width -1 downto 0);
      pgaddr_o       : out std_logic_vector(g_page_addr_width -1 downto 0);
      pgaddr_valid_o : out std_logic;
      idle_o         : out std_logic;
      done_o         : out std_logic;
      nomem_o        : out std_logic);
  end component;

  --component swc_page_allocator
  component swc_page_allocator_new
    generic (
      g_num_pages      : integer;
      g_page_addr_width : integer;
      g_usecount_width : integer);
    port (
      clk_i          : in  std_logic;
      rst_n_i        : in  std_logic;
      alloc_i        : in  std_logic;
      free_i         : in  std_logic;
      force_free_i   : in std_logic;
      set_usecnt_i   : in std_logic;
      usecnt_i       : in  std_logic_vector(g_usecount_width-1 downto 0);
      pgaddr_i       : in  std_logic_vector(g_page_addr_width -1 downto 0);
      pgaddr_o       : out std_logic_vector(g_page_addr_width -1 downto 0);
      pgaddr_valid_o : out std_logic;
      idle_o         : out std_logic;
      done_o         : out std_logic;
      nomem_o        : out std_logic);
  end component;

  component swc_rr_arbiter
    generic (
      g_num_ports      : natural;
      g_num_ports_log2 : natural);
    port (
      rst_n_i       : in  std_logic;
      clk_i         : in  std_logic;
      next_i        : in  std_logic;
      request_i     : in  std_logic_vector(g_num_ports -1 downto 0);
      grant_o       : out std_logic_vector(g_num_ports_log2 - 1 downto 0);
      grant_valid_o : out std_logic);
  end component;

  component swc_packet_mem_write_pump
    generic ( 
      g_page_addr_width                  : integer ;--:= c_swc_page_addr_width;
      g_pump_data_width                  : integer ;--:= c_swc_pump_width
      g_mem_addr_width                   : integer ;--:= c_swc_packet_mem_addr_width
      g_page_addr_offset_width           : integer ;--:= c_swc_page_offset_width
      -- probably useless with new memory
      g_packet_mem_multiply              : integer --:= c_swc_packet_mem_multiply
      );  
    port (
      clk_i               : in  std_logic;
      rst_n_i             : in  std_logic;
      pgaddr_i            : in  std_logic_vector(g_page_addr_width-1 downto 0);
      pgreq_i             : in  std_logic;
      pgend_o             : out std_logic;
      pckstart_i          : in std_logic;
      drdy_i              : in  std_logic;
      full_o              : out std_logic;
      flush_i             : in  std_logic;
      ll_addr_o           : out std_logic_vector(g_page_addr_width -1 downto 0);
      ll_data_o           : out std_logic_vector(g_page_addr_width - 1 downto 0);
      ll_wr_req_o         : out std_logic;
      ll_wr_done_i        : in std_logic;
      sync_i              : in  std_logic;
      addr_o              : out std_logic_vector(g_mem_addr_width -1 downto 0);
      d_i                 : in  std_logic_vector(g_pump_data_width -1 downto 0);
      q_o                 : out std_logic_vector(g_pump_data_width * g_packet_mem_multiply - 1 downto 0);
      we_o                : out std_logic);
  end component;

  component swc_packet_mem_read_pump
  generic ( 
      g_page_addr_width                  : integer ;--:= c_swc_page_addr_width;
      g_pump_data_width                  : integer ;--:= c_swc_pump_width
      g_mem_addr_width                   : integer ;--:= c_swc_packet_mem_addr_width
      g_page_addr_offset_width           : integer ;--:= c_swc_page_offset_width
      -- probably useless with new memory
      g_packet_mem_multiply              : integer --:= c_swc_packet_mem_multiply
    );
    port (
      clk_i               : in  std_logic;
      rst_n_i             : in  std_logic;
      pgreq_i             : in  std_logic;
      pgaddr_i            : in  std_logic_vector(g_page_addr_width - 1 downto 0);
      pgend_o             : out std_logic;
      pckend_o            : out std_logic;
      drdy_o              : out std_logic;
      dreq_i              : in  std_logic;
      sync_read_i         : in  std_logic;
      ll_read_addr_o      : out std_logic_vector(g_page_addr_width -1 downto 0);
      ll_read_data_i      : in  std_logic_vector(g_page_addr_width - 1 downto 0);
      ll_read_req_o       : out std_logic;
      ll_read_valid_data_i: in  std_logic;
      sync_i              : in  std_logic;
      d_o                 : out std_logic_vector(g_pump_data_width - 1 downto 0);
      addr_o              : out std_logic_vector(g_mem_addr_width - 1 downto 0);
      q_i                 : in  std_logic_vector(g_pump_data_width * g_packet_mem_multiply -1 downto 0));
  end component;
  
  component swc_multiport_linked_list is
  generic ( 
    g_num_ports                        : integer; --:= c_swc_num_ports
    g_page_addr_width                  : integer; --:= c_swc_page_addr_width;
    g_page_num                         : integer  --:= c_swc_packet_mem_num_pages
    );
    port (
      rst_n_i               : in  std_logic;
      clk_i                 : in  std_logic;

      write_i               : in  std_logic_vector(g_num_ports - 1 downto 0);
      write_done_o          : out std_logic_vector(g_num_ports - 1 downto 0);
      write_addr_i          : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      write_data_i          : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      
      free_i                : in  std_logic_vector(g_num_ports - 1 downto 0);
      free_done_o           : out std_logic_vector(g_num_ports - 1 downto 0);
      free_addr_i           : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      
      read_pump_read_i      : in  std_logic_vector(g_num_ports - 1 downto 0);
      read_pump_read_done_o : out std_logic_vector(g_num_ports - 1 downto 0);
      read_pump_addr_i      : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
            
      free_pck_read_i       : in  std_logic_vector(g_num_ports - 1 downto 0);
      free_pck_read_done_o  : out std_logic_vector(g_num_ports - 1 downto 0);
      free_pck_addr_i       : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      
      data_o                : out std_logic_vector(g_page_addr_width - 1 downto 0)
      );
  
  end component;
  
  component xswc_input_block is
    generic ( 
      g_page_addr_width                  : integer ;--:= c_swc_page_addr_width;
      g_num_ports                        : integer ;--:= c_swc_num_ports
      g_prio_width                       : integer ;--:= c_swc_prio_width;
      g_max_pck_size_width               : integer ;--:= c_swc_max_pck_size_width  
      g_usecount_width                   : integer ;--:= c_swc_usecount_width
      g_data_width                       : integer ;--:= c_swc_data_width
      g_ctrl_width                       : integer ;--:= c_swc_ctrl_width
      g_input_block_cannot_accept_data   : string := "drop_pck"; --"stall_o", "rty_o" -- Don't CHANGE !

      -- probably useless with new memory
      g_packet_mem_multiply              : integer ;--:= c_swc_packet_mem_multiply
      g_input_block_fifo_size            : integer ;--:= c_swc_input_fifo_size
      g_input_block_fifo_full_in_advance : integer  --:=c_swc_fifo_full_in_advance
    );
    port (
      clk_i   : in std_logic;
      rst_n_i : in std_logic;
      -------------------------------------------------------------------------------
      -- pWB  : input (comes from the Endpoint)
      -------------------------------------------------------------------------------
      snk_i : in  t_wrf_sink_in;
      snk_o : out t_wrf_sink_out;
      -------------------------------------------------------------------------------
      -- I/F with Page allocator (MMU)
      -------------------------------------------------------------------------------    
      mmu_page_alloc_req_o : out std_logic;
      mmu_page_alloc_done_i : in std_logic;
      mmu_pageaddr_i : in std_logic_vector(g_page_addr_width - 1 downto 0);
      mmu_pageaddr_o : out std_logic_vector(g_page_addr_width - 1 downto 0);
      mmu_force_free_o : out std_logic;
      mmu_force_free_done_i : in std_logic;
      mmu_force_free_addr_o : out std_logic_vector(g_page_addr_width - 1 downto 0);
      mmu_set_usecnt_o : out std_logic;
      mmu_set_usecnt_done_i : in std_logic;
      mmu_usecnt_o : out std_logic_vector(g_usecount_width - 1 downto 0);
      mmu_nomem_i : in std_logic;
  -------------------------------------------------------------------------------
  -- I/F with Routing Table Unit (RTU)
  -------------------------------------------------------------------------------      
      rtu_rsp_valid_i     : in  std_logic;
      rtu_rsp_ack_o       : out std_logic;
      rtu_dst_port_mask_i : in  std_logic_vector(g_num_ports - 1 downto 0);
      rtu_drop_i          : in  std_logic;
      rtu_prio_i          : in  std_logic_vector(g_prio_width - 1 downto 0);
  -------------------------------------------------------------------------------
  -- I/F with Multiport Memory (MPU)
  -------------------------------------------------------------------------------    
      mpm_pckstart_o : out std_logic;
      mpm_pageaddr_o : out std_logic_vector(g_page_addr_width - 1 downto 0);
      mpm_pagereq_o : out std_logic;
      mpm_pageend_i : in  std_logic;
      mpm_data_o : out std_logic_vector(g_data_width - 1 downto 0);
      mpm_ctrl_o : out std_logic_vector(g_ctrl_width - 1 downto 0);
      mpm_drdy_o : out std_logic;
      mpm_full_i : in std_logic;
      mpm_flush_o : out std_logic;
      mpm_wr_sync_i : in std_logic;
  -------------------------------------------------------------------------------
  -- I/F with Page Transfer Arbiter (PTA)
  -------------------------------------------------------------------------------     
      pta_transfer_pck_o : out std_logic;
      pta_transfer_ack_i : in std_logic;
      pta_pageaddr_o : out std_logic_vector(g_page_addr_width - 1 downto 0);
      pta_mask_o : out std_logic_vector(g_num_ports - 1 downto 0);
      pta_pck_size_o : out std_logic_vector(g_max_pck_size_width - 1 downto 0);
      pta_prio_o : out std_logic_vector(g_prio_width - 1 downto 0)
      );
  end component;


  component swc_multiport_page_allocator is
    generic ( 
      g_page_addr_width                  : integer ;--:= c_swc_page_addr_width;
      g_num_ports                        : integer ;--:= c_swc_num_ports
      g_page_num                         : integer ;--:= c_swc_packet_mem_num_pages
      g_usecount_width                   : integer --:= c_swc_usecount_width
    );  
    port (
      rst_n_i             : in std_logic;
      clk_i               : in std_logic;
      alloc_i             : in  std_logic_vector(g_num_ports - 1 downto 0);
      free_i              : in  std_logic_vector(g_num_ports - 1 downto 0);
      force_free_i        : in  std_logic_vector(g_num_ports - 1 downto 0);
      set_usecnt_i        : in  std_logic_vector(g_num_ports - 1 downto 0);
      alloc_done_o        : out std_logic_vector(g_num_ports - 1 downto 0);
      free_done_o         : out std_logic_vector(g_num_ports - 1 downto 0);
      force_free_done_o   : out std_logic_vector(g_num_ports - 1 downto 0);
      set_usecnt_done_o   : out std_logic_vector(g_num_ports - 1 downto 0);
      pgaddr_free_i       : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      pgaddr_force_free_i : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      pgaddr_usecnt_i     : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      usecnt_i            : in  std_logic_vector(g_num_ports * g_usecount_width - 1 downto 0);
      pgaddr_alloc_o      : out std_logic_vector(g_page_addr_width-1 downto 0);
      nomem_o             : out std_logic
      );
  
  end component;
  
  component swc_packet_mem is
    generic ( 
      g_mem_size                         : integer ;--:= c_swc_packet_mem_size (in input words: 16 bits)
      g_num_ports                        : integer ;--:= c_swc_num_ports
      g_page_num                         : integer ;--:= c_swc_packet_mem_num_pages
      g_page_addr_width                  : integer ;--:= c_swc_page_addr_width;
      g_data_width                       : integer ;--:= c_swc_data_width
      g_ctrl_width                       : integer ;--:= c_swc_ctrl_width
      g_page_size                        : integer ;--:= c_swc_page_size

      -- probably useless with new memory
      g_packet_mem_multiply              : integer --:= c_swc_packet_mem_multiply
      );
    port (
      clk_i   : in std_logic;
      rst_n_i : in std_logic;
      ------------------- writing to the shared memory --------------------------
      wr_pagereq_i  : in  std_logic_vector(g_num_ports-1 downto 0);
      wr_pckstart_i : in  std_logic_vector(g_num_ports-1 downto 0);
      wr_pageaddr_i : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      wr_pageend_o  : out std_logic_vector(g_num_ports -1 downto 0);
      wr_ctrl_i  : in  std_logic_vector(g_num_ports * g_ctrl_width - 1 downto 0);
      wr_data_i  : in  std_logic_vector(g_num_ports * g_data_width - 1 downto 0);
      wr_drdy_i  : in  std_logic_vector(g_num_ports-1 downto 0);
      wr_full_o  : out std_logic_vector(g_num_ports-1 downto 0);
      wr_flush_i : in  std_logic_vector(g_num_ports-1 downto 0);
      wr_sync_o : out std_logic_vector(g_num_ports -1 downto 0);
      ------------------- reading from the shared memory --------------------------
      rd_pagereq_i   : in  std_logic_vector(g_num_ports-1 downto 0);
      rd_pageaddr_i  : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      rd_pageend_o   : out std_logic_vector(g_num_ports -1 downto 0);
      rd_pckend_o    : out  std_logic_vector(g_num_ports-1 downto 0);
      rd_drdy_o      : out std_logic_vector(g_num_ports -1 downto 0);
      rd_dreq_i      : in  std_logic_vector(g_num_ports -1 downto 0);
      rd_sync_read_i : in std_logic_vector(g_num_ports -1 downto 0);
      rd_data_o      : out std_logic_vector(g_num_ports * g_data_width - 1 downto 0);
      rd_ctrl_o      : out std_logic_vector(g_num_ports * g_ctrl_width - 1 downto 0);
      rd_sync_o      : out std_logic_vector(g_num_ports -1 downto 0);
      
      write_o               : out  std_logic_vector(g_num_ports - 1 downto 0);
      write_done_i          : in   std_logic_vector(g_num_ports - 1 downto 0);
      write_addr_o          : out  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      write_data_o          : out  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      read_pump_read_o      : out  std_logic_vector(g_num_ports - 1 downto 0);
      read_pump_read_done_i : in  std_logic_vector(g_num_ports - 1 downto 0);
      read_pump_addr_o      : out  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
      data_i                : in   std_logic_vector(g_page_addr_width - 1 downto 0)
      );
      
    end component;
  
  component swc_pck_transfer_input is
    generic(
      g_page_addr_width    : integer ;--:= c_swc_page_addr_width;
      g_prio_width         : integer ;--:= c_swc_prio_width;
      g_max_pck_size_width : integer ;--:= c_swc_max_pck_size_width    
      g_num_ports          : integer  --:= c_swc_num_ports
    );
    port (
      clk_i              : in std_logic;
      rst_n_i            : in std_logic;
      
      pto_transfer_pck_o : out  std_logic;
      pto_pageaddr_o     : out  std_logic_vector(g_page_addr_width - 1 downto 0);
      pto_output_mask_o  : out  std_logic_vector(g_num_ports - 1 downto 0);
      pto_read_mask_i    : in  std_logic_vector(g_num_ports - 1 downto 0);
      pto_prio_o         : out  std_logic_vector(g_prio_width - 1 downto 0);
      pto_pck_size_o     : out  std_logic_vector(g_max_pck_size_width - 1 downto 0);
      
      ib_transfer_pck_i  : in  std_logic;
      ib_pageaddr_i      : in  std_logic_vector(g_page_addr_width - 1 downto 0);
      ib_mask_i          : in  std_logic_vector(g_num_ports - 1 downto 0);
      ib_prio_i          : in  std_logic_vector(g_prio_width - 1 downto 0);
      ib_pck_size_i      : in  std_logic_vector(g_max_pck_size_width - 1 downto 0);
      ib_transfer_ack_o  : out std_logic;
      ib_busy_o          : out std_logic
      
      );
  end component;  
  
  component swc_pck_transfer_output is
    generic(
      g_page_addr_width    : integer ;--:= g_page_addr_width;
      g_prio_width         : integer ;--:= c_swc_prio_width;
      g_max_pck_size_width : integer --:= c_swc_max_pck_size_width
      );
    port (
      clk_i                    : in  std_logic;
      rst_n_i                  : in  std_logic;
      
      ob_transfer_data_valid_o : out std_logic;
      ob_pageaddr_o            : out std_logic_vector(g_page_addr_width - 1 downto 0);
      ob_prio_o                : out std_logic_vector(g_prio_width - 1 downto 0);
      ob_pck_size_o            : out std_logic_vector(g_max_pck_size_width - 1 downto 0);
      ob_transfer_data_ack_i   : in  std_logic;
      
      pti_transfer_data_valid_i: in  std_logic;
      pti_transfer_data_ack_o  : out std_logic;
      pti_pageaddr_i           : in  std_logic_vector(g_page_addr_width - 1 downto 0);
      pti_prio_i               : in  std_logic_vector(g_prio_width - 1 downto 0);
      pti_pck_size_i           : in  std_logic_vector(g_max_pck_size_width - 1 downto 0)
      
      );
  end component;
  
  component swc_pck_transfer_arbiter is
    generic(
      g_page_addr_width    : integer ;--:= c_swc_page_addr_width;
      g_prio_width         : integer ;--:= c_swc_prio_width;
      g_max_pck_size_width : integer ;--:= c_swc_max_pck_size_width    
      g_num_ports          : integer  --:= c_swc_num_ports
      );
    port (
      clk_i              : in  std_logic;
      rst_n_i            : in  std_logic;
      
      ob_data_valid_o    : out std_logic_vector(g_num_ports - 1 downto 0);
      ob_ack_i           : in  std_logic_vector(g_num_ports - 1 downto 0);
      ob_pageaddr_o      : out std_logic_vector(g_num_ports * g_page_addr_width    - 1 downto 0);
      ob_prio_o          : out std_logic_vector(g_num_ports * g_prio_width         - 1 downto 0);
      ob_pck_size_o      : out std_logic_vector(g_num_ports * g_max_pck_size_width - 1 downto 0);
      
      ib_transfer_pck_i  : in  std_logic_vector(g_num_ports - 1 downto 0);
      ib_transfer_ack_o  : out std_logic_vector(g_num_ports - 1 downto 0);
      ib_busy_o          : out std_logic_vector(g_num_ports - 1 downto 0);  
      ib_pageaddr_i      : in  std_logic_vector(g_num_ports * g_page_addr_width    - 1 downto 0);
      ib_mask_i          : in  std_logic_vector(g_num_ports * g_num_ports          - 1 downto 0);
      ib_prio_i          : in  std_logic_vector(g_num_ports * g_prio_width         - 1 downto 0);
      ib_pck_size_i      : in  std_logic_vector(g_num_ports * g_max_pck_size_width - 1 downto 0)
      );  
  end component;
  
  component swc_ob_prio_queue is
    generic(
      g_per_prio_fifo_size_width : integer --:= c_swc_output_fifo_addr_width
      );
    port (
      clk_i             : in   std_logic;
      rst_n_i           : in   std_logic;
      write_i           : in   std_logic;
      read_i            : in   std_logic;
      not_full_o        : out  std_logic;
      not_empty_o       : out  std_logic;
      wr_en_o           : out  std_logic;
      wr_addr_o         : out  std_logic_vector(g_per_prio_fifo_size_width - 1 downto 0);
      rd_addr_o         : out  std_logic_vector(g_per_prio_fifo_size_width - 1 downto 0)
      );
  end component;
  
  component xswc_output_block is
    generic ( 
      g_page_addr_width                  : integer ;--:= c_swc_page_addr_width;
      g_max_pck_size_width               : integer ;--:= c_swc_max_pck_size_width  
      g_data_width                       : integer ;--:= c_swc_data_width
      g_ctrl_width                       : integer ;--:= c_swc_ctrl_width
      g_output_block_per_prio_fifo_size  : integer ;--:= c_swc_output_fifo_size
      g_prio_width                       : integer ;--:= c_swc_prio_width;, c_swc_output_prio_num_width
      g_prio_num                         : integer  --:= c_swc_output_prio_num
    );
    port (
      clk_i   : in std_logic;
      rst_n_i : in std_logic;
      pta_transfer_data_valid_i : in   std_logic;
      pta_pageaddr_i            : in   std_logic_vector(g_page_addr_width - 1 downto 0);
      pta_prio_i                : in   std_logic_vector(g_prio_width - 1 downto 0);
      pta_pck_size_i            : in   std_logic_vector(g_max_pck_size_width - 1 downto 0);
      pta_transfer_data_ack_o   : out  std_logic;
      mpm_pgreq_o  : out std_logic;
      mpm_pgaddr_o : out std_logic_vector(g_page_addr_width - 1 downto 0);
      mpm_pckend_i : in  std_logic;
      mpm_pgend_i  : in  std_logic;
      mpm_drdy_i   : in  std_logic;
      mpm_dreq_o   : out std_logic;
      mpm_data_i   : in  std_logic_vector(g_data_width - 1 downto 0);
      mpm_ctrl_i   : in  std_logic_vector(g_ctrl_width - 1 downto 0);
      mpm_sync_i   : in  std_logic; 
      ppfm_free_o            : out  std_logic;
      ppfm_free_done_i       : in   std_logic;
      ppfm_free_pgaddr_o     : out  std_logic_vector(g_page_addr_width - 1 downto 0);
      src_i : in  t_wrf_source_in;
      src_o : out t_wrf_source_out
      );
  end component;

component  swc_multiport_pck_pg_free_module is
  generic( 
    g_num_ports             : integer ; --:= c_swc_num_ports
    g_page_addr_width       : integer ;--:= c_swc_page_addr_width;
    g_pck_pg_free_fifo_size : integer  --:= c_swc_freeing_fifo_size
      ); 
  port (
    clk_i   : in std_logic;
    rst_n_i : in std_logic;

    ib_force_free_i         : in  std_logic_vector(g_num_ports-1 downto 0);
    ib_force_free_done_o    : out std_logic_vector(g_num_ports-1 downto 0);
    ib_force_free_pgaddr_i  : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);

    ob_free_i               : in  std_logic_vector(g_num_ports-1 downto 0);
    ob_free_done_o          : out std_logic_vector(g_num_ports-1 downto 0);
    ob_free_pgaddr_i        : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
    
    ll_read_addr_o          : out std_logic_vector(g_num_ports * g_page_addr_width -1 downto 0);
    --ll_read_data_i          : in  std_logic_vector(g_num_ports * g_page_addr_width - 1 downto 0);
    ll_read_data_i          : in  std_logic_vector(g_page_addr_width - 1 downto 0);
    ll_read_req_o           : out std_logic_vector(g_num_ports-1 downto 0);
    ll_read_valid_data_i    : in  std_logic_vector(g_num_ports-1 downto 0);

    mmu_free_o              : out std_logic_vector(g_num_ports-1 downto 0);
    mmu_free_done_i         : in  std_logic_vector(g_num_ports-1 downto 0);
    mmu_free_pgaddr_o       : out std_logic_vector(g_num_ports * g_page_addr_width -1 downto 0);
    
    mmu_force_free_o        : out std_logic_vector(g_num_ports-1 downto 0);
    mmu_force_free_done_i   : in  std_logic_vector(g_num_ports-1 downto 0);
    mmu_force_free_pgaddr_o : out std_logic_vector(g_num_ports * g_page_addr_width -1 downto 0)
    );
  end component;

  component swc_pck_pg_free_module is
    generic( 
      g_page_addr_width       : integer ;--:= c_swc_page_addr_width;
      g_pck_pg_free_fifo_size : integer  --:= c_swc_freeing_fifo_size
      );  
    port (
      clk_i   : in std_logic;
      rst_n_i : in std_logic;
  
      ib_force_free_i         : in  std_logic;
      ib_force_free_done_o    : out std_logic;
      ib_force_free_pgaddr_i  : in  std_logic_vector(g_page_addr_width - 1 downto 0);
  
      ob_free_i               : in  std_logic;
      ob_free_done_o          : out std_logic;
      ob_free_pgaddr_i        : in  std_logic_vector(g_page_addr_width - 1 downto 0);
      
      ll_read_addr_o          : out std_logic_vector(g_page_addr_width -1 downto 0);
      ll_read_data_i          : in  std_logic_vector(g_page_addr_width - 1 downto 0);
      ll_read_req_o           : out std_logic;
      ll_read_valid_data_i    : in  std_logic;
  
      mmu_free_o              : out std_logic;
      mmu_free_done_i         : in  std_logic;
      mmu_free_pgaddr_o       : out std_logic_vector(g_page_addr_width -1 downto 0);
          
      mmu_force_free_o        : out std_logic;
      mmu_force_free_done_i   : in  std_logic;
      mmu_force_free_pgaddr_o : out std_logic_vector(g_page_addr_width -1 downto 0)
      );
  end component;
  
  component xswc_core is
    generic( 
      g_mem_size                         : integer ;--:= c_swc_packet_mem_size
      g_page_size                        : integer ;--:= c_swc_page_size
      g_prio_num                         : integer ;--:= c_swc_output_prio_num;
      g_max_pck_size                     : integer ;--:= c_swc_max_pck_size
      g_num_ports                        : integer ;--:= c_swc_num_ports
      g_data_width                       : integer ;--:= c_swc_data_width
      g_ctrl_width                       : integer ; --:= c_swc_ctrl_width
      g_pck_pg_free_fifo_size            : integer ; --:= c_swc_freeing_fifo_size (in pck_pg_free_module.vhd)
      g_input_block_cannot_accept_data   : string  ;--:= "drop_pck"; --"stall_o", "rty_o" -- (xswc_input_block) Don't CHANGE !
      g_output_block_per_prio_fifo_size  : integer ; --:= c_swc_output_fifo_size    (xswc_output_block)
      -- probably useless with new memory
      g_packet_mem_multiply              : integer ;--:= c_swc_packet_mem_multiply (xswc_input_block, )
      g_input_block_fifo_size            : integer ;--:= c_swc_input_fifo_size     (xswc_input_block)
      g_input_block_fifo_full_in_advance : integer --:=c_swc_fifo_full_in_advance (xswc_input_block)
      );
      port (
      clk_i   : in std_logic;
      rst_n_i : in std_logic;
  
      snk_i : in  t_wrf_sink_in_array(g_num_ports-1 downto 0);
      snk_o : out t_wrf_sink_out_array(g_num_ports-1 downto 0);
  
      src_i : in  t_wrf_source_in_array(g_num_ports-1 downto 0);
      src_o : out t_wrf_source_out_array(g_num_ports-1 downto 0);
      
      rtu_rsp_i           : in t_rtu_response_array(g_num_ports  - 1 downto 0);
      rtu_ack_o          : out std_logic_vector(g_num_ports  - 1 downto 0)
      );
  end component;

end swc_swcore_pkg;

package body swc_swcore_pkg is

end swc_swcore_pkg;