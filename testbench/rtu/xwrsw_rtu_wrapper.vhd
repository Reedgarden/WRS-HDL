-------------------------------------------------------------------------------
-- Title      : Testbench wrapper for extended (interface-wise) Routing Table Unit (RTU)
-- Project    : White Rabbit Switch
-------------------------------------------------------------------------------
-- File       : xwrsw_rtu_wrapper.vhd
-- Authors    : Maciej Lipinski
-- Company    : CERN BE-Co-HT
-- Created    : 2012-05-23
-- Last update: 2012-05-23
-- Platform   : FPGA-generic
-- Standard   : VHDL
-------------------------------------------------------------------------------
-- Description: wrapping xwrsw_rtu.vhd for testbench in systemVerilog
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
-- Date        Version  Author          Description
-- 2012-05-23  1.0      mlipinsk        Created
-------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.wishbone_pkg.all;
use work.wrsw_shared_types_pkg.all;
use work.wrsw_rtu_private_pkg.all;


entity xwrsw_rtu_wrapper is
  generic (
    g_interface_mode      : t_wishbone_interface_mode      := PIPELINED;
    g_address_granularity : t_wishbone_address_granularity := WORD;
    g_handle_only_single_req_per_port : boolean := TRUE;
    g_prio_num            : integer;
    g_prio_num_width      : integer;
    g_vid_num_width       : integer;
    g_mac_addr_width      : integer;
    g_num_ports           : integer;
    g_port_mask_bits      : integer);

  port(
    clk_i   : in std_logic;
    rst_n_i     : in std_logic;
-------------------------------------------------------------------------------
-- N-port RTU input interface (from the endpoint)
-------------------------------------------------------------------------------
    rtu_idle_o          : out std_logic_vector(g_num_ports-1 downto 0);
    rq_strobe_p_i       : in std_logic_vector(g_num_ports-1 downto 0);
    rq_smac_i           : in std_logic_vector(g_num_ports * g_mac_addr_width - 1 downto 0);
    rq_dmac_i           : in std_logic_vector(g_num_ports * g_mac_addr_width -1 downto 0);
    rq_vid_i            : in std_logic_vector(g_num_ports * g_vid_num_width - 1 downto 0);
    rq_has_vid_i        : in std_logic_vector(g_num_ports -1 downto 0);
    rq_prio_i           : in std_logic_vector(g_num_ports * g_prio_num_width -1 downto 0);
    rq_has_prio_i       : in std_logic_vector(g_num_ports -1 downto 0);
-------------------------------------------------------------------------------
-- N-port RTU output interface (to the packet buffer
-------------------------------------------------------------------------------
    rsp_valid_o         : out std_logic_vector(g_num_ports-1 downto 0);
    rsp_dst_port_mask_o : out std_logic_vector(g_num_ports * g_port_mask_bits - 1 downto 0);
    rsp_drop_o          : out std_logic_vector(g_num_ports -1 downto 0);
    rsp_prio_o          : out std_logic_vector(g_num_ports * g_prio_num_width - 1 downto 0);
    rsp_ack_i           : in  std_logic_vector(g_num_ports -1 downto 0);
    port_almost_full_o  : out std_logic_vector(g_num_ports -1 downto 0);
    port_full_o         : out std_logic_vector(g_num_ports -1 downto 0);
-------------------------------------------------------------------------------
-- Wishbone (synchronous to refclk2_i). See the wbgen2 file for register details
-------------------------------------------------------------------------------
    wb_addr_i : in  std_logic_vector(31 downto 0);
    wb_data_i : in  std_logic_vector(31 downto 0);
    wb_data_o : out std_logic_vector(31 downto 0);
    wb_sel_i  : in  std_logic_vector(3 downto 0);
    wb_cyc_i  : in  std_logic;
    wb_stb_i  : in  std_logic;
    wb_ack_o  : out std_logic;
    wb_irq_o  : out std_logic;
    wb_we_i   : in  std_logic

    );

end xwrsw_rtu_wrapper;

architecture behavioral of xwrsw_rtu_wrapper is

  component xwrsw_rtu 
    generic (
      g_interface_mode      : t_wishbone_interface_mode      := PIPELINED;
      g_address_granularity : t_wishbone_address_granularity := BYTE;
      g_handle_only_single_req_per_port : boolean := FALSE;
      g_prio_num            : integer;
      g_num_ports           : integer;
      g_port_mask_bits      : integer);
    port (
      clk_sys_i : in std_logic;
      rst_n_i   : in std_logic;
      req_i      : in  t_rtu_request_array(g_num_ports-1 downto 0);
      req_full_o : out std_logic_vector(g_num_ports-1 downto 0);
      rsp_o     : out t_rtu_response_array(g_num_ports-1 downto 0);
      rsp_ack_i : in  std_logic_vector(g_num_ports-1 downto 0);
      wb_i : in  t_wishbone_slave_in;
      wb_o : out t_wishbone_slave_out
      );
    end component;


  signal req      : t_rtu_request_array(g_num_ports-1 downto 0);
  signal req_full : std_logic_vector(g_num_ports-1 downto 0);

  signal rsp      : t_rtu_response_array(g_num_ports-1 downto 0);
  signal rsp_ack  : std_logic_vector(g_num_ports-1 downto 0);


  signal wb_in    : t_wishbone_slave_in;
  signal wb_out   : t_wishbone_slave_out;

begin 


  U_xwrsw_rtu: xwrsw_rtu
    generic map (
      g_interface_mode                  => g_interface_mode,
      g_address_granularity             => g_address_granularity,
      g_handle_only_single_req_per_port => g_handle_only_single_req_per_port,
      g_prio_num                        => g_prio_num,
      g_num_ports                       => g_num_ports,
      g_port_mask_bits                  => g_port_mask_bits )

    port map(
        clk_sys_i                       => clk_i,
        rst_n_i                         => rst_n_i,
        req_i                           => req,
        req_full_o                      => req_full,
        rsp_o                           => rsp,
        rsp_ack_i                       => rsp_ack,
        wb_i                            => wb_in,
        wb_o                            => wb_out
        );

  rtu_idle_o <=(others => '0');
  
  wb_in.adr(13 downto 0) <= wb_addr_i(13 downto 0);
  wb_in.dat              <= wb_data_i;
  wb_in.sel              <= wb_sel_i;
  wb_in.cyc              <= wb_cyc_i;
  wb_in.stb              <= wb_stb_i;
  wb_in.we               <= wb_we_i;

  wb_data_o              <= wb_out.dat;
  wb_ack_o               <= wb_out.ack;
  wb_irq_o               <= wb_out.int;

  gen_merge_signals : for i in 0 to g_num_ports-1 generate
    req(i).valid     <= rq_strobe_p_i(i);
    req(i).smac      <= rq_smac_i(g_mac_addr_width * (i+1) - 1 downto g_mac_addr_width * i);
    req(i).dmac      <= rq_dmac_i(g_mac_addr_width * (i+1) - 1 downto g_mac_addr_width * i);
    req(i).vid       <= rq_vid_i(g_vid_num_width   * (i+1) - 1 downto g_vid_num_width  * i);
    req(i).prio      <= rq_prio_i(g_prio_num_width * (i+1) - 1 downto g_prio_num_width * i);
    req(i).has_prio  <= rq_has_prio_i(i);
    req(i).has_vid   <= rq_has_vid_i(i);

    rsp_valid_o(i)                                                               <= rsp(i).valid;
    rsp_dst_port_mask_o(g_port_mask_bits  * (i+1) -1 downto g_port_mask_bits * i)<= rsp(i).port_mask(g_port_mask_bits-1 downto 0) ;
    rsp_drop_o(i)                                                                <= rsp(i).drop;
    rsp_ack(i)                                                                   <= rsp_ack_i(i);
    rsp_prio_o(         g_prio_num_width * (i+1) -1 downto g_prio_num_width*i)   <= rsp(i).prio;
    port_full_o(i)                                                               <= req_full(i);
  end generate gen_merge_signals;


end architecture;  -- end of wrsw_rtu