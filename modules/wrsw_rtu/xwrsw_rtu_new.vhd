-------------------------------------------------------------------------------
-- Title      : eXtended Routing Table Unit (RTU)
-- Project    : White Rabbit Switch
-------------------------------------------------------------------------------
-- File       : xwrsw_rtu_new.vhd
-- Authors    : Tomasz Wlostowski, Maciej Lipinski
-- Company    : CERN BE-Co-HT
-- Created    : 2012-01-10
-- Last update: 2012-12-03
-- Platform   : FPGA-generic
-- Standard   : VHDL
-------------------------------------------------------------------------------
-- Description: Module takes packet source & destination MAC addresses, VLAN ID
-- and priority priority and decides where and with what final priority (after
-- evalating the per MAC-assigned priorities, per-VLAN priorities, per-port and
-- per-packet), the packet shall be routed. The exact lookup algorithm is described in
-- rtu_sim.c file.
--
-- RTU has c_rtu_num_ports independent request ports which take RTU requests
-- from the endpoints, and c_rtu_num_ports response ports which deliver the routing
-- decisions for requests coming to associated input ports.
--
-- You can assume that RTU requests won't come more often than every 40 refclk2
-- cycles for each port. 
--
-- Since the RTU engine is shared by all ports, the requests are:
-- - scheduled by a round-robin arbiter, so each port gets the same priority
-- - inserted into a common request FIFO
-- - processed by the lookup engine
-- - responses are outputted to response FIFO and delivered into appropriate
--   destination ports.
--
-- RTU has 2 memory blocks:
-- external ZBT memory to store the main MAC table
-- small BRAM block (HCAM) for storage of entries which cause hash collisions in main
--
-- The main MAC table is organized as a
-- bucketed hashtable (see rtu_sim.c for details). Each bucket contains 4 entries:
--
-- addr 0: bucket 0 [entry 1] [entry 2] [entry 3] [entry 4]
-- addr 1: bucket 1 [entry 1] [entry 2] [entry 3] [entry 4]
--
-- If there are more than 4 MAC addresses with the same hash, the last entry in
-- the bucket contains a pointer to CAM memory which stores the remaining MAC
-- entries.
--
-- Both memories (ZBT and HCAM) are split into 2 banks. While one bank is
-- being used by the lookup engine, the other can be accessed from the Wishbone.
-- Bank switching is done by setting appropriate bit in WB control register.
-- RTU has a separate FIFO for writing the memories by the CPU (MFIFO). Each MFIFO
-- entry has 2 fields:
-- - address/data field select bit (determines if A/D field is a new address or
--   data value)
-- - address/data value
-- MFIFO has a separate timeslot for accessing the memory, which is scheduled
-- in the same manner as the input ports.
--
-- For all unrecognized requests RTU should (depending on configuration bit,
-- independently for each port) either drop or broadcast the packet. The
-- request itself is put into a separate FIFO (along with requesting port
-- number) and an interrupt is triggered. CPU parses the request using more sophisticated
-- algorithm and eventually updates the MAC table.
--
-- Aging: There is a separate RAM block ARAM (8192 + some bits for CAM entries), accessible both
-- from the CPU and the Wishbone. Every time matching entry is found, it's
-- corresponding bit is set to 1. CPU reads this table every few seconds and
-- updates the aging counters (aging is not implemented in hardware to make it
-- simpler)
--
-- Additional port configuration bits (needed for RSTP/STP implementation)
-- - LEARN_EN: enable learning on certain port (unrecognized requests go to
--   FIFO) (port is in ENABLED or LEARNING mode)
-- - DROP: drop all the packets regardless of the RTU decision (port is BLOCKING)
-- - PASS_BPDU: enable passing of BPDU packets (port is BLOCKING). BPDUs go to
--   the designated NIC port (ID/mask set in separate register)
--
-- Maciek: if you decide to use CRC-based hash, make the initial hash value & polynomial
-- programmable from Wishbone.
-- 
-- RTUeX:
-- - debugged new feature (Simulation and H/W on the switch):
--   * singe MAC FastForward
--   * range MAC FastForward
--   * Broadcast FastForward
--   * PTP FastForward
--   * LinkLimited FastForward
--   * Mirroring
-- - not debbuged: HP packet recognision - I need VLANs
-- 
-- 
-------------------------------------------------------------------------------
--
-- Copyright (c) 2012 CERN / BE-CO-HT
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
-- 2012-01-10  1.0      twlostow created
-- 2010-11-29  1.1      mlipinsk connected prio, added temp_hack
-- 2012-11-06  1.2      mlipinsk RTUeX - added fast match and config, integrated with RTU,
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.math_real.CEIL;
use ieee.math_real.log2;
use ieee.numeric_std.all;

use work.wishbone_pkg.all;
use work.wrsw_shared_types_pkg.all;
use work.rtu_private_pkg.all;
use work.pack_unpack_pkg.all;
use work.genram_pkg.all;
use work.rtu_wbgen2_pkg.all;
use work.gencores_pkg.all;

entity xwrsw_rtu_new is
  
  generic (
    g_interface_mode                  : t_wishbone_interface_mode      := PIPELINED;
    g_address_granularity             : t_wishbone_address_granularity := BYTE;
    g_handle_only_single_req_per_port : boolean                        := FALSE;
    g_prio_num                        : integer;
    g_num_ports                       : integer;
    g_cpu_port_num                    : integer := -1; --TODO: get rid of this
    g_match_req_fifo_size             : integer                        := 32;    
    g_port_mask_bits                  : integer;
    g_rmon_events_bits_pp             : integer := 9); -- rmon events num ber per port

  port (
    clk_sys_i   : in std_logic;
    rst_n_i     : in std_logic;

    req_i       : in  t_rtu_request_array(g_num_ports-1 downto 0);
    req_full_o  : out std_logic_vector(g_num_ports-1 downto 0);

    rsp_o       : out t_rtu_response_array(g_num_ports-1 downto 0);
    rsp_ack_i   : in  std_logic_vector(g_num_ports-1 downto 0);
    rq_abort_i  : in  std_logic_vector(g_num_ports-1 downto 0);
    rsp_abort_i  : in  std_logic_vector(g_num_ports-1 downto 0);
    tru_req_o   : out  t_tru_request;
    tru_resp_i  : in   t_tru_response;  
    rtu2tru_o   : out  t_rtu2tru;
    tru_enabled_i: in std_logic;
    rmon_events_o : out std_logic_vector(g_num_ports*g_rmon_events_bits_pp-1 downto 0);
    wb_i        : in  t_wishbone_slave_in;
    wb_o        : out t_wishbone_slave_out
    );

end xwrsw_rtu_new;
architecture behavioral of xwrsw_rtu_new is

  constant c_prio_num_width       : integer := integer(CEIL(LOG2(real(g_prio_num ))));
  constant c_g_num_ports_width    : integer := integer(CEIL(LOG2(real(g_num_ports ))));
  constant c_VLAN_TAB_ENTRY_WIDTH : integer := 46;
  constant c_match_req_fifo_size  : integer := g_match_req_fifo_size + g_num_ports;
  constant c_match_req_fifo_size_width : integer := integer(CEIL(LOG2(real(c_match_req_fifo_size ))));
  
  -- PORT_N -> MATCH_FIFO_ACCESS (round robin access to FIFO)
  signal rq_fifo_wr_access                  : std_logic_vector(g_num_ports-1 downto 0);
  signal rq_fifo_write_sng                  : std_logic;
  signal gnt_fifo_access                    : std_logic_vector(g_num_ports-1 downto 0);
  type t_rq_fifo_request_array is array (integer range <>) of std_logic_vector(c_PACKED_REQUEST_WIDTH-1 downto 0);
  signal rq_fifo_d_requests : t_rq_fifo_request_array(0 to g_num_ports-1);
  signal rq_fifo_d_muxed    : std_logic_vector(g_num_ports+ c_PACKED_REQUEST_WIDTH -1 downto 0);

  -- PORT_N -> Fast_Match
  signal fast_match_req                     : std_logic_vector(g_num_ports-1 downto 0);
  signal fast_match_req_data                : t_rtu_request_array(g_num_ports-1 downto 0);
  signal fast_match_rsp_data                : t_match_response;
  signal fast_match_rsp_valid               : std_logic_vector(g_num_ports-1 downto 0);

  -- MATCH_FIFO_ACCESS -> rtu_match
  signal rq_fifo_read                       : std_logic;
  signal rq_fifo_qvalid                     : std_logic;
  signal rq_fifo_data                       : std_logic_vector(g_num_ports + c_PACKED_REQUEST_WIDTH - 1 downto 0);
  signal rq_fifo_full                       : std_logic;
  signal rq_fifo_almost_full                : std_logic;
  signal rq_fifo_full_for_ports             : std_logic;
  signal rq_fifo_empty                      : std_logic;
  
  -- rtu_match -> PORTs
  signal rsp_valid                          : std_logic;
  signal rsp_data                           : std_logic_vector(g_num_ports + c_PACKED_RESPONSE_WIDTH - 1 downto 0);
  
  -- rtu_match -> rtu_lookup_engine (HTAB interface)
  signal htab_start                         : std_logic;
  signal htab_ack                           : std_logic;
  signal htab_hash                          : std_logic_vector(c_wrsw_hash_width-1 downto 0);
  signal htab_mac                           : std_logic_vector(47 downto 0);
  signal htab_fid                           : std_logic_vector(7 downto 0);
  signal htab_found                         : std_logic;
  signal htab_drdy                          : std_logic;
  signal htab_valid                         : std_logic;
  signal htab_entry                         : t_rtu_htab_entry;  

  -- U_WB_Slave <-> others
  type   t_pcr_prio_val_array is array(integer range <>) of std_logic_vector(c_wrsw_prio_width-1 downto 0);
  signal pcr_learn_en                       : std_logic_vector(c_rtu_max_ports - 1 downto 0);
  signal pcr_pass_all                       : std_logic_vector(c_rtu_max_ports - 1 downto 0);
  signal pcr_pass_bpdu                      : std_logic_vector(c_rtu_max_ports - 1 downto 0);
  signal pcr_fix_prio                       : std_logic_vector(c_rtu_max_ports - 1 downto 0);
  signal pcr_prio_val                       : t_pcr_prio_val_array(c_rtu_max_ports-1 downto 0);
  signal pcr_b_unrec                        : std_logic_vector(c_rtu_max_ports - 1 downto 0);
  signal regs_towb                          : t_rtu_in_registers;
  signal regs_fromwb                        : t_rtu_out_registers;
  signal current_pcr                        : integer;
  signal current_mac_ID                     : integer;
  signal rtu_gcr_poly_used                  : std_logic_vector(15 downto 0);
  signal mfifo_trigger                      : std_logic;
  signal current_MAC_entry                  : std_logic_vector(47 downto 0);

  --|HCAM - Hash collision memory
  signal aram_main_addr                     : std_logic_vector(7 downto 0);
  signal aram_main_data_i                   : std_logic_vector(31 downto 0);
  signal aram_main_data_o                   : std_logic_vector(31 downto 0);
  signal aram_main_rd                       : std_logic;
  signal aram_main_wr                       : std_logic;
  
  signal irq_nempty                         : std_logic;

  -- U_Adapter <-> U_WB_Slave : wishbone adapter (pipeline2classic) to WBgen-erated wisbhone slave
  signal wb_in                              : t_wishbone_slave_in;
  signal wb_out                             : t_wishbone_slave_out;

  -- FULL_Match to VLAN Tab memory
  signal vlan_tab_rd_vid                    : std_logic_vector(c_wrsw_vid_width-1 downto 0);
  signal vlan_tab_rd_data4match             : std_logic_vector(c_VLAN_TAB_ENTRY_WIDTH-1 downto 0); --packed
  signal vlan_tab_rd_entry4match            : t_rtu_vlan_tab_entry; -- unpacked
  
  -- Fast_Match to VLAN Tab memroy
  signal vlan_tab_rd_entry4fast_match       : t_rtu_vlan_tab_entry;
  signal fast_match_vtab_addr               : std_logic_vector(c_wrsw_vid_width-1 downto 0);
  signal fast_match_vtab_data               : std_logic_vector(c_VLAN_TAB_ENTRY_WIDTH-1 downto 0);

  -- WB slave to VLAN tab
  signal vlan_tab_wr_data                   : std_logic_vector(c_VLAN_TAB_ENTRY_WIDTH-1 downto 0);

  signal port_idle                          : std_logic_vector(g_num_ports-1 downto 0);
  signal rtu_special_traffic_config         : t_rtu_special_traffic_config;
  signal zeros                              : std_logic_vector(c_rtu_max_ports-1 downto 0);
  
  -- coutning fifo occupanccy (no usecnt provided) to indicate full to port when there
  -- is still N (port number) free places in FIFO (this is in case all the ports make
  -- request at the same time)
  signal match_req_fifo_cnt                 : unsigned(c_match_req_fifo_size_width-1 downto 0);

  -- aux    
  signal rsp_fifo_read_all_zeros            : std_logic_vector(g_num_ports - 1 downto 0);
  signal cpu_port_mask                      : std_logic_vector(c_rtu_max_ports - 1 downto 0);
  
  signal rsp                                : t_rtu_response_array(g_num_ports-1 downto 0);
  signal htab_port : std_logic_vector(g_num_ports - 1 downto 0);
  signal htab_src_dst : std_logic;
  
  signal dbg_forwarded_to_port              : std_logic_vector(g_num_ports - 1 downto 0);

  signal CONTROL0                   : std_logic_vector(35 downto 0);
  signal TRIG0, TRIG1, TRIG2, TRIG3 : std_logic_vector(31 downto 0);

  component chipscope_icon
    port (
      CONTROL0 : inout std_logic_vector(35 downto 0));
  end component;

  component chipscope_ila
    port (
      CONTROL : inout std_logic_vector(35 downto 0);
      CLK     : in    std_logic;
      TRIG0   : in    std_logic_vector(31 downto 0);
      TRIG1   : in    std_logic_vector(31 downto 0);
      TRIG2   : in    std_logic_vector(31 downto 0);
      TRIG3   : in    std_logic_vector(31 downto 0));
  end component;


begin 

  zeros                    <= (others => '0');
  rsp_fifo_read_all_zeros  <= (others => '0');
  irq_nempty               <= regs_fromwb.ufifo_wr_empty_o;
  req_full_o               <= not port_idle;
  
  GEN_NO_CPU_MASK: if(g_cpu_port_num < 0) generate
    cpu_port_mask            <= zeros;
  end generate GEN_NO_CPU_MASK;
  GEN_CPU_MASK: if(g_cpu_port_num >= 0) generate
    cpu_port_mask            <= f_set_bit(zeros,'1',g_cpu_port_num);
  end generate GEN_CPU_MASK;

  -- ??? (legacy)
--   gen_term_unused : for i in g_num_ports to g_num_ports-1 generate
--     rq_strobe_p(i) <= '0';
--     rsp_ack(i)   <= '1';
--   end generate gen_term_unused;

  U_Adapter : wb_slave_adapter
    generic map (
      g_master_use_struct  => true,
      g_master_mode        => CLASSIC,
      g_master_granularity => WORD,
      g_slave_use_struct   => true,
      g_slave_mode         => g_interface_mode,
      g_slave_granularity  => g_address_granularity)
    port map (
      clk_sys_i => clk_sys_i,
      rst_n_i   => rst_n_i,
      slave_i   => wb_i,
      slave_o   => wb_o,
      master_i  => wb_out,
      master_o  => wb_in);

  wb_out.err <= '0';
  wb_out.rty <= '0';
  --------------------------------------------------------------------------------------------
  --| PORTS - g_num_ports number of I/O ports, a port:
  --| - inputs request to REQUEST FIFO
  --| - waits for the response,
  --| - reads response from RESPONSE FIFO  of full match and from fast match
  --------------------------------------------------------------------------------------------
  ports : for i in 0 to (g_num_ports - 1) generate

    U_PortX : rtu_port_new
      generic map (
        g_num_ports      => g_num_ports,
        g_port_mask_bits => g_port_mask_bits,
        g_match_req_fifo_size => g_match_req_fifo_size,
        g_port_index     => i)
      port map(
        clk_i                    => clk_sys_i,
        rst_n_i                  => rst_n_i,

        rtu_idle_o               => port_idle(i), -- TODO: req_full_o ??/
        rtu_rq_i                 => req_i(i),
        rtu_rq_abort_i           => rq_abort_i(i), 
        rtu_rsp_abort_i          => rsp_abort_i(i),--'0', -- new stuff from SWcore
        rtu_rsp_o                => rsp(i),
        rtu_rsp_ack_i            => rsp_ack_i(i),
        
        full_match_wr_req_o      => rq_fifo_wr_access(i),
        full_match_wr_data_o     => rq_fifo_d_requests(i),
        full_match_wr_done_i     => gnt_fifo_access(i),
        full_match_wr_full_i     => rq_fifo_full_for_ports,    
        full_match_rd_data_i     => rsp_data,
        full_match_rd_valid_i    => rsp_valid,

        fast_match_wr_req_o      => fast_match_req(i),
        fast_match_wr_data_o     => fast_match_req_data(i),
        fast_match_rd_valid_i    => fast_match_rsp_valid(i),
        fast_match_rd_data_i     => fast_match_rsp_data,

        port_almost_full_o       => open,
        port_full_o              => open,

--         tru_o                    => rtu2tru_o,

        rtu_str_config_i         => rtu_special_traffic_config,

        rtu_gcr_g_ena_i          => regs_fromwb.gcr_g_ena_o,
        rtu_pcr_pass_bpdu_i      => pcr_pass_bpdu(i),
        rtu_pcr_pass_all_i       => pcr_pass_all(i),
--         rtu_pcr_pass_bpdu_i      => pcr_pass_bpdu,
--         rtu_pcr_pass_all_i       => pcr_pass_all,
        rtu_pcr_fix_prio_i       => pcr_fix_prio(i),
        rtu_pcr_prio_val_i       => pcr_prio_val(i)
        );
        
        -- NOTE: inside {fast,full}_match we also take into account the priority assigned to VLAN,
        --       this value is not taken into account in TRU !!       
        rtu2tru_o.request_valid(i)  <= req_i(i).valid;
        rtu2tru_o.priorities(i)     <= f_pick(pcr_fix_prio(i) = '1', pcr_prio_val(i), req_i(i).prio) when (pcr_fix_prio(i)='1' or req_i(i).has_prio='1') else
                                       (others=>'0');

--         rtu2tru_o.priorities(i)     <= f_pick(pcr_fix_prio(i) = '0', req_i(i).prio, pcr_prio_val(i));
        rtu2tru_o.has_prio(i)       <= '1' ;--req_i(i).has_prio;
   
        rsp_o(i)                    <= rsp(i); -- for RMONs
  end generate;  -- end ports

  rtu2tru_o.pass_all             <= pcr_pass_all;
  rtu2tru_o.forward_bpdu_only    <= pcr_pass_bpdu;   
  ------------------------------------------------------------------------
  -- REQUEST FIFO BUS
  -- Data from all ports into one match module
  ------------------------------------------------------------------------
  U_req_fifo_arbiter : rtu_rr_arbiter
    generic map (
      g_width => g_num_ports)
    port map(
      clk_i   => clk_sys_i,
      rst_n_i => rst_n_i,
      req_i   => rq_fifo_wr_access,
      gnt_o   => gnt_fifo_access
      );

  p_mux_fifo_req : process(rq_fifo_d_requests, gnt_fifo_access)
    variable do_wr   : std_logic;
    variable do_data : std_logic_vector(c_PACKED_REQUEST_WIDTH-1 downto 0);
  begin

    do_data := (others => 'X');
    do_wr   := '0';

    if(gnt_fifo_access = rsp_fifo_read_all_zeros) then
      do_wr := '0';
    else
      do_wr := '1';
    end if;

    for i in 0 to g_num_ports-1 loop
      if(gnt_fifo_access(i) = '1') then
        do_data := rq_fifo_d_requests(i);
      end if;
    end loop;  -- i
    
    rq_fifo_write_sng <= do_wr;
    rq_fifo_d_muxed   <= gnt_fifo_access & do_data;
  end process;

  -----------------------------------------------------------------------------
  -- REQUEST FIFO: takes requests from ports and makes it available for RTU MATCH
  -----------------------------------------------------------------------------
  U_ReqFifo : generic_shiftreg_fifo
    generic map (
      g_data_width => g_num_ports + c_PACKED_REQUEST_WIDTH,
      g_size       => 32
      )
    port map
    (
      rst_n_i   => rst_n_i,
      d_i       => rq_fifo_d_muxed,
      clk_i     => clk_sys_i,
      rd_i      => rq_fifo_read,        --rtu_match
      we_i      => rq_fifo_write_sng,
      q_o       => rq_fifo_data,
      q_valid_o => rq_fifo_qvalid,
      full_o    => rq_fifo_full
      );

  rq_fifo_empty <= not rq_fifo_qvalid;

  p_reqFifo_cnt : process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      if(rst_n_i = '0') then
        match_req_fifo_cnt   <= (others =>'0');
      else
        if(rq_fifo_write_sng = '1' and rq_fifo_read ='0') then
          match_req_fifo_cnt <= match_req_fifo_cnt + 1;
        elsif(rq_fifo_write_sng = '0' and rq_fifo_read ='1') then
          match_req_fifo_cnt <= match_req_fifo_cnt - 1;
        end if;
      end if;
    end if;
  end process p_reqFifo_cnt;

  -- coutning fifo occupanccy (no usecnt provided) to indicate full to port when there
  -- is still N (port number) free places in FIFO (this is in case all the ports make
  -- request at the same time)
  rq_fifo_almost_full <= '1' when (match_req_fifo_cnt > to_unsigned(g_match_req_fifo_size,c_match_req_fifo_size_width)) else '0';
  rq_fifo_empty       <= not rq_fifo_qvalid;
  rq_fifo_full_for_ports <= rq_fifo_full or rq_fifo_almost_full;
  
  ------------------------------------------------------------------------
  -- RTU FAST MATCH 
  -- provides match :
  -- * for special traffic
  -- * based on VLANs and TRU (topology resolution: RSTP/MSTP/LACP)
  -- * deterministic (N+5 cycles)
  -- * forwarding decision when Full Match abandond when it takes too much time
  ------------------------------------------------------------------------

  U_Fast_match: rtu_fast_match
    generic map(
      g_num_ports          => g_num_ports,
      g_port_mask_bits     => g_port_mask_bits
      )
    port map(
      clk_i                => clk_sys_i,     
      rst_n_i              => rst_n_i,     
      match_req_i          => fast_match_req,
      match_req_data_i     => fast_match_req_data,     
      match_rsp_data_o     => fast_match_rsp_data,     
      match_rsp_valid_o    => fast_match_rsp_valid,       
      vtab_rd_addr_o       => fast_match_vtab_addr,     
      vtab_rd_entry_i      => vlan_tab_rd_entry4fast_match,     
      tru_req_o            => tru_req_o,     
      tru_rsp_i            => tru_resp_i,     
      tru_enabled_i        => tru_enabled_i,      
      rtu_str_config_i     => rtu_special_traffic_config,     
      rtu_pcr_pass_all_i   => pcr_pass_all
    );

  --------------------------------------------------------------------------------------------
  --| RTU FULL MATCH: Routing Table Unit Engine
  --------------------------------------------------------------------------------------------
  U_Full_Match : rtu_match
    generic map (
      g_num_ports => g_num_ports)
    port map(

      clk_i                => clk_sys_i,
      rst_n_i              => rst_n_i,
      rq_fifo_read_o       => rq_fifo_read,
      rq_fifo_empty_i      => rq_fifo_empty,
      rq_fifo_input_i      => rq_fifo_data,

      rsp_fifo_write_o     => rsp_valid,
      rsp_fifo_full_i      => '0', --rsp_fifo_full,
      rsp_fifo_output_o    => rsp_data,

      htab_start_o         => htab_start,
      htab_ack_o           => htab_ack,
      htab_found_i         => htab_found,
      htab_hash_o          => htab_hash,
      htab_mac_o           => htab_mac,
      htab_fid_o           => htab_fid,
      htab_drdy_i          => htab_drdy,
      htab_entry_i         => htab_entry,
      htab_port_o    => htab_port,    -- ML (24/03/2013): aging bugfix
      htab_src_dst_o => htab_src_dst, -- ML (24/03/2013): aging bugfix

      rtu_ufifo_wr_req_o   => regs_towb.ufifo_wr_req_i,
      rtu_ufifo_wr_full_i  => regs_fromwb.ufifo_wr_full_o,
      rtu_ufifo_wr_empty_i => regs_fromwb.ufifo_wr_empty_o,
      rtu_ufifo_dmac_lo_o  => regs_towb.ufifo_dmac_lo_i,
      rtu_ufifo_dmac_hi_o  => regs_towb.ufifo_dmac_hi_i,
      rtu_ufifo_smac_lo_o  => regs_towb.ufifo_smac_lo_i,
      rtu_ufifo_smac_hi_o  => regs_towb.ufifo_smac_hi_i,
      rtu_ufifo_vid_o      => regs_towb.ufifo_vid_i,
      rtu_ufifo_prio_o     => regs_towb.ufifo_prio_i,
      rtu_ufifo_pid_o      => regs_towb.ufifo_pid_i,
      rtu_ufifo_has_vid_o  => regs_towb.ufifo_has_vid_i,
      rtu_ufifo_has_prio_o => regs_towb.ufifo_has_prio_i,

      rtu_aram_main_addr_o => aram_main_addr,
      rtu_aram_main_data_i => aram_main_data_i,
      rtu_aram_main_rd_o   => aram_main_rd,
      rtu_aram_main_data_o => aram_main_data_o,
      rtu_aram_main_wr_o   => aram_main_wr,

      vlan_tab_addr_o      => vlan_tab_rd_vid,
      vlan_tab_entry_i     => vlan_tab_rd_entry4match,

      rtu_gcr_g_ena_i      => regs_fromwb.gcr_g_ena_o,
      rtu_pcr_pass_all_i   => pcr_pass_all(g_num_ports - 1 downto 0),
      rtu_pcr_learn_en_i   => pcr_learn_en(g_num_ports - 1 downto 0),
      rtu_pcr_pass_bpdu_i  => pcr_pass_bpdu(g_num_ports - 1 downto 0),
      rtu_pcr_b_unrec_i    => pcr_b_unrec(g_num_ports - 1 downto 0),
      rtu_b_unrec_fw_cpu_i => regs_fromwb.rx_ctr_urec_fw_cpu_ena_o,
      rtu_cpu_mask_i       => cpu_port_mask,
      rtu_crc_poly_i       => rtu_gcr_poly_used  --x"1021"-- x"0589" -- x"8005" --x"1021" --x"8005", --
--    rtu_rw_bank_i                                => s_vlan_bsel
      );

  rtu_gcr_poly_used <= c_default_hash_poly when (regs_fromwb.gcr_poly_val_o = x"0000") else regs_fromwb.gcr_poly_val_o;

  mfifo_trigger <= regs_fromwb.gcr_mfifotrig_o and regs_fromwb.gcr_mfifotrig_load_o;

  U_Lookup : rtu_lookup_engine
    generic map (
      g_num_ports => g_num_ports)
    port map (
      clk_sys_i   => clk_sys_i,
      clk_match_i => clk_sys_i,
      rst_n_i     => rst_n_i,

      mfifo_rd_req_o   => regs_towb.mfifo_rd_req_i,
      mfifo_rd_empty_i => regs_fromwb.mfifo_rd_empty_o,
      mfifo_ad_sel_i   => regs_fromwb.mfifo_ad_sel_o,
      mfifo_ad_val_i   => regs_fromwb.mfifo_ad_val_o,
      mfifo_trigger_i  => mfifo_trigger,
      mfifo_busy_o     => regs_towb.gcr_mfifotrig_i,

      start_i => htab_start,
      ack_i   => htab_ack,
      found_o => htab_found,
      hash_i  => htab_hash,
      mac_i   => htab_mac,
      fid_i   => htab_fid,
      drdy_o  => htab_drdy,
      port_i  => htab_port,      -- ML (24/03/2013): aging bugfix
      src_dst_i => htab_src_dst, -- ML (24/03/2013): aging bugfix      
      entry_o => htab_entry
      );

  --------------------------------------------------------------------------------------------
  --| WISHBONE I/F: interface with CPU and RAM/CAM
  --------------------------------------------------------------------------------------------
--   U_WB_Slave : rtu_wishbone_slave_old
  U_WB_Slave : rtu_wishbone_slave
    port map(
      rst_n_i   => rst_n_i,
      clk_sys_i => clk_sys_i,

      wb_adr_i   => wb_in.adr(8 downto 0),
      wb_dat_i   => wb_in.dat,
      wb_dat_o   => wb_out.dat,
      wb_cyc_i   => wb_in.cyc,
      wb_sel_i   => wb_in.sel,
      wb_stb_i   => wb_in.stb,
      wb_we_i    => wb_in.we,
      wb_ack_o   => wb_out.ack,
      wb_int_o   => wb_out.int,
      wb_stall_o => open,
      clk_match_i => clk_sys_i,
      regs_o => regs_fromwb,
      regs_i => regs_towb,
      irq_nempty_i => irq_nempty,       --'1',
      rtu_aram_addr_i => aram_main_addr,
      rtu_aram_data_o => aram_main_data_i,
      rtu_aram_rd_i   => aram_main_rd,
      rtu_aram_data_i => aram_main_data_o,
      rtu_aram_wr_i   => aram_main_wr
      );  


  current_pcr             <= to_integer(unsigned(regs_fromwb.psr_port_sel_o));
  regs_towb.psr_n_ports_i <= std_logic_vector(to_unsigned(g_num_ports, 8));

  --------------------------------------------------------------------------------------------
  --/  Her we interpret confiration registers (from CPU) provided by WB I/F
  --------------------------------------------------------------------------------------------
  
  -- indirectly addressed PCR registers - this is to allow easy generic-based
  -- scaling of the number of ports
  p_pcr_registers : process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      if(rst_n_i = '0') then

        regs_towb.pcr_learn_en_i   <= '0';
        regs_towb.pcr_pass_all_i   <= '0';
        regs_towb.pcr_pass_bpdu_i  <= '0';
        regs_towb.pcr_fix_prio_i   <= '0';
        regs_towb.pcr_prio_val_i   <= (others => '0');
        regs_towb.pcr_b_unrec_i    <= '0';
        pcr_learn_en               <= (others => '0');
        pcr_pass_all               <= (others => '0');
        pcr_pass_bpdu              <= (others => '0');
        pcr_fix_prio               <= (others => '0');
        pcr_prio_val               <= (others => (others => '0'));
        pcr_b_unrec                <= (others => '0');
    
      else

        regs_towb.pcr_learn_en_i  <= pcr_learn_en(current_pcr);
        regs_towb.pcr_pass_all_i  <= pcr_pass_all(current_pcr);
        regs_towb.pcr_pass_bpdu_i <= pcr_pass_bpdu(current_pcr);
        regs_towb.pcr_fix_prio_i  <= pcr_fix_prio(current_pcr);
        regs_towb.pcr_prio_val_i  <= pcr_prio_val(current_pcr);
        regs_towb.pcr_b_unrec_i   <= pcr_b_unrec(current_pcr);

        if(regs_fromwb.pcr_learn_en_load_o = '1') then
          pcr_learn_en(current_pcr)  <= regs_fromwb.pcr_learn_en_o;
          pcr_pass_all(current_pcr)  <= regs_fromwb.pcr_pass_all_o;
          pcr_pass_bpdu(current_pcr) <= regs_fromwb.pcr_pass_bpdu_o;
          pcr_fix_prio(current_pcr)  <= regs_fromwb.pcr_fix_prio_o;
          pcr_prio_val(current_pcr)  <= regs_fromwb.pcr_prio_val_o;
          pcr_b_unrec(current_pcr)   <= regs_fromwb.pcr_b_unrec_o;
        end if;
      end if;
    end if;
  end process;

  irq_nempty                     <= regs_fromwb.ufifo_wr_empty_o;
  
  current_mac_ID                 <= to_integer(unsigned(regs_fromwb.rx_ff_mac_r1_id_o));
  current_MAC_entry              <= regs_fromwb.rx_ff_mac_r1_hi_id_o & regs_fromwb.rx_ff_mac_r0_lo_o;
  regs_towb.rx_ff_mac_r1_id_i    <= std_logic_vector(to_unsigned(c_ff_single_macs_number, 8));
  regs_towb.rx_ff_mac_r1_hi_id_i <= std_logic_vector(to_unsigned(c_ff_range_macs_number, 16));

  regs_towb.gcr_rtu_version_i    <= x"8";

  -- RTU Extension index-access configration regiters for Fast Forward MACs 
  p_rx_registers : process(clk_sys_i)
  begin
    if rising_edge(clk_sys_i) then
      if(rst_n_i = '0') then   
        rtu_special_traffic_config.single_macs          <= (others => std_logic_vector(to_unsigned(0, 48)));
        rtu_special_traffic_config.single_macs_valid    <= (others => '0');
        rtu_special_traffic_config.macs_range_valid     <= '0';
        rtu_special_traffic_config.macs_range_up        <= (others => '0');
        rtu_special_traffic_config.macs_range_down      <= (others => '0');
        rtu_special_traffic_config.mirror_port_dst      <= (others => '0');
        rtu_special_traffic_config.mirror_port_src_rx   <= (others => '0');
        rtu_special_traffic_config.mirror_port_src_tx   <= (others => '0');
      else
        -- output selected mirror mask    
        if(regs_fromwb.rx_mp_r0_dst_src_o = '0') then  -- mirror destination mask
          regs_towb.rx_mp_r1_mask_i                     <= rtu_special_traffic_config.mirror_port_dst;   
        else                                           -- mirror source mask
          if(regs_fromwb.rx_mp_r0_rx_tx_o = '0') then  -- * rx
            regs_towb.rx_mp_r1_mask_i                   <= rtu_special_traffic_config.mirror_port_src_rx;
          else                                         -- * tx
            regs_towb.rx_mp_r1_mask_i                   <= rtu_special_traffic_config.mirror_port_src_tx;
          end if;               
        end if;
        -- register selected mirror mask
        if(regs_fromwb.rx_mp_r1_mask_load_o = '1') then
          if(regs_fromwb.rx_mp_r0_dst_src_o = '0') then  -- mirror destination mask
            rtu_special_traffic_config.mirror_port_dst      <= regs_fromwb.rx_mp_r1_mask_o;   
          else                                           -- mirror source mask
            if(regs_fromwb.rx_mp_r0_rx_tx_o = '0') then  -- * rx
              rtu_special_traffic_config.mirror_port_src_rx <= regs_fromwb.rx_mp_r1_mask_o;
            else                                         -- * tx
              rtu_special_traffic_config.mirror_port_src_tx <= regs_fromwb.rx_mp_r1_mask_o;
            end if; -- rx_mp_r0_rx_tx_o      
          end if; -- rx_mp_r0_dst_src_o
        end if; -- rx_mp_r1_mask_load_o
        
        -- register selected Fast Forward MAC
        if(regs_fromwb.rx_ff_mac_r1_hi_id_load_o = '1') then
          if(regs_fromwb.rx_ff_mac_r1_type_o = '0') then-- TYPE: single MAC
            rtu_special_traffic_config.single_macs(current_mac_ID)       <= current_MAC_entry;
            rtu_special_traffic_config.single_macs_valid(current_mac_ID) <= regs_fromwb.rx_ff_mac_r1_valid_o;
          else                                           -- TYPE: range MAC
            if(regs_fromwb.rx_ff_mac_r1_id_o(7) = '0') then  --lower range
              rtu_special_traffic_config.macs_range_down      <= current_MAC_entry;            
            else                                                 -- upper range
              rtu_special_traffic_config.macs_range_up        <= current_MAC_entry;         
            end if;
            rtu_special_traffic_config.macs_range_valid       <= regs_fromwb. rx_ff_mac_r1_valid_o;
          end if;
        end if;
      end if;
    end if;
  end process;

  rtu_special_traffic_config.hp_prio              <= regs_fromwb.rx_ctr_prio_mask_o;
  rtu_special_traffic_config.cpu_forward_mask     <= cpu_port_mask; --regs_fromwb.rx_llf_ff_mask_o;
  rtu_special_traffic_config.dop_on_fmatch_full   <= regs_fromwb.rx_ctr_at_fmatch_too_slow_o;
  rtu_special_traffic_config.ff_mac_br_ena        <= regs_fromwb.rx_ctr_ff_mac_br_o;
  rtu_special_traffic_config.ff_mac_range_ena     <= regs_fromwb.rx_ctr_ff_mac_range_o;
  rtu_special_traffic_config.ff_mac_single_ena    <= regs_fromwb.rx_ctr_ff_mac_single_o;
  rtu_special_traffic_config.ff_mac_ll_ena        <= regs_fromwb.rx_ctr_ff_mac_ll_o;
  rtu_special_traffic_config.ff_mac_ptp_ena       <= regs_fromwb.rx_ctr_ff_mac_ptp_o;
  rtu_special_traffic_config.mr_ena               <= regs_fromwb.rx_ctr_mr_ena_o;
  rtu_special_traffic_config.hp_fw_cpu_ena        <= regs_fromwb.rx_ctr_hp_fw_cpu_ena_o;
  rtu_special_traffic_config.unrec_fw_cpu_ena     <= regs_fromwb.rx_ctr_urec_fw_cpu_ena_o;
  regs_towb.cpu_port_mask_i                       <= cpu_port_mask;
  rtu_special_traffic_config.dbg_force_fast_match_only <= regs_fromwb.rx_ctr_force_fast_match_ena_o;
  rtu_special_traffic_config.dbg_force_full_match_only <= regs_fromwb.rx_ctr_force_full_match_ena_o;
  --------------------------------------------------------------------------------------------
  --| VLAN memories 
  --| * one used by Full Match
  --| * one used by Fast Match
  --------------------------------------------------------------------------------------------
  U_VLAN_Table_for_full_match : generic_dpram
    generic map (
      g_data_width       => c_VLAN_TAB_ENTRY_WIDTH,
      g_size             => 4096,
      g_with_byte_enable => false,
      g_dual_clock       => false)
    port map (
      rst_n_i => rst_n_i,

      clka_i  => clk_sys_i,
      bwea_i  => (others => '1'),
      wea_i   => regs_fromwb.vtr1_update_o,
      aa_i    => regs_fromwb.vtr1_vid_o,
      da_i    => vlan_tab_wr_data,
      qa_o    => open,

      clkb_i  => clk_sys_i, --'0',
      bweb_i  => (others => '1'),
      web_i   => '0',
      ab_i    => vlan_tab_rd_vid,
      db_i    => (others => '0'),
      qb_o    => vlan_tab_rd_data4match);

  U_VLAN_Table_for_fast_match : generic_dpram
    generic map (
      g_data_width       => c_VLAN_TAB_ENTRY_WIDTH,
      g_size             => 4096,
      g_with_byte_enable => false,
      g_dual_clock       => false)
    port map (
      rst_n_i => rst_n_i,

      clka_i  => clk_sys_i,
      bwea_i  => (others => '1'),
      wea_i   => regs_fromwb.vtr1_update_o,
      aa_i    => regs_fromwb.vtr1_vid_o,
      da_i    => vlan_tab_wr_data,
      qa_o    => open,

      clkb_i => clk_sys_i, --'0',
      bweb_i  => (others => '1'),
      ab_i    => fast_match_vtab_addr,  -- address
      db_i    => (others => '0'),
      qb_o    => fast_match_vtab_data); -- data

  vlan_tab_wr_data <= regs_fromwb.vtr2_port_mask_o
                      & regs_fromwb.vtr1_fid_o
                      & regs_fromwb.vtr1_drop_o
                      & regs_fromwb.vtr1_prio_override_o
                      & regs_fromwb.vtr1_prio_o
                      & regs_fromwb.vtr1_has_prio_o;

  f_unpack6(vlan_tab_rd_data4match,
            vlan_tab_rd_entry4match.port_mask,
            vlan_tab_rd_entry4match.fid,
            vlan_tab_rd_entry4match.drop,
            vlan_tab_rd_entry4match.prio_override,
            vlan_tab_rd_entry4match.prio,
            vlan_tab_rd_entry4match.has_prio);

  f_unpack6(fast_match_vtab_data,
            vlan_tab_rd_entry4fast_match.port_mask,
            vlan_tab_rd_entry4fast_match.fid,
            vlan_tab_rd_entry4fast_match.drop,
            vlan_tab_rd_entry4fast_match.prio_override,
            vlan_tab_rd_entry4fast_match.prio,
            vlan_tab_rd_entry4fast_match.has_prio);

  events_gen: if(g_rmon_events_bits_pp = 9 ) generate
    rmon_events_gen: for i in 0 to (g_num_ports - 1) generate
      rmon_events_o((i+1)*g_rmon_events_bits_pp-1 downto i*g_rmon_events_bits_pp) <= 
        dbg_forwarded_to_port(i)                                      & -- 8: forwarded to port 
        std_logic(rsp_valid and rsp_data(i))                          & -- 7: FullMatch: resp valid
        fast_match_rsp_valid(i)                                       & -- 6: FastMatch: resp valid
        std_logic(fast_match_rsp_valid(i) and fast_match_rsp_data.nf) & -- 5: FastMatch: non-forward (as config)
        std_logic(fast_match_rsp_valid(i) and fast_match_rsp_data.ff) & -- 4: FastMatch: fast forward (as config)                   
        std_logic(fast_match_rsp_valid(i) and fast_match_rsp_data.hp) & -- 3: FastMatch: high priority frames
        std_logic(rsp(i).valid and rsp_ack_i(i) and rsp(i).drop     ) & -- 2: dropped
        std_logic(rsp(i).valid and rsp_ack_i(i)                     ) & -- 1: valid respons
        std_logic(req_i(i).valid                                    ) ; -- 0: valid request 
    end generate rmon_events_gen;
  end generate events_gen;
  no_events_gen: if(g_rmon_events_bits_pp /= 9 ) generate
    rmon_events_o <= (others=>'0'); 
    assert true report "g_rmon_events_bits_pp not equal to the defined number of RMON event ";
  end generate no_events_gen;
  
  fw_gen : for i in 0 to (g_num_ports - 1) generate
    egress_port_p: process(rsp, rsp_ack_i)
    variable fw : std_logic;
    begin
      fw := '0';
      L0: for j in 0 to (g_num_ports -1) loop
        fw := fw or (rsp(j).valid and rsp(j).port_mask(i) and rsp_ack_i(j) and not rsp(j).drop);
      end loop;
      dbg_forwarded_to_port(i) <= fw;
    end process;
  end generate fw_gen;

--   CS_ICON : chipscope_icon
--    port map (
--     CONTROL0 => CONTROL0);
--   CS_ILA : chipscope_ila
--    port map (
--      CONTROL => CONTROL0,
--      CLK     => clk_sys_i,
--      TRIG0   => TRIG0,
--      TRIG1   => TRIG1,
--      TRIG2   => TRIG2,
--      TRIG3   => TRIG3);
-- 
-- RAM
--   TRIG0(11  downto   0)  <= regs_fromwb.vtr1_vid_o;
--   TRIG0(23  downto  12)  <= fast_match_vtab_addr;
--   TRIG0(            24)  <= regs_fromwb.vtr1_update_o;
-- 
--   TRIG1(31  downto   0)  <= vlan_tab_wr_data(31 downto 0);
--   TRIG2(13  downto   0)  <= vlan_tab_wr_data(45 downto 32);
--   TRIG2(27  downto  14)  <= fast_match_vtab_data(45 downto 32);
--   TRIG3(31  downto   0)  <= fast_match_vtab_data(31 downto 0);

----------------------- port ----------------------
--   TRIG0(31     downto   0) <= fast_match_req_data(0).smac(31 downto 0);
--   
--   TRIG1(11     downto   0) <= fast_match_req_data(0).vid;
--   TRIG1(14     downto  12) <= fast_match_req_data(0).prio;
--   TRIG1(               15) <= fast_match_req_data(0).valid;
--   TRIG1(               16) <= fast_match_req_data(0).has_vid;
--   TRIG1(               17) <= fast_match_req_data(0).has_prio;
--   TRIG1(               18) <= fast_match_req(0);
-- 
--   TRIG1(               19) <= fast_match_rsp_valid(0);
--   TRIG1(               20) <= fast_match_rsp_data(0).valid;
--   TRIG1(23     downto  21) <= fast_match_rsp_data(0).prio;
--   TRIG1(31     downto  24) <= fast_match_rsp_data(0).port_mask(7 downto 0);
-- 
--   TRIG2(31     downto   0) <= fast_match_req_data(2).smac(31 downto 0);
--   
--   TRIG3(11     downto   0) <= fast_match_req_data(2).vid;
--   TRIG3(14     downto  12) <= fast_match_req_data(2).prio;
--   TRIG3(               15) <= fast_match_req_data(2).valid;
--   TRIG3(               16) <= fast_match_req_data(2).has_vid;
--   TRIG3(               17) <= fast_match_req_data(2).has_prio;
--   TRIG3(               18) <= fast_match_req(2);
-- 
--   TRIG3(               19) <= fast_match_rsp_valid(2);
--   TRIG3(               20) <= fast_match_rsp_data(2).valid;
--   TRIG3(23     downto  21) <= fast_match_rsp_data(2).prio;
--   TRIG3(31     downto  24) <= fast_match_rsp_data(2).port_mask(7 downto 0);


end behavioral;
