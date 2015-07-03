-------------------------------------------------------------------------------
-- Title      : WR Switch bare top level
-- Project    : White Rabbit Switch
-------------------------------------------------------------------------------
-- File       : scb_top_bare.vhd
-- Author     : Tomasz Wlostowski, Maciej Lipinski, Grzegorz Daniluk
-- Company    : CERN BE-CO-HT
-- Created    : 2012-02-21
-- Last update: 2014-03-17
-- Platform   : FPGA-generic
-- Standard   : VHDL
-------------------------------------------------------------------------------
-- Description:
-- Bare switch top module, without GTX transceivers and CPU bridge.
-------------------------------------------------------------------------------
--
-- Copyright (c) 2012 - 2014 CERN / BE-CO-HT
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

library ieee;
use ieee.STD_LOGIC_1164.all;
use ieee.std_logic_misc.all;
use ieee.numeric_std.all;
use ieee.math_real.CEIL;
use work.wishbone_pkg.all;
use work.gencores_pkg.all;
use work.wr_fabric_pkg.all;
use work.endpoint_pkg.all;
use work.wrsw_txtsu_pkg.all;
use work.hwinfo_pkg.all;
use work.wrsw_top_pkg.all;
use work.wrsw_shared_types_pkg.all;
use work.wrsw_tru_pkg.all;
use work.wrsw_tatsu_pkg.all;
use work.wrs_sdb_pkg.all;
use work.wrs_dbg_pkg.all;

library UNISIM;
use UNISIM.vcomponents.all;

entity scb_top_bare is
  generic(
    g_num_ports       : integer := 6;
    g_simulation      : boolean := false;
    g_without_network : boolean := false;
    g_with_TRU        : boolean := false;
    g_with_TATSU      : boolean := false;
    g_with_HWIU       : boolean := false;
    g_with_PSTATS     : boolean := true;
    g_with_muxed_CS   : boolean := false;
    g_with_watchdog   : boolean := false;
    g_inj_per_EP      : std_logic_vector(17 downto 0) := (others=>'0')
    );
  port (
    sys_rst_n_i : in std_logic;         -- global reset

    -- Startup 25 MHz clock (from onboard 25 MHz oscillator)
    clk_startup_i : in std_logic;

    -- 62.5 MHz timing reference (from the AD9516 PLL output QDRII_CLK)
    clk_ref_i : in std_logic;

    -- 62.5+ MHz DMTD offset clock (from the CDCM62001 PLL output DMTDCLK_MAIN)
    clk_dmtd_i : in std_logic;

    -- Programmable aux clock (from the AD9516 PLL output QDRII_200CLK). Used
    -- for re-phasing the 10 MHz input as well as clocking the 
    clk_aux_i : in std_logic;

		clk_ext_mul_i	:	in std_logic;

    clk_aux_p_o  : out std_logic; -- going to CLK2 SMC on the front pannel, by
    clk_aux_n_o  : out std_logic; -- default it's 10MHz, but is configurable

    clk_500_o    :  out std_logic;

    -- Muxed system clock
    clk_sys_o : out std_logic;

    -------------------------------------------------------------------------------
    -- Master wishbone bus (from the CPU bridge)
    -------------------------------------------------------------------------------
    cpu_wb_i    : in  t_wishbone_slave_in;
    cpu_wb_o    : out t_wishbone_slave_out;
    cpu_irq_n_o : out std_logic;

    -------------------------------------------------------------------------------
    -- Timing I/O
    -------------------------------------------------------------------------------    

    pps_i : in  std_logic;
    pps_o : out std_logic;

    -- DAC Drive
    dac_helper_sync_n_o : out std_logic;
    dac_helper_sclk_o   : out std_logic;
    dac_helper_data_o   : out std_logic;

    dac_main_sync_n_o : out std_logic;

    dac_main_sclk_o : out std_logic;
    dac_main_data_o : out std_logic;

    -------------------------------------------------------------------------------
    -- AD9516 PLL Control signals
    -------------------------------------------------------------------------------    

    pll_status_i  : in  std_logic;
    pll_mosi_o    : out std_logic;
    pll_miso_i    : in  std_logic;
    pll_sck_o     : out std_logic;
    pll_cs_n_o    : out std_logic;
    pll_sync_n_o  : out std_logic;
    pll_reset_n_o : out std_logic;

    uart_txd_o : out std_logic;
    uart_rxd_i : in  std_logic;

    -------------------------------------------------------------------------------
    -- Misc pins
    -------------------------------------------------------------------------------

    -- GTX clock fanout enable
    clk_en_o : out std_logic;

    -- GTX clock fanout source select
    clk_sel_o : out std_logic;

    -- DMTD clock divider selection (0 = 125 MHz, 1 = 62.5 MHz)
    clk_dmtd_divsel_o : out std_logic;

    -- UART source selection (FPGA/DBGU)
    uart_sel_o : out std_logic;

    ---------------------------------------------------------------------------
    -- GTX ports
    ---------------------------------------------------------------------------

    phys_o : out t_phyif_output_array(g_num_ports-1 downto 0);
    phys_i : in  t_phyif_input_array(g_num_ports-1 downto 0);

    led_link_o : out std_logic_vector(g_num_ports-1 downto 0);
    led_act_o  : out std_logic_vector(g_num_ports-1 downto 0);

    gpio_o : out std_logic_vector(31 downto 0);
    gpio_i : in  std_logic_vector(31 downto 0);

    ---------------------------------------------------------------------------
    -- I2C I/Os
    -- mapping: 0/1 -> MiniBackplane busses 0/1
    --          2   -> Onboard temp sensors
    ---------------------------------------------------------------------------

    i2c_scl_oen_o : out std_logic_vector(2 downto 0);
    i2c_scl_o     : out std_logic_vector(2 downto 0);
    i2c_scl_i     : in  std_logic_vector(2 downto 0) := "111";
    i2c_sda_oen_o : out std_logic_vector(2 downto 0);
    i2c_sda_o     : out std_logic_vector(2 downto 0);
    i2c_sda_i     : in  std_logic_vector(2 downto 0) := "111";

    ---------------------------------------------------------------------------
    -- Mini-backplane PWM fans
    ---------------------------------------------------------------------------

    mb_fan1_pwm_o : out std_logic;
    mb_fan2_pwm_o : out std_logic;

    spll_dbg_o  : out std_logic_vector(5 downto 0)
    );
end scb_top_bare;

architecture rtl of scb_top_bare is

  constant c_GW_VERSION    : std_logic_vector(31 downto 0) := x"20_02_14_00"; --DD_MM_YY_VV
  constant c_NUM_WB_SLAVES : integer := 14;
  constant c_NUM_PORTS     : integer := g_num_ports;
  constant c_MAX_PORTS     : integer := 18;
  constant c_NUM_GL_PAUSE  : integer := 2; -- number of output global PAUSE sources for SWcore
  constant c_RTU_EVENTS    : integer := 9; -- number of RMON events per port
  constant c_DBG_V_SWCORE  : integer := (3*10) + 2;         -- 3 resources, each has with of CNT of 10 bits +2 to make it 32
  constant c_DBG_N_REGS    : integer := 1 + integer(ceil(real(c_DBG_V_SWCORE)/real(32))); -- 32-bits debug registers which go to HWIU
  constant c_TRU_EVENTS    : integer := 1;
  constant c_ALL_EVENTS    : integer := c_TRU_EVENTS + c_RTU_EVENTS + c_epevents_sz;
  constant c_DUMMY_RMON    : boolean := false; -- define TRUE to enable dummy_rmon module for debugging PSTAT
  constant c_NUM_GPIO_PINS : integer := 1;
  constant c_NUM_IRQS      : integer := 4;
--   constant c_epevents_sz   : integer := 15;
-------------------------------------------------------------------------------
-- Interconnect & memory layout
-------------------------------------------------------------------------------  

  constant c_SLAVE_RT_SUBSYSTEM : integer := 0;
  constant c_SLAVE_NIC          : integer := 1;
  constant c_SLAVE_ENDPOINTS    : integer := 2;
  constant c_SLAVE_VIC          : integer := 3;
  constant c_SLAVE_TXTSU        : integer := 4;
  constant c_SLAVE_RTU          : integer := 5;
  constant c_SLAVE_GPIO         : integer := 6;
  constant c_SLAVE_I2C          : integer := 7;
  constant c_SLAVE_PWM          : integer := 8;
  constant c_SLAVE_TRU          : integer := 9;
  constant c_SLAVE_TATSU        : integer := 10;
  constant c_SLAVE_PSTATS       : integer := 11;
  constant c_SLAVE_HWIU         : integer := 12;
  constant c_SLAVE_WDOG         : integer := 13;
  --constant c_SLAVE_DUMMY        : integer := 13;

  type t_dbg_ep_array is array (natural range <>) of t_dbg_ep;

  function f_bool2int(x : boolean) return integer is
  begin
    if(x) then
      return 1;
    else
      return 0;
    end if;
  end f_bool2int;

  function f_logic2bool(x : std_logic) return boolean is
  begin
    if(x = '1') then
      return true;
    else
      return false;
    end if;
  end f_logic2bool;


  signal cnx_slave_in  : t_wishbone_slave_in_array(0 downto 0);
  signal cnx_slave_out : t_wishbone_slave_out_array(0 downto 0);

  signal bridge_master_in  : t_wishbone_master_in;
  signal bridge_master_out : t_wishbone_master_out;

  signal cnx_master_in  : t_wishbone_master_in_array(c_NUM_WB_SLAVES-1 downto 0);
  signal cnx_master_out : t_wishbone_master_out_array(c_NUM_WB_SLAVES-1 downto 0);

  signal cnx_endpoint_in  : t_wishbone_master_in_array(c_MAX_PORTS-1 downto 0);
  signal cnx_endpoint_out : t_wishbone_master_out_array(c_MAX_PORTS-1 downto 0);

  -------------------------------------------------------------------------------
  -- Clocks
  -------------------------------------------------------------------------------

  signal clk_sys    : std_logic;
  signal clk_rx_vec : std_logic_vector(c_NUM_PORTS-1 downto 0);


-------------------------------------------------------------------------------
-- Fabric/Endpoint interconnect
-------------------------------------------------------------------------------

  signal endpoint_src_out : t_wrf_source_out_array(c_NUM_PORTS downto 0);
  signal endpoint_src_in  : t_wrf_source_in_array(c_NUM_PORTS downto 0);
  signal endpoint_snk_out : t_wrf_sink_out_array(c_NUM_PORTS downto 0);
  signal endpoint_snk_in  : t_wrf_sink_in_array(c_NUM_PORTS downto 0);

  signal wrfreg_src_out : t_wrf_source_out_array(c_NUM_PORTS downto 0);
  signal wrfreg_src_in  : t_wrf_source_in_array(c_NUM_PORTS downto 0);

  signal swc_src_out : t_wrf_source_out_array(c_NUM_PORTS downto 0);
  signal swc_src_in  : t_wrf_source_in_array(c_NUM_PORTS downto 0);
  signal swc_snk_out : t_wrf_sink_out_array(c_NUM_PORTS downto 0);
  signal swc_snk_in  : t_wrf_sink_in_array(c_NUM_PORTS downto 0);

  signal dummy_snk_in  : t_wrf_sink_in_array(c_NUM_PORTS downto 0);
  signal dummy_src_in  : t_wrf_source_in_array(c_NUM_PORTS downto 0);
  signal dummy_src_out : t_wrf_source_out_array(c_NUM_PORTS downto 0);


  signal rtu_req                            : t_rtu_request_array(c_NUM_PORTS downto 0);
  signal rtu_rsp                            : t_rtu_response_array(c_NUM_PORTS downto 0);
  signal rtu_req_ack, rtu_full, rtu_rsp_ack, rtu_rq_abort, rtu_rsp_abort: std_logic_vector(c_NUM_PORTS downto 0);
  signal swc_rtu_ack  : std_logic_vector(c_NUM_PORTS downto 0);

-- System clock selection: 0 = startup clock, 1 = PLL clock
  signal sel_clk_sys, sel_clk_sys_int : std_logic;
  signal switchover_cnt               : unsigned(4 downto 0);

  signal rst_n_sys  : std_logic;
  signal pps_p_main : std_logic;

  signal txtsu_timestamps_ack : std_logic_vector(c_NUM_PORTS-1 downto 0);
  signal txtsu_timestamps     : t_txtsu_timestamp_array(c_NUM_PORTS-1 downto 0);
  signal tru_enabled          : std_logic;

  -- PSTAT: RMON counters
  signal rtu_events  : std_logic_vector(c_NUM_PORTS*c_RTU_EVENTS  -1 downto 0);  --
  signal ep_events   : std_logic_vector(c_NUM_PORTS*c_epevents_sz -1 downto 0);  --
  signal rmon_events : std_logic_vector(c_NUM_PORTS*c_ALL_EVENTS  -1 downto 0);  --

  --TEMP
  signal dummy_events : std_logic_vector(c_NUM_PORTS*2-1 downto 0);

  -----------------------------------------------------------------------------
  -- Component declarations
  -----------------------------------------------------------------------------

  signal vic_irqs : std_logic_vector(c_NUM_IRQS-1 downto 0);
  type t_trig is array(integer range <>) of std_logic_vector(31 downto 0);

  signal control0                   : std_logic_vector(35 downto 0);
  signal trig0, trig1, trig2, trig3, trig4, trig5, trig6, trig7, trig8, trig9,
         trig10,trig11 : t_trig(7 downto 0);
  signal t0, t1, t2, t3, t4, t5, t6, t7, t8, t9,
         t10, t11 : std_logic_vector(31 downto 0);
  signal rst_n_periph               : std_logic;
  signal link_kill                  : std_logic_vector(c_NUM_PORTS-1 downto 0);
  signal rst_n_swc  : std_logic;
  signal swc_nomem  : std_logic;
  signal swc_wdog_out : t_swc_fsms_array(c_NUM_PORTS downto 0);
  signal ep_stop_traffic : std_logic;

 



  function f_fabric_2_slv (
    in_i : t_wrf_sink_in;
    in_o : t_wrf_sink_out) return std_logic_vector is
    variable tmp : std_logic_vector(31 downto 0);
  begin
    tmp(15 downto 0)  := in_i.dat;
    tmp(17 downto 16) := in_i.adr;
    tmp(19 downto 18) := in_i.sel;
    tmp(20)           := in_i.cyc;
    tmp(21)           := in_i.stb;
    tmp(22)           := in_i.we;
    tmp(23)           := in_o.ack;
    tmp(24)           := in_o.stall;
    tmp(25)           := in_o.err;
    tmp(26)           := in_o.rty;
    return tmp;
  end f_fabric_2_slv;

  function f_swc_ratio return integer is
  begin
    if(g_num_ports < 12) then
      return 4;
    else
      return 6;
    end if;
  end f_swc_ratio;


  signal cpu_irq_n            : std_logic;
  signal pps_csync, pps_valid : std_logic;



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
      TRIG3   : in    std_logic_vector(31 downto 0);
      TRIG4   : in    std_logic_vector(31 downto 0);
      TRIG5   : in    std_logic_vector(31 downto 0);
      TRIG6   : in    std_logic_vector(31 downto 0);
      TRIG7   : in    std_logic_vector(31 downto 0);
      TRIG8   : in    std_logic_vector(31 downto 0);
      TRIG9   : in    std_logic_vector(31 downto 0);
      TRIG10  : in    std_logic_vector(31 downto 0);
      TRIG11  : in    std_logic_vector(31 downto 0));
  end component;

  signal gpio_out : std_logic_vector(c_NUM_GPIO_PINS-1 downto 0);
  signal gpio_in  : std_logic_vector(c_NUM_GPIO_PINS-1 downto 0);
  signal dummy    : std_logic_vector(c_NUM_GPIO_PINS-1 downto 0);

  -----------------------------------------------------------------------------
  -- TRU stuff
  -----------------------------------------------------------------------------
  signal tru_req    : t_tru_request;
  signal tru_resp   : t_tru_response;    
  signal rtu2tru    : t_rtu2tru;
  signal ep2tru     : t_ep2tru_array(g_num_ports-1 downto 0);
  signal tru2ep     : t_tru2ep_array(g_num_ports-1 downto 0);
  signal swc2tru_req: t_global_pause_request; -- for pause
  -----------------------------------------------------------------------------
  -- Time-Aware Traffic Shaper
  -----------------------------------------------------------------------------

  signal tm_utc              : std_logic_vector(39 downto 0);
  signal tm_cycles           : std_logic_vector(27 downto 0);
  signal tm_time_valid       : std_logic;
  signal pps_o_predelay      : std_logic;
  signal ppsdel_tap_out      : std_logic_vector(4 downto 0);
  signal ppsdel_tap_in       : std_logic_vector(4 downto 0);
  signal ppsdel_tap_wr_in    : std_logic;
  signal shaper_request      : t_global_pause_request;
  signal shaper_drop_at_hp_ena : std_logic;
  signal   fc_rx_pause       : t_pause_request_array(g_num_ports+1-1 downto 0);
  constant c_zero_pause      : t_pause_request        :=('0',x"0000", x"00");
  constant c_zero_gl_pause   : t_global_pause_request :=('0',x"0000", x"00",(others=>'0'));
  signal global_pause        : t_global_pause_request_array(c_NUM_GL_PAUSE-1 downto 0);

  signal dbg_n_regs          : std_logic_vector(c_DBG_N_REGS*32 -1 downto 0);
  
  type t_ep_dbg_data_array   is array(integer range <>) of std_logic_vector(15 downto 0);
  type t_ep_dbg_k_array      is array(integer range <>) of std_logic_vector(1 downto 0);
  type t_ep_dbg_rx_buf_array is array(integer range <>) of std_logic_vector(7 downto 0);
  type t_ep_dbg_fab_pipes_array is array(integer range <>) of std_logic_vector(63 downto 0);
  type t_ep_dbg_tx_pcs_array is array(integer range <>) of std_logic_vector(5+4 downto 0);

  signal ep_dbg_data_array   : t_ep_dbg_data_array(g_num_ports-1 downto 0);
  signal ep_dbg_k_array      : t_ep_dbg_k_array(g_num_ports-1 downto 0);
  signal ep_dbg_rx_buf_array : t_ep_dbg_rx_buf_array(g_num_ports-1 downto 0);
  signal ep_dbg_fab_pipes_array : t_ep_dbg_fab_pipes_array(g_num_ports-1 downto 0);
  signal ep_dbg_tx_pcs_wr_array : t_ep_dbg_tx_pcs_array(g_num_ports-1 downto 0);
  signal ep_dbg_tx_pcs_rd_array : t_ep_dbg_tx_pcs_array(g_num_ports-1 downto 0);
  signal dbg_chps_id          : std_logic_vector(7 downto 0);
  signal swc_dbg : t_dbg_swc;
  signal ep_dbg  : t_dbg_ep_array(g_num_ports-1 downto 0);
  signal ep_dbg_synced : t_dbg_ep_array(g_num_ports-1 downto 0);

  -- debug
  type t_dbg_rtu_cnt is array(integer range <>) of unsigned(7 downto 0);
  signal dbg_rtu_cnt : t_dbg_rtu_cnt(g_num_ports downto 0);
  signal dbg_rtu_bug : t_dbg_rtu_cnt(g_num_ports downto 0);
  type t_dbg_cnt is array(integer range <>) of unsigned(11 downto 0);
  signal dbg_cnt_eq  : t_dbg_cnt(g_num_ports-1 downto 0);
  signal dbg_cnt_dif : t_dbg_cnt(g_num_ports-1 downto 0);
  signal hwiu_dbg1 : std_logic_vector(31 downto 0);
  signal hwiu_dbg2 : std_logic_vector(31 downto 0);
  signal hwiu_val1 : unsigned(31 downto 0);
  signal hwiu_val2 : unsigned(31 downto 0);
  
begin



  --CS_ICON : chipscope_icon
  --  port map (
  --   CONTROL0 => CONTROL0);
  --CS_ILA : chipscope_ila
  --  port map (
  --    CONTROL => CONTROL0,

  --    CLK     => clk_sys,
  --    TRIG0   => TRIG0,
  --    TRIG1   => TRIG1,
  --    TRIG2   => TRIG2,
  --    TRIG3   => TRIG3);


  cnx_slave_in(0) <= cpu_wb_i;
  cpu_wb_o        <= cnx_slave_out(0);

  --TRIG0 <= cpu_wb_i.adr;
  --TRIG1 <= cpu_wb_i.dat;
  --TRIG3 <= cnx_slave_out(0).dat;
  --TRIG2(0) <= cpu_wb_i.cyc;
  --TRIG2(1) <= cpu_wb_i.stb;
  --TRIG2(2) <= cpu_wb_i.we;
  --TRIG2(3) <= cnx_slave_out(0).stall;
  --TRIG2(4) <= cnx_slave_out(0).ack;

  U_Sys_Clock_Mux : BUFGMUX
    generic map (
      CLK_SEL_TYPE => "SYNC")
    port map (
      O  => clk_sys,
      I0 => clk_startup_i,
      I1 => clk_ref_i,                  -- both are 62.5 MHz
      S  => sel_clk_sys_int);



  U_Intercon : xwb_sdb_crossbar
    generic map (
      g_num_masters => 1,
      g_num_slaves  => c_NUM_WB_SLAVES,
      g_registered  => true,
      g_wraparound  => true,
      g_layout      => c_layout,
      g_sdb_addr    => c_sdb_address)
    port map (
      clk_sys_i => clk_sys,
      rst_n_i   => rst_n_sys,
      slave_i   => cnx_slave_in,
      slave_o   => cnx_slave_out,
      master_i  => cnx_master_in,
      master_o  => cnx_master_out);


  U_sync_reset : gc_sync_ffs
    port map (
      clk_i    => clk_sys,
      rst_n_i  => '1',
      data_i   => sys_rst_n_i,
      synced_o => rst_n_sys);

  p_gen_sel_clk_sys : process(sys_rst_n_i, clk_sys)
  begin
    if sys_rst_n_i = '0' then
      sel_clk_sys_int <= '0';
      switchover_cnt  <= (others => '0');
    elsif rising_edge(clk_sys) then
      if(switchover_cnt = "11111") then
        sel_clk_sys_int <= sel_clk_sys;
      else
        switchover_cnt <= switchover_cnt + 1;
      end if;
    end if;
  end process;


  U_RT_Subsystem : wrsw_rt_subsystem
    generic map (
      g_num_rx_clocks => c_NUM_PORTS,
      g_simulation    => g_simulation)
    port map (
      clk_ref_i           => clk_ref_i,
      clk_sys_i           => clk_sys,
      clk_dmtd_i          => clk_dmtd_i,
      clk_rx_i            => clk_rx_vec,
      clk_ext_i           => pll_status_i,  -- FIXME: UGLY HACK
			clk_ext_mul_i				=> clk_ext_mul_i,
      clk_aux_p_o         => clk_aux_p_o,
      clk_aux_n_o         => clk_aux_n_o,
      clk_500_o           => clk_500_o,
      rst_n_i             => rst_n_sys,
      rst_n_o             => rst_n_periph,
      wb_i                => cnx_master_out(c_SLAVE_RT_SUBSYSTEM),
      wb_o                => cnx_master_in(c_SLAVE_RT_SUBSYSTEM),
      dac_helper_sync_n_o => dac_helper_sync_n_o,
      dac_helper_sclk_o   => dac_helper_sclk_o,
      dac_helper_data_o   => dac_helper_data_o,
      dac_main_sync_n_o   => dac_main_sync_n_o,
      dac_main_sclk_o     => dac_main_sclk_o,
      dac_main_data_o     => dac_main_data_o,
      uart_txd_o          => uart_txd_o,
      uart_rxd_i          => uart_rxd_i,

      pps_csync_o => pps_csync,
      pps_valid_o => pps_valid,
      pps_ext_i   => pps_i,
      pps_ext_o   => pps_o_predelay,

      sel_clk_sys_o => sel_clk_sys,

      ppsdel_tap_i    => ppsdel_tap_out,
      ppsdel_tap_o    => ppsdel_tap_in,
      ppsdel_tap_wr_o => ppsdel_tap_wr_in,

      tm_utc_o            => tm_utc, 
      tm_cycles_o         => tm_cycles, 
      tm_time_valid_o     => tm_time_valid, 

      pll_status_i  => '0',
      pll_mosi_o    => pll_mosi_o,
      pll_miso_i    => pll_miso_i,
      pll_sck_o     => pll_sck_o,
      pll_cs_n_o    => pll_cs_n_o,
      pll_sync_n_o  => pll_sync_n_o,
      pll_reset_n_o => pll_reset_n_o,
      spll_dbg_o    => spll_dbg_o);

  U_DELAY_PPS: IODELAYE1
    generic map (
      CINVCTRL_SEL           => FALSE,
      DELAY_SRC              => "O",
      HIGH_PERFORMANCE_MODE  => TRUE,
      IDELAY_TYPE            => "FIXED",
      IDELAY_VALUE           => 0,
      ODELAY_TYPE            => "VAR_LOADABLE",
      ODELAY_VALUE           => 0,
      REFCLK_FREQUENCY       => 200.0,
      SIGNAL_PATTERN         => "DATA")
    port map (
      DATAOUT                => pps_o,
      DATAIN                 => '0',
      C                      => clk_sys,
      CE                     => '0',
      INC                    => '0',
      IDATAIN                => '0',
      ODATAIN                => pps_o_predelay,
      RST                    => ppsdel_tap_wr_in,
      T                      => '0',
      CNTVALUEIN             => ppsdel_tap_in,
      CNTVALUEOUT            => ppsdel_tap_out,
      CLKIN                  => '0',
      CINVCTRL               => '0');

  U_IRQ_Controller : xwb_vic
    generic map (
      g_interface_mode      => PIPELINED,
      g_address_granularity => BYTE,
      g_num_interrupts      => c_NUM_IRQS)
    port map (
      clk_sys_i    => clk_sys,
      rst_n_i      => rst_n_sys,
      slave_i      => cnx_master_out(c_SLAVE_VIC),
      slave_o      => cnx_master_in(c_SLAVE_VIC),
      irqs_i       => vic_irqs,
      irq_master_o => cpu_irq_n_o);

  gen_network_stuff : if(g_without_network = false) generate
    
    U_Nic : xwrsw_nic
      generic map (
        g_interface_mode      => PIPELINED,
        g_address_granularity => BYTE,
        g_port_mask_bits      => c_NUM_PORTS+1,
        g_rx_untagging        => false)
      port map (
        clk_sys_i           => clk_sys,
        rst_n_i             => rst_n_sys,
        snk_i               => endpoint_snk_in(c_NUM_PORTS),
        snk_o               => endpoint_snk_out(c_NUM_PORTS),
        src_i               => endpoint_src_in(c_NUM_PORTS),
        src_o               => endpoint_src_out(c_NUM_PORTS),
        rtu_dst_port_mask_o => rtu_rsp(c_NUM_PORTS).port_mask(c_NUM_PORTS downto 0),
        rtu_prio_o          => rtu_rsp(c_NUM_PORTS).prio,
        rtu_drop_o          => rtu_rsp(c_NUM_PORTS).drop,
        rtu_rsp_valid_o     => rtu_rsp(c_NUM_PORTS).valid,
        rtu_rsp_ack_i       => rtu_rsp_ack(c_NUM_PORTS),
        wb_i                => cnx_master_out(c_SLAVE_NIC),
        wb_o                => cnx_master_in(c_SLAVE_NIC));
  
    rtu_rsp(c_NUM_PORTS).hp <= '0';
    fc_rx_pause(c_NUM_PORTS)       <= c_zero_pause; -- no pause for NIC  
    
    U_Endpoint_Fanout : xwb_sdb_crossbar
      generic map (
        g_num_masters => 1,
        g_num_slaves  => c_MAX_PORTS,
        g_registered  => true,
        g_wraparound  => true,
        g_layout      => c_epbar_layout,
        g_sdb_addr    => c_epbar_sdb_address)
      port map (
        clk_sys_i  => clk_sys,
        rst_n_i    => rst_n_sys,
        slave_i(0) => cnx_master_out(c_SLAVE_ENDPOINTS),
        slave_o(0) => cnx_master_in(c_SLAVE_ENDPOINTS),
        master_i   => cnx_endpoint_in,
        master_o   => cnx_endpoint_out);


    gen_endpoints_and_phys : for i in 0 to c_NUM_PORTS-1 generate
      U_Endpoint_X : xwr_endpoint
        generic map (
          g_interface_mode      => PIPELINED,
          g_address_granularity => BYTE,
          g_simulation          => g_simulation,
          g_tx_force_gap_length => 0,
          g_tx_runt_padding     => false,
          g_pcs_16bit           => true,
          g_rx_buffer_size      => 1024,
          g_with_rx_buffer      => true,
          g_with_flow_control   => false,-- useless: flow control commented out 
          g_with_timestamper    => true,
          g_with_dpi_classifier => true,
          g_with_vlans          => true,
          g_with_rtu            => true,
          g_with_leds           => true,
          g_with_dmtd           => false,
          g_with_packet_injection => f_logic2bool(g_inj_per_EP(i)),
          g_use_new_rxcrc       => true,
          g_use_new_txcrc       => false,
          g_with_stop_traffic   => g_with_watchdog)
        port map (
          clk_ref_i  => clk_ref_i,
          clk_sys_i  => clk_sys,
          clk_dmtd_i => clk_dmtd_i,
          rst_n_i    => rst_n_periph,

          pps_csync_p1_i => pps_csync,
          pps_valid_i    => pps_valid,

          phy_rst_o          => phys_o(i).rst,
          phy_loopen_o       => phys_o(i).loopen,
          phy_enable_o       => phys_o(i).enable,
          phy_rdy_i          => phys_i(i).rdy,
          phy_ref_clk_i      => phys_i(i).ref_clk,
          phy_tx_data_o      => ep_dbg_data_array(i), -- phys_o(i).tx_data, --
          phy_tx_k_o         => ep_dbg_k_array(i),    -- phys_o(i).tx_k,    -- 
          phy_tx_disparity_i => phys_i(i).tx_disparity,
          phy_tx_enc_err_i   => phys_i(i).tx_enc_err,
          phy_rx_data_i      => phys_i(i).rx_data,
          phy_rx_clk_i       => phys_i(i).rx_clk,
          phy_rx_k_i         => phys_i(i).rx_k,
          phy_rx_enc_err_i   => phys_i(i).rx_enc_err,
          phy_rx_bitslide_i  => phys_i(i).rx_bitslide,

          txtsu_port_id_o      => txtsu_timestamps(i).port_id(4 downto 0),
          txtsu_frame_id_o     => txtsu_timestamps(i).frame_id,
          txtsu_ts_value_o     => txtsu_timestamps(i).tsval,
          txtsu_ts_incorrect_o => txtsu_timestamps(i).incorrect,
          txtsu_stb_o          => txtsu_timestamps(i).stb,
          txtsu_ack_i          => txtsu_timestamps_ack(i),

          rtu_full_i         => rtu_full(i),
          rtu_rq_strobe_p1_o => rtu_req(i).valid,
          rtu_rq_abort_o     => rtu_rq_abort(i),
          rtu_rq_smac_o      => rtu_req(i).smac,
          rtu_rq_dmac_o      => rtu_req(i).dmac,
          rtu_rq_prio_o      => rtu_req(i).prio,
          rtu_rq_vid_o       => rtu_req(i).vid,
          rtu_rq_has_vid_o   => rtu_req(i).has_vid,
          rtu_rq_has_prio_o  => rtu_req(i).has_prio,

          src_o      => endpoint_src_out(i),
          src_i      => endpoint_src_in(i),
          snk_o      => endpoint_snk_out(i),
          snk_i      => endpoint_snk_in(i),
          wb_i       => cnx_endpoint_out(i),
          wb_o       => cnx_endpoint_in(i),

          ----- TRU stuff ------------
          pfilter_pclass_o     => ep2tru(i).pfilter_pclass,
          pfilter_drop_o       => ep2tru(i).pfilter_drop,
          pfilter_done_o       => ep2tru(i).pfilter_done,
          fc_tx_pause_req_i    => tru2ep(i).fc_pause_req,   -- we don't use it, use inject instead
          fc_tx_pause_delay_i  => tru2ep(i).fc_pause_delay, -- we don't use it, use inject instead
          fc_tx_pause_ready_o  => ep2tru(i).fc_pause_ready, -- we don't use it, use inject instead
          inject_req_i         => tru2ep(i).inject_req,
          inject_ready_o       => ep2tru(i).inject_ready,
          inject_packet_sel_i  => tru2ep(i).inject_packet_sel,
          inject_user_value_i  => tru2ep(i).inject_user_value,
          link_kill_i          => tru2ep(i).link_kill, --'0' , --link_kill(i), -- to change
          link_up_o            => ep2tru(i).status,
          ------ PAUSE to SWcore  ------------
          fc_rx_pause_start_p_o   => fc_rx_pause(i).req,  
          fc_rx_pause_quanta_o    => fc_rx_pause(i).quanta,    
          fc_rx_pause_prio_mask_o => fc_rx_pause(i).classes, 
          ----------------------------

          rmon_events_o => ep_events((i+1)*c_epevents_sz-1 downto i*c_epevents_sz),

          led_link_o => led_link_o(i),
          led_act_o  => led_act_o(i),
          stop_traffic_i => ep_stop_traffic,
          nice_dbg_o => ep_dbg(i));

          phys_o(i).tx_data <= ep_dbg_data_array(i);
          phys_o(i).tx_k    <= ep_dbg_k_array(i);

      txtsu_timestamps(i).port_id(5) <= '0';
      
      ------- TEMP ---------
--       link_kill(i)                <= not tru2ep(i).ctrlWr; 
--       tru2ep(i).fc_pause_req      <= '0';
--       tru2ep(i).fc_pause_delay    <= (others =>'0');
--       tru2ep(i).inject_req        <= '0';
--       tru2ep(i).inject_packet_sel <= (others => '0');
--       tru2ep(i).inject_user_value <= (others => '0');
--       ep2tru(i).rx_pck            <= '0';
--       ep2tru(i).rx_pck_class      <= (others => '0');
      ---------------------------

      clk_rx_vec(i) <= phys_i(i).rx_clk;

      EP_DBG_SYNC_AFULL: gc_sync_ffs
        port map(
          clk_i    => clk_sys,
          rst_n_i  => rst_n_periph,
          data_i   => ep_dbg(i).rxpath.pcs_fifo_afull,
          synced_o => ep_dbg_synced(i).rxpath.pcs_fifo_afull);

      EP_DBG_SYNC_EMPTY: gc_sync_ffs
        port map(
          clk_i    => clk_sys,
          rst_n_i  => rst_n_periph,
          data_i   => ep_dbg(i).rxpath.pcs_fifo_empty,
          synced_o => ep_dbg_synced(i).rxpath.pcs_fifo_empty);

      EP_DBG_SYNC_FULL: gc_sync_ffs
        port map(
          clk_i   => clk_sys,
          rst_n_i => rst_n_periph,
          data_i   => ep_dbg(i).rxpath.pcs_fifo_full,
          synced_o => ep_dbg_synced(i).rxpath.pcs_fifo_full);
    end generate gen_endpoints_and_phys;

    GEN_TIMING: for I in 0 to c_NUM_PORTS generate
      -- improve timing
      U_WRF_RXREG_X: xwrf_reg
        port map (
          rst_n_i => rst_n_periph,
          clk_i   => clk_sys,
          snk_i   => endpoint_src_out(i),
          snk_o   => endpoint_src_in(i),
          src_o   => wrfreg_src_out(i),
          src_i   => wrfreg_src_in(i));

      U_WRF_TXREG_X: xwrf_reg
        port map(
          rst_n_i => rst_n_periph,
          clk_i   => clk_sys,
          snk_i   => swc_src_out(i),
          snk_o   => swc_src_in(i),
          src_o   => endpoint_snk_in(i),
          src_i   => endpoint_snk_out(i));
    end generate;

    gen_terminate_unused_eps : for i in c_NUM_PORTS to c_MAX_PORTS-1 generate
      cnx_endpoint_in(i).ack   <= '1';
      cnx_endpoint_in(i).stall <= '0';
      cnx_endpoint_in(i).dat   <= x"deadbeef";
      cnx_endpoint_in(i).err   <= '0';
      cnx_endpoint_in(i).rty   <= '0';
      --txtsu_timestamps(i).valid <= '0';
    end generate gen_terminate_unused_eps;

--     gen_txtsu_debug : for i in 0 to c_NUM_PORTS-1 generate
--       TRIG0(i) <= txtsu_timestamps(i).stb;
--       trig1(i) <= txtsu_timestamps_ack(i);
--       trig2(0) <= vic_irqs(0);
--       trig2(1) <= vic_irqs(1);
--       trig2(2) <= vic_irqs(2);
--     end generate gen_txtsu_debug;

    U_Swcore : xswc_core
      generic map (
        g_prio_num                        => 8,
        g_output_queue_num                => 8,
        g_max_pck_size                    => 10 * 1024,
        g_max_oob_size                    => 3,
        g_num_ports                       => g_num_ports+1,
        g_pck_pg_free_fifo_size           => 512,
        g_input_block_cannot_accept_data  => "drop_pck",
        g_output_block_per_queue_fifo_size=> 64,
        g_wb_data_width                   => 16,
        g_wb_addr_width                   => 2,
        g_wb_sel_width                    => 2,
        g_wb_ob_ignore_ack                => false,
        g_mpm_mem_size                    => 65536, --test: 61440,--old: 65536,
        g_mpm_page_size                   => 64,    --test:    60,--old: 64,
        g_mpm_ratio                       => 8,     --test:    10,--old: 8,  --f_swc_ratio,  --2
        g_mpm_fifo_size                   => 8,
        g_mpm_fetch_next_pg_in_advance    => false,
        g_drop_outqueue_head_on_full      => true,
        g_num_global_pause                => c_NUM_GL_PAUSE,
        g_num_dbg_vector_width            => c_DBG_V_SWCORE)
      port map (
        clk_i          => clk_sys,
        clk_mpm_core_i => clk_aux_i,
        rst_n_i        => rst_n_swc,

        src_i => swc_src_in,
        src_o => swc_src_out,
        snk_i => swc_snk_in,
        snk_o => swc_snk_out,

        shaper_drop_at_hp_ena_i   => shaper_drop_at_hp_ena,
        
        -- pause stuff
        global_pause_i            => global_pause,
        perport_pause_i           => fc_rx_pause,

        dbg_o     => dbg_n_regs(32+c_DBG_V_SWCORE-1 downto 32), 
        nice_dbg_o => swc_dbg,
        
        rtu_rsp_i       => rtu_rsp,
        rtu_ack_o       => swc_rtu_ack,
        rtu_abort_o     =>rtu_rsp_abort,-- open --rtu_rsp_abort
        wdog_o    => swc_wdog_out,
        nomem_o   => swc_nomem
        );
     
    -- SWcore global pause nr=0 assigned to TRU
    global_pause(0)          <= swc2tru_req;

    -- SWcore global pause nr=1 assigned to TATSU
    global_pause(1)          <= shaper_request; 
    
    -- NIC sink
    --TRIG0 <= f_fabric_2_slv(endpoint_snk_in(1), endpoint_snk_out(1));
    ---- NIC source
    --TRIG1 <= f_fabric_2_slv(endpoint_src_out(1), endpoint_src_in(1));
    ---- NIC sink
    --TRIG2 <= f_fabric_2_slv(endpoint_snk_in(2), endpoint_snk_out(2));
    ---- NIC source
    --TRIG3 <= f_fabric_2_slv(endpoint_src_out(2), endpoint_src_in(2));
    --TRIG3(31) <= rst_n_periph;

    --TRIG2 <= rtu_rsp(c_NUM_PORTS).port_mask(31 downto 0);
    --TRIG3(0) <= rtu_rsp(c_NUM_PORTS).valid;
    --TRIG3(1) <= rtu_rsp_ack(c_NUM_PORTS);

   U_RTU : xwrsw_rtu_new
--   U_RTU : xwrsw_rtu
      generic map (
        g_prio_num                        => 8,
        g_interface_mode                  => PIPELINED,
        g_address_granularity             => BYTE,
        g_num_ports                       => g_num_ports,
        g_cpu_port_num                    => g_num_ports, -- g_num_ports-nt port is connected to CPU
        g_port_mask_bits                  => g_num_ports+1,
        g_handle_only_single_req_per_port => true,
        g_rmon_events_bits_pp             => c_RTU_EVENTS)
      port map (
        clk_sys_i  => clk_sys,
        rst_n_i    => rst_n_sys,--rst_n_periph,
        req_i      => rtu_req(g_num_ports-1 downto 0),
        req_full_o => rtu_full(g_num_ports-1 downto 0),
        rsp_o      => rtu_rsp(g_num_ports-1 downto 0),
        rsp_ack_i  => rtu_rsp_ack(g_num_ports-1 downto 0),
        rsp_abort_i=> rtu_rsp_abort(g_num_ports-1 downto 0), -- this is request from response receiving node
        rq_abort_i => rtu_rq_abort(g_num_ports-1 downto 0), -- this is request from requesting module
        ------ new TRU stuff ----------
        tru_req_o  => tru_req,
        tru_resp_i => tru_resp,
        rtu2tru_o  => rtu2tru,
        tru_enabled_i => tru_enabled,
        -------------------------------
        rmon_events_o => rtu_events,
        wb_i       => cnx_master_out(c_SLAVE_RTU),
        wb_o       => cnx_master_in(c_SLAVE_RTU),
        nice_dbg_o  => open);

    gen_TRU : if(g_with_TRU = true) generate
      U_TRU: xwrsw_tru
        generic map(     
          g_num_ports           => g_num_ports,
          g_tru_subentry_num    => 8,
          g_patternID_width     => 4,
          g_pattern_width       => g_num_ports,
          g_stableUP_treshold   => 100,
          g_pclass_number       => 8,
          g_mt_trans_max_fr_cnt => 1000,
          g_prio_width          => 3,
          g_pattern_mode_width  => 4,
          g_tru_entry_num       => 256,
          g_interface_mode      => PIPELINED,
          g_address_granularity => BYTE 
         )
        port map(
          clk_i               => clk_sys,
          rst_n_i             => rst_n_periph,
          req_i               => tru_req,
          resp_o              => tru_resp,
          rtu_i               => rtu2tru, 
          ep_i                => ep2tru,
          ep_o                => tru2ep,
          swc_block_oq_req_o  => swc2tru_req,
          enabled_o           => tru_enabled,
          wb_i                => cnx_master_out(c_SLAVE_TRU),
          wb_o                => cnx_master_in(c_SLAVE_TRU));

    end generate gen_TRU;    

    gen_no_TRU : if(g_with_TRU = false) generate
      swc2tru_req                    <= c_zero_gl_pause;
      tru2ep                         <= (others => c_tru2ep_zero);
      tru_resp                       <= c_tru_response_zero;
      tru_enabled                    <= '0';
      cnx_master_in(c_SLAVE_TRU).ack <= '1';
    end generate gen_no_TRU; 

    gen_TATSU: if(g_with_TATSU = true) generate
      U_TATSU:  xwrsw_tatsu
        generic map(     
          g_num_ports             => g_num_ports,
          g_simulation            => g_simulation,
          g_interface_mode        => PIPELINED,
          g_address_granularity   => BYTE
          )
        port map(
          clk_sys_i               => clk_sys,
          clk_ref_i               => clk_ref_i,
          rst_n_i                 => rst_n_sys,

          shaper_request_o        => shaper_request,
          shaper_drop_at_hp_ena_o => shaper_drop_at_hp_ena,
          tm_utc_i                => tm_utc,
          tm_cycles_i             => tm_cycles,
          tm_time_valid_i         => tm_time_valid,
          wb_i                    => cnx_master_out(c_SLAVE_TATSU),
          wb_o                    => cnx_master_in(c_SLAVE_TATSU)
        );

    end generate gen_TATSU;
    
    gen_no_TATSU: if(g_with_TATSU = false) generate
      shaper_request                            <= c_zero_gl_pause;
      shaper_drop_at_hp_ena                     <= '0';
      cnx_master_in(c_SLAVE_TATSU).ack          <= '1';
    end generate gen_no_TATSU;

  end generate gen_network_stuff;

  gen_no_network_stuff : if(g_without_network = true) generate
    gen_dummy_resets : for i in 0 to g_num_ports-1 generate
      phys_o(i).rst    <= not rst_n_periph;
      phys_o(i).loopen <= '0';
    end generate gen_dummy_resets;
  end generate gen_no_network_stuff;

  U_Tx_TSU : xwrsw_tx_tsu
    generic map (
      g_num_ports           => c_NUM_PORTS,
      g_interface_mode      => PIPELINED,
      g_address_granularity => BYTE)
    port map (
      clk_sys_i        => clk_sys,
      rst_n_i          => rst_n_periph,
      timestamps_i     => txtsu_timestamps,
      timestamps_ack_o => txtsu_timestamps_ack,
      wb_i             => cnx_master_out(c_SLAVE_TXTSU),
      wb_o             => cnx_master_in(c_SLAVE_TXTSU));


  --TRIG2(15 downto 0) <= txtsu_timestamps(0).frame_id;
  --TRIG2(21 downto 16) <= txtsu_timestamps(0).port_id;
  --TRIG2(22) <= txtsu_timestamps(0).valid;
  
  U_GPIO : xwb_gpio_port
    generic map (
      g_interface_mode         => PIPELINED,
      g_address_granularity    => BYTE,
      g_num_pins               => c_NUM_GPIO_PINS,
      g_with_builtin_tristates => false)
    port map (
      clk_sys_i  => clk_sys,
      rst_n_i    => rst_n_periph,
      slave_i    => cnx_master_out(c_SLAVE_GPIO),
      slave_o    => cnx_master_in(c_SLAVE_GPIO),
      gpio_b     => dummy,
      gpio_out_o => gpio_out,
      gpio_in_i  => gpio_in);

  uart_sel_o <= gpio_out(0);


  gpio_o(0) <= gpio_out(0);
  gpio_in(0) <= gpio_i(0);

  U_MiniBackplane_I2C : xwb_i2c_master
    generic map (
      g_interface_mode      => PIPELINED,
      g_address_granularity => BYTE,
      g_num_interfaces      => 3)
    port map (
      clk_sys_i    => clk_sys,
      rst_n_i      => rst_n_periph,
      slave_i      => cnx_master_out(c_SLAVE_I2C),
      slave_o      => cnx_master_in(c_SLAVE_I2C),
      desc_o       => open,
      scl_pad_i    => i2c_scl_i,
      scl_pad_o    => i2c_scl_o,
      scl_padoen_o => i2c_scl_oen_o,
      sda_pad_i    => i2c_sda_i,
      sda_pad_o    => i2c_sda_o,
      sda_padoen_o => i2c_sda_oen_o);

  --=====================================--
  --               PSTATS                --
  --=====================================--
  gen_PSTATS: if(g_with_PSTATS = true) generate
    U_PSTATS : xwrsw_pstats
      generic map(
        g_interface_mode      => PIPELINED,
        g_address_granularity => BYTE,
        g_nports => c_NUM_PORTS,
        g_cnt_pp => c_ALL_EVENTS,
        g_cnt_pw => 4)
      port map(
        rst_n_i => rst_n_periph,
        clk_i   => clk_sys,
  
        events_i => rmon_events,
  
        wb_i  => cnx_master_out(c_SLAVE_PSTATS),
        wb_o  => cnx_master_in(c_SLAVE_PSTATS));
 
  end generate;

  gen_no_PSTATS: if(g_with_PSTATS = false) generate
    cnx_master_in(c_SLAVE_PSTATS).ack <= '1';
    cnx_master_in(c_SLAVE_PSTATS).int <= '0';
  end generate;

  gen_events_assemble : for i in 0 to c_NUM_PORTS-1 generate
    rmon_events((i+1)*c_ALL_EVENTS-1 downto i*c_ALL_EVENTS) <= 
                std_logic(tru_resp.respMask(i) and tru_resp.valid)     &
                rtu_events((i+1)*c_RTU_EVENTS-1 downto i*c_RTU_EVENTS) &
                ep_events ((i+1)*c_epevents_sz-1 downto i*c_epevents_sz);
  end generate gen_events_assemble;

  --============================================================--
  -- Hardware Info Unit providing firmware version for software --
  --============================================================--

  gen_HWIU: if(g_with_HWIU = true) generate
    U_HWIU: xwrsw_hwiu
      generic map(
        g_interface_mode => PIPELINED,
        g_address_granularity => BYTE,
        g_ndbg_regs => c_DBG_N_REGS,
        g_ver_major => 4,
        g_ver_minor => 0,
        g_build     => 1)
      port map(
        rst_n_i       => rst_n_periph,
        clk_i         => clk_sys,
        dbg_regs_i    => dbg_n_regs,
        dbg_chps_id_o => dbg_chps_id,
        dbg1_o        => hwiu_dbg1,
        dbg2_o        => hwiu_dbg2,
        wb_i          => cnx_master_out(c_SLAVE_HWIU),
        wb_o          => cnx_master_in(c_SLAVE_HWIU));
  end generate;

  gen_no_HWIU: if(g_with_HWIU = false) generate
    cnx_master_in(c_SLAVE_HWIU).ack   <= '1';
    cnx_master_in(c_SLAVE_HWIU).dat   <= c_GW_VERSION;
    cnx_master_in(c_SLAVE_HWIU).err   <= '0';
    cnx_master_in(c_SLAVE_HWIU).stall <= '0';
    cnx_master_in(c_SLAVE_HWIU).rty   <= '0';
    dbg_chps_id                       <= (others =>'0');
  end generate;

  -----------------------------------------------------------------------------
  -- PWM Controlle for mini-backplane fan drive
  -----------------------------------------------------------------------------
  
  U_PWM_Controller : xwb_simple_pwm
    generic map (
      g_num_channels        => 2,
      g_regs_size           => 8,
      g_default_period      => 255,
      g_default_presc       => 30,
      g_default_val         => 255,
      g_interface_mode      => PIPELINED,
      g_address_granularity => BYTE)
    port map (
      clk_sys_i => clk_sys,
      rst_n_i   => rst_n_periph,
      slave_i   => cnx_master_out(c_SLAVE_PWM),
      slave_o   => cnx_master_in(c_SLAVE_PWM),
      pwm_o(0)  => mb_fan1_pwm_o,
      pwm_o(1)  => mb_fan2_pwm_o);

  -----------------------------------------------------------------------------
  -- Interrupt assignment
  -----------------------------------------------------------------------------
  
  vic_irqs(0)           <= cnx_master_in(c_SLAVE_NIC).int;
  vic_irqs(1)           <= cnx_master_in(c_SLAVE_TXTSU).int;
  vic_irqs(2)           <= cnx_master_in(c_SLAVE_RTU).int;
  vic_irqs(3)           <= cnx_master_in(c_SLAVE_PSTATS).int;

-------------------------------------------------------------------------------
-- Various constant-driven I/Os
-------------------------------------------------------------------------------

  clk_en_o          <= '0';
  clk_sel_o         <= '0';
  clk_dmtd_divsel_o <= '1';             -- choose 62.5 MHz DDMTD clock
  clk_sys_o         <= clk_sys;

-------------------------------------------------------------------------------
-- Reset of the SW-Core in case something goes horribly wrong
-------------------------------------------------------------------------------
  GEN_SWC_RST: if(g_with_watchdog) generate
    WDOG: xwrsw_watchdog
      generic map(
        g_num_ports => g_num_ports+1)
      port map(
        rst_n_i => rst_n_periph,
        clk_i   => clk_sys,
        swc_nomem_i => swc_nomem,
        swc_fsms_i  => swc_wdog_out,

        swcrst_n_o  => rst_n_swc,
        epstop_o   => ep_stop_traffic,

        rtu_ack_i  => swc_rtu_ack,
        rtu_ack_o  => rtu_rsp_ack,

        snk_i => wrfreg_src_out,
        snk_o => wrfreg_src_in,
        src_o => swc_snk_in,
        src_i => swc_snk_out,
        wb_i  => cnx_master_out(c_SLAVE_WDOG),
        wb_o  => cnx_master_in(c_SLAVE_WDOG));
  end generate;

  GEN_NO_SWC_RST: if(not g_with_watchdog) generate
    ep_stop_traffic <= '0';
    rst_n_swc     <= rst_n_periph;
    rtu_rsp_ack   <= swc_rtu_ack;
    wrfreg_src_in <= swc_snk_out;
    swc_snk_in    <= wrfreg_src_out;
  end generate;

  gen_muxed_CS: if g_with_muxed_CS = true generate
    CS_ICON : chipscope_icon
     port map (
      CONTROL0 => CONTROL0);
    CS_ILA : chipscope_ila
     port map (
       CONTROL => CONTROL0,
       CLK     => clk_sys, --phys_i(0).rx_clk,
       TRIG0   => T0,
       TRIG1   => T1,
       TRIG2   => T2,
       TRIG3   => T3,
       TRIG4   => T4,
       TRIG5   => T5,
       TRIG6   => T6,
       TRIG7   => T7,
       TRIG8   => T8,
       TRIG9   => T9,
       TRIG10  => T10,
       TRIG11  => T11);
  
       T0   <= TRIG0(to_integer(unsigned(dbg_chps_id)));
       T1   <= TRIG1(to_integer(unsigned(dbg_chps_id)));
       T2   <= TRIG2(to_integer(unsigned(dbg_chps_id)));
       T3   <= TRIG3(to_integer(unsigned(dbg_chps_id)));
       T4   <= TRIG4(to_integer(unsigned(dbg_chps_id)));
       T5   <= TRIG5(to_integer(unsigned(dbg_chps_id)));
       T6   <= TRIG6(to_integer(unsigned(dbg_chps_id)));
       T7   <= TRIG7(to_integer(unsigned(dbg_chps_id)));
       T8   <= TRIG8(to_integer(unsigned(dbg_chps_id)));
       T9   <= TRIG9(to_integer(unsigned(dbg_chps_id)));
       T10  <= TRIG10(to_integer(unsigned(dbg_chps_id)));
       T11  <= TRIG11(to_integer(unsigned(dbg_chps_id)));
  end generate;

  process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if (rst_n_periph='0') then
        hwiu_val1 <= x"000001F4"; --500
        hwiu_val2 <= x"00000000";
      else
        if(hwiu_dbg1 /= x"00000000") then
          hwiu_val1 <= unsigned(hwiu_dbg1);
        end if;

        if(hwiu_dbg2 /= x"00000000") then
          hwiu_val2 <= unsigned(hwiu_dbg2);
        end if;
      end if;
    end if;
  end process;

  GEN_RTU_DBG: for I in 0 to g_num_ports-1 generate
    process(clk_sys)
    begin
      if rising_edge(clk_sys) then
        if(rst_n_periph='0') then
          dbg_rtu_cnt(I) <= (others=>'0');
          dbg_rtu_bug(I) <= (others=>'0');
          dbg_cnt_eq(I) <= (others=>'0');
          dbg_cnt_dif(I) <= (others=>'0');
        else
          if( (swc_dbg.ib(I).dbg_bare_sof > swc_dbg.ib(I).sof_cnt  and
                unsigned(swc_dbg.ib(I).dbg_bare_sof) - unsigned(swc_dbg.ib(I).sof_cnt) > hwiu_val2) or
              (swc_dbg.ib(I).dbg_bare_sof < swc_dbg.ib(I).sof_cnt  and
                unsigned(swc_dbg.ib(I).sof_cnt) - unsigned(swc_dbg.ib(I).dbg_bare_sof) > hwiu_val2)
            ) then
            dbg_cnt_dif(I) <= dbg_cnt_dif(I) + 1;
            dbg_cnt_eq(I)  <= (others=>'0');
          elsif(swc_dbg.ib(I).dbg_bare_sof = swc_dbg.ib(I).sof_cnt) then
            dbg_cnt_eq(I) <= dbg_cnt_eq(I) + 1;
            dbg_cnt_dif(I) <= (others=>'0');
          end if;

          if(dbg_cnt_dif(I) > hwiu_val1 and unsigned(swc_dbg.ib(I).dbg_bare_sof)/=dbg_rtu_cnt(I)) then
            dbg_rtu_bug(I) <= dbg_rtu_bug(I) + 1;
            dbg_rtu_cnt(I) <= unsigned(swc_dbg.ib(I).dbg_bare_sof);
          elsif(dbg_cnt_eq(I) > hwiu_val1) then
            dbg_rtu_bug(I) <= (others=>'0');
            dbg_rtu_cnt(I) <= unsigned(swc_dbg.ib(I).dbg_bare_sof);
          end if;
        end if;
      end if;
    end process;
  end generate;


  -------------------------------------------------------------------
  -------------------------------------------------------------------
  -- bank 0

  TRIG0(0)(0)            <= swc_dbg.mmu.palloc.res_almost_full;
  TRIG0(0)(1)            <= swc_dbg.mmu.palloc.res_full;
  TRIG0(0)(7 downto 2)   <= swc_dbg.mmu.palloc.free_pages(5 downto 0);
  TRIG0(0)(8)            <= swc_dbg.mmu.palloc.q_write;
  TRIG0(0)(9)            <= swc_dbg.mmu.palloc.q_read;
  TRIG0(0)(13 downto 10) <= swc_dbg.ib(0).trans_fsm;
  TRIG0(0)(16 downto 14) <= swc_dbg.ib(0).alloc_fsm;
  TRIG0(0)(20 downto 17) <= swc_dbg.ib(0).rcv_fsm;
  TRIG0(0)(24 downto 21) <= swc_dbg.ib(0).ll_fsm;
  TRIG0(0)(25)           <= swc_dbg.ib(0).rtu_valid;
  TRIG0(0)(26)           <= swc_dbg.ib(0).mem_full_dump;
  TRIG0(0)(27)           <= swc_dbg.ib(0).finish_rcv;
  TRIG0(0)(31 downto 28) <= swc_dbg.ib(1).trans_fsm;

  TRIG1(0)(2 downto 0)   <= swc_dbg.ib(1).alloc_fsm;
  TRIG1(0)(6 downto 3)   <= swc_dbg.ib(1).rcv_fsm;
  TRIG1(0)(10 downto 7)  <= swc_dbg.ib(1).ll_fsm;
  TRIG1(0)(11)           <= swc_dbg.ib(1).rtu_valid;
  TRIG1(0)(12)           <= swc_dbg.ib(1).mem_full_dump;
  TRIG1(0)(13)           <= swc_dbg.ib(1).finish_rcv;
  TRIG1(0)(17 downto 14) <= swc_dbg.ib(6).trans_fsm;
  TRIG1(0)(20 downto 18) <= swc_dbg.ib(6).alloc_fsm;
  TRIG1(0)(24 downto 21) <= swc_dbg.ib(6).rcv_fsm;
  TRIG1(0)(28 downto 25) <= swc_dbg.ib(6).ll_fsm;
  TRIG1(0)(29)           <= swc_dbg.ib(6).rtu_valid;
  TRIG1(0)(30)           <= swc_dbg.ib(6).mem_full_dump;
  TRIG1(0)(31)           <= swc_dbg.ib(6).finish_rcv;

  TRIG2(0)(3 downto 0)   <= swc_dbg.ib(7).trans_fsm;
  TRIG2(0)(6 downto 4)   <= swc_dbg.ib(7).alloc_fsm;
  TRIG2(0)(10 downto 7)  <= swc_dbg.ib(7).rcv_fsm;
  TRIG2(0)(14 downto 11) <= swc_dbg.ib(7).ll_fsm;
  TRIG2(0)(15)           <= swc_dbg.ib(7).rtu_valid;
  TRIG2(0)(16)           <= swc_dbg.ib(7).mem_full_dump;
  TRIG2(0)(17)           <= swc_dbg.ib(7).finish_rcv;
  TRIG2(0)(18)           <= swc_dbg.ib(0).force_free;
  TRIG2(0)(19)           <= swc_dbg.ib(0).force_free_done;
  TRIG2(0)(20)           <= swc_dbg.ib(1).force_free;
  TRIG2(0)(21)           <= swc_dbg.ib(1).force_free_done;
  TRIG2(0)(22)           <= swc_dbg.ib(6).force_free;
  TRIG2(0)(23)           <= swc_dbg.ib(6).force_free_done;
  TRIG2(0)(24)           <= swc_dbg.ib(7).force_free;
  TRIG2(0)(25)           <= swc_dbg.ib(7).force_free_done;
  TRIG2(0)(31 downto 26) <= swc_dbg.ib(0).force_free_adr(5 downto 0);

  TRIG3(0)(0)            <= swc_dbg.ib(0).pckstart_in_adv;
  TRIG3(0)(1)            <= swc_dbg.ib(0).pckinter_in_adv;
  TRIG3(0)(2)            <= swc_dbg.ib(0).ll_wr;
  TRIG3(0)(3)            <= swc_dbg.ib(0).ll_wr_done;
  TRIG3(0)(4)            <= swc_dbg.ib(0).ll_page_valid;
  TRIG3(0)(5)            <= swc_dbg.ib(0).ll_eof;
  TRIG3(0)(11 downto  6) <= swc_dbg.ib(0).ll_page(5 downto 0);
  TRIG3(0)(17 downto 12) <= swc_dbg.ib(0).ll_next_page(5 downto 0);
  TRIG3(0)(18)           <= swc_dbg.ib(0).alloc_req;
  TRIG3(0)(19)           <= swc_dbg.ib(0).alloc_done;
  TRIG3(0)(25 downto 20) <= swc_dbg.ib(0).page_start(5 downto 0);
  TRIG3(0)(31 downto 26) <= swc_dbg.ib(0).page_inter(5 downto 0);

  TRIG4(0)(0)            <= swc_dbg.ib(1).pckstart_in_adv;
  TRIG4(0)(1)            <= swc_dbg.ib(1).pckinter_in_adv;
  TRIG4(0)(2)            <= swc_dbg.ib(1).ll_wr;
  TRIG4(0)(3)            <= swc_dbg.ib(1).ll_wr_done;
  TRIG4(0)(4)            <= swc_dbg.ib(1).ll_page_valid;
  TRIG4(0)(5)            <= swc_dbg.ib(1).ll_eof;
  TRIG4(0)(11 downto  6) <= swc_dbg.ib(1).ll_page(5 downto 0);
  TRIG4(0)(17 downto 12) <= swc_dbg.ib(1).ll_next_page(5 downto 0);
  TRIG4(0)(18)           <= swc_dbg.ib(1).alloc_req;
  TRIG4(0)(19)           <= swc_dbg.ib(1).alloc_done;
  TRIG4(0)(25 downto 20) <= swc_dbg.ib(1).page_start(5 downto 0);
  TRIG4(0)(31 downto 26) <= swc_dbg.ib(1).page_inter(5 downto 0);

  TRIG5(0)(0)            <= swc_dbg.ib(6).pckstart_in_adv;
  TRIG5(0)(1)            <= swc_dbg.ib(6).pckinter_in_adv;
  TRIG5(0)(2)            <= swc_dbg.ib(6).ll_wr;
  TRIG5(0)(3)            <= swc_dbg.ib(6).ll_wr_done;
  TRIG5(0)(4)            <= swc_dbg.ib(6).ll_page_valid;
  TRIG5(0)(5)            <= swc_dbg.ib(6).ll_eof;
  TRIG5(0)(11 downto  6) <= swc_dbg.ib(6).ll_page(5 downto 0);
  TRIG5(0)(17 downto 12) <= swc_dbg.ib(6).ll_next_page(5 downto 0);
  TRIG5(0)(18)           <= swc_dbg.ib(6).alloc_req;
  TRIG5(0)(19)           <= swc_dbg.ib(6).alloc_done;
  TRIG5(0)(25 downto 20) <= swc_dbg.ib(6).page_start(5 downto 0);
  TRIG5(0)(31 downto 26) <= swc_dbg.ib(6).page_inter(5 downto 0);

  TRIG6(0)(0)            <= swc_dbg.ib(7).pckstart_in_adv;
  TRIG6(0)(1)            <= swc_dbg.ib(7).pckinter_in_adv;
  TRIG6(0)(2)            <= swc_dbg.ib(7).ll_wr;
  TRIG6(0)(3)            <= swc_dbg.ib(7).ll_wr_done;
  TRIG6(0)(4)            <= swc_dbg.ib(7).ll_page_valid;
  TRIG6(0)(5)            <= swc_dbg.ib(7).ll_eof;
  TRIG6(0)(11 downto  6) <= swc_dbg.ib(7).ll_page(5 downto 0);
  TRIG6(0)(17 downto 12) <= swc_dbg.ib(7).ll_next_page(5 downto 0);
  TRIG6(0)(18)           <= swc_dbg.ib(7).alloc_req;
  TRIG6(0)(19)           <= swc_dbg.ib(7).alloc_done;
  TRIG6(0)(25 downto 20) <= swc_dbg.ib(7).page_start(5 downto 0);
  TRIG6(0)(31 downto 26) <= swc_dbg.ib(7).page_inter(5 downto 0);

  TRIG7(0)(0) <= swc_dbg.ib(0).pages_same;
  TRIG7(0)(1) <= swc_dbg.ib(1).pages_same;
  TRIG7(0)(2) <= swc_dbg.ib(6).pages_same;
  TRIG7(0)(3) <= swc_dbg.ib(7).pages_same;
  TRIG7(0)(4) <= swc_dbg.ib(0).rcv_stuck;
  TRIG7(0)(5) <= swc_dbg.ib(1).rcv_stuck;
  TRIG7(0)(6) <= swc_dbg.ib(6).rcv_stuck;
  TRIG7(0)(7) <= swc_dbg.ib(7).rcv_stuck;
  TRIG7(0)(8)  <= swc_dbg.mmu.p0_req_alloc;
  TRIG7(0)(9)  <= swc_dbg.mmu.p1_req_alloc;
  TRIG7(0)(10) <= swc_dbg.mmu.p6_req_alloc;
  TRIG7(0)(11) <= swc_dbg.mmu.p7_req_alloc;
  TRIG7(0)(12) <= swc_dbg.mmu.p0_arb_req;
  TRIG7(0)(13) <= swc_dbg.mmu.p0_arb_grant;
  TRIG7(0)(14) <= swc_dbg.mmu.p1_arb_req;
  TRIG7(0)(15) <= swc_dbg.mmu.p1_arb_grant;
  TRIG7(0)(21 downto 16) <= swc_dbg.ib(0).alloc_page(5 downto 0); --all input blocks share
                                                                  -- the same output from MMU
  TRIG7(0)(27 downto 22) <= swc_dbg.mmu.palloc.rd_ptr(5 downto 0);
  TRIG7(0)(28) <= swc_dbg.mmu.palloc.out_nomem_d1;
  TRIG7(0)(29) <= swc_dbg.mmu.palloc.alloc_done;
  TRIG7(0)(30) <= swc_dbg.mmu.palloc.alloc_req_d0;
  TRIG7(0)(31) <= swc_dbg.mmu.palloc.pg_adv_valid;


  TRIG8(0)(5 downto 0)   <= swc_dbg.ib(1).force_free_adr(5 downto 0);
  TRIG8(0)(11 downto 6)  <= swc_dbg.ib(6).force_free_adr(5 downto 0);
  TRIG8(0)(17 downto 12) <= swc_dbg.ib(7).force_free_adr(5 downto 0);
  TRIG8(0)(18) <= swc_dbg.ob(0).free;
  TRIG8(0)(19) <= swc_dbg.ob(0).free_done;
  TRIG8(0)(20) <= swc_dbg.ob(1).free;
  TRIG8(0)(21) <= swc_dbg.ob(1).free_done;
  TRIG8(0)(22) <= swc_dbg.ob(6).free;
  TRIG8(0)(23) <= swc_dbg.ob(6).free_done;
  TRIG8(0)(24) <= swc_dbg.ob(7).free;
  TRIG8(0)(25) <= swc_dbg.ob(7).free_done;
  TRIG8(0)(31 downto 26) <= swc_dbg.ob(0).free_adr(5 downto 0);

  TRIG9(0)(5 downto 0)   <= swc_dbg.ob(1).free_adr(5 downto 0);
  TRIG9(0)(11 downto 6)  <= swc_dbg.ob(6).free_adr(5 downto 0);
  TRIG9(0)(17 downto 12) <= swc_dbg.ob(7).free_adr(5 downto 0);

  TRIG9(0)(23 downto 18) <= swc_dbg.mmu.palloc.in_pg(5 downto 0);
  TRIG9(0)(29 downto 24) <= swc_dbg.mmu.palloc.out_pg(5 downto 0);
  TRIG9(0)(30) <= swc_dbg.mmu.palloc.dbg_trig;

  TRIG10(0)(0) <= swc_dbg.mmu.p6_arb_req;
  TRIG10(0)(1) <= swc_dbg.mmu.p6_arb_grant;
  TRIG10(0)(2) <= swc_dbg.mmu.p7_arb_req;
  TRIG10(0)(3) <= swc_dbg.mmu.p7_arb_grant;
  TRIG10(0)(11 downto 4) <= swc_dbg.mmu.grant_ib_d0(7 downto 0);
  TRIG10(0)(19 downto 12) <= swc_dbg.mmu.palloc.grant_port_msk(7 downto 0);

  -------------------------------------------------------------------
  ---- bank 1
  TRIG0(1)(5 downto 0) <= swc_dbg.mmu.palloc.free_pages(5 downto 0);
  TRIG0(1)(11 downto 6)  <= swc_dbg.ib(0).page_start(5 downto 0);
  TRIG0(1)(17 downto 12) <= swc_dbg.ib(0).page_inter(5 downto 0);
  TRIG0(1)(23 downto 18) <= swc_dbg.ib(1).page_start(5 downto 0);
  TRIG0(1)(29 downto 24) <= swc_dbg.ib(1).page_inter(5 downto 0);
  TRIG1(1)(5 downto 0)   <= swc_dbg.ib(6).page_start(5 downto 0);
  TRIG1(1)(11 downto 6)  <= swc_dbg.ib(6).page_inter(5 downto 0);
  TRIG1(1)(17 downto 12) <= swc_dbg.ib(7).page_start(5 downto 0);
  TRIG1(1)(23 downto 18) <= swc_dbg.ib(7).page_inter(5 downto 0);
  TRIG1(1)(24)           <= swc_dbg.ib(0).rcv_stuck;
  TRIG1(1)(25)           <= swc_dbg.ib(1).rcv_stuck;
  TRIG1(1)(26)           <= swc_dbg.ib(6).rcv_stuck;
  TRIG1(1)(27)           <= swc_dbg.ib(7).rcv_stuck;
  TRIG1(1)(31 downto 28) <= (others=>'0');

  TRIG2(1)(0)            <= swc_dbg.free(0).free;
  TRIG2(1)(1)            <= swc_dbg.free(0).free_done;
  TRIG2(1)(2)            <= swc_dbg.free(0).ffree;
  TRIG2(1)(3)            <= swc_dbg.free(0).ffree_done;
  TRIG2(1)(9 downto 4)   <= swc_dbg.free(0).pgadr(5 downto 0);
  TRIG2(1)(10)           <= swc_dbg.free(0).last_ucnt;
  TRIG2(1)(13 downto 11) <= swc_dbg.free(0).fsm;
  TRIG2(1)(14)           <= swc_dbg.free(0).ib_ffree;
  TRIG2(1)(15)           <= swc_dbg.free(0).ib_ffree_done;
  TRIG2(1)(21 downto 16) <= swc_dbg.free(0).ib_pgadr(5 downto 0);
  TRIG2(1)(22)           <= swc_dbg.free(0).ob_free;
  TRIG2(1)(23)           <= swc_dbg.free(0).ob_free_done;
  TRIG2(1)(29 downto 24) <= swc_dbg.free(0).ob_pgadr(5 downto 0);
  TRIG2(1)(31 downto 30) <= (others=>'0');

  TRIG3(1)(0)            <= swc_dbg.free(1).free;
  TRIG3(1)(1)            <= swc_dbg.free(1).free_done;
  TRIG3(1)(2)            <= swc_dbg.free(1).ffree;
  TRIG3(1)(3)            <= swc_dbg.free(1).ffree_done;
  TRIG3(1)(9 downto 4)   <= swc_dbg.free(1).pgadr(5 downto 0);
  TRIG3(1)(10)           <= swc_dbg.free(1).last_ucnt;
  TRIG3(1)(13 downto 11) <= swc_dbg.free(1).fsm;
  TRIG3(1)(14)           <= swc_dbg.free(1).ib_ffree;
  TRIG3(1)(15)           <= swc_dbg.free(1).ib_ffree_done;
  TRIG3(1)(21 downto 16) <= swc_dbg.free(1).ib_pgadr(5 downto 0);
  TRIG3(1)(22)           <= swc_dbg.free(1).ob_free;
  TRIG3(1)(23)           <= swc_dbg.free(1).ob_free_done;
  TRIG3(1)(29 downto 24) <= swc_dbg.free(1).ob_pgadr(5 downto 0);
  TRIG3(1)(31 downto 30) <= (others=>'0');

  TRIG4(1)(0)            <= swc_dbg.free(6).free;
  TRIG4(1)(1)            <= swc_dbg.free(6).free_done;
  TRIG4(1)(2)            <= swc_dbg.free(6).ffree;
  TRIG4(1)(3)            <= swc_dbg.free(6).ffree_done;
  TRIG4(1)(9 downto 4)   <= swc_dbg.free(6).pgadr(5 downto 0);
  TRIG4(1)(10)           <= swc_dbg.free(6).last_ucnt;
  TRIG4(1)(13 downto 11) <= swc_dbg.free(6).fsm;
  TRIG4(1)(14)           <= swc_dbg.free(6).ib_ffree;
  TRIG4(1)(15)           <= swc_dbg.free(6).ib_ffree_done;
  TRIG4(1)(21 downto 16) <= swc_dbg.free(6).ib_pgadr(5 downto 0);
  TRIG4(1)(22)           <= swc_dbg.free(6).ob_free;
  TRIG4(1)(23)           <= swc_dbg.free(6).ob_free_done;
  TRIG4(1)(29 downto 24) <= swc_dbg.free(6).ob_pgadr(5 downto 0);
  TRIG4(1)(31 downto 30) <= (others=>'0');

  TRIG5(1)(0)            <= swc_dbg.free(7).free;
  TRIG5(1)(1)            <= swc_dbg.free(7).free_done;
  TRIG5(1)(2)            <= swc_dbg.free(7).ffree;
  TRIG5(1)(3)            <= swc_dbg.free(7).ffree_done;
  TRIG5(1)(9 downto 4)   <= swc_dbg.free(7).pgadr(5 downto 0);
  TRIG5(1)(10)           <= swc_dbg.free(7).last_ucnt;
  TRIG5(1)(13 downto 11) <= swc_dbg.free(7).fsm;
  TRIG5(1)(14)           <= swc_dbg.free(7).ib_ffree;
  TRIG5(1)(15)           <= swc_dbg.free(7).ib_ffree_done;
  TRIG5(1)(21 downto 16) <= swc_dbg.free(7).ib_pgadr(5 downto 0);
  TRIG5(1)(22)           <= swc_dbg.free(7).ob_free;
  TRIG5(1)(23)           <= swc_dbg.free(7).ob_free_done;
  TRIG5(1)(29 downto 24) <= swc_dbg.free(7).ob_pgadr(5 downto 0);
  TRIG5(1)(31 downto 30) <= (others=>'0');

  TRIG6(1)(0)            <= swc_dbg.free(2).free;
  TRIG6(1)(1)            <= swc_dbg.free(2).free_done;
  TRIG6(1)(2)            <= swc_dbg.free(2).ffree;
  TRIG6(1)(3)            <= swc_dbg.free(2).ffree_done;
  TRIG6(1)(9 downto 4)   <= swc_dbg.free(2).pgadr(5 downto 0);
  TRIG6(1)(10)           <= swc_dbg.free(2).last_ucnt;
  TRIG6(1)(13 downto 11) <= swc_dbg.free(2).fsm;
  TRIG6(1)(14)           <= swc_dbg.free(2).ib_ffree;
  TRIG6(1)(15)           <= swc_dbg.free(2).ib_ffree_done;
  TRIG6(1)(21 downto 16) <= swc_dbg.free(2).ib_pgadr(5 downto 0);
  TRIG6(1)(22)           <= swc_dbg.free(2).ob_free;
  TRIG6(1)(23)           <= swc_dbg.free(2).ob_free_done;
  TRIG6(1)(29 downto 24) <= swc_dbg.free(2).ob_pgadr(5 downto 0);
  TRIG6(1)(31 downto 30) <= (others=>'0');

  TRIG7(1)(0)            <= swc_dbg.free(3).free;
  TRIG7(1)(1)            <= swc_dbg.free(3).free_done;
  TRIG7(1)(2)            <= swc_dbg.free(3).ffree;
  TRIG7(1)(3)            <= swc_dbg.free(3).ffree_done;
  TRIG7(1)(9 downto 4)   <= swc_dbg.free(3).pgadr(5 downto 0);
  TRIG7(1)(10)           <= swc_dbg.free(3).last_ucnt;
  TRIG7(1)(13 downto 11) <= swc_dbg.free(3).fsm;
  TRIG7(1)(14)           <= swc_dbg.free(3).ib_ffree;
  TRIG7(1)(15)           <= swc_dbg.free(3).ib_ffree_done;
  TRIG7(1)(21 downto 16) <= swc_dbg.free(3).ib_pgadr(5 downto 0);
  TRIG7(1)(22)           <= swc_dbg.free(3).ob_free;
  TRIG7(1)(23)           <= swc_dbg.free(3).ob_free_done;
  TRIG7(1)(29 downto 24) <= swc_dbg.free(3).ob_pgadr(5 downto 0);
  TRIG7(1)(31 downto 30) <= (others=>'0');

  TRIG8(1)(0)            <= swc_dbg.free(4).free;
  TRIG8(1)(1)            <= swc_dbg.free(4).free_done;
  TRIG8(1)(2)            <= swc_dbg.free(4).ffree;
  TRIG8(1)(3)            <= swc_dbg.free(4).ffree_done;
  TRIG8(1)(9 downto 4)   <= swc_dbg.free(4).pgadr(5 downto 0);
  TRIG8(1)(10)           <= swc_dbg.free(4).last_ucnt;
  TRIG8(1)(13 downto 11) <= swc_dbg.free(4).fsm;
  TRIG8(1)(14)           <= swc_dbg.free(4).ib_ffree;
  TRIG8(1)(15)           <= swc_dbg.free(4).ib_ffree_done;
  TRIG8(1)(21 downto 16) <= swc_dbg.free(4).ib_pgadr(5 downto 0);
  TRIG8(1)(22)           <= swc_dbg.free(4).ob_free;
  TRIG8(1)(23)           <= swc_dbg.free(4).ob_free_done;
  TRIG8(1)(29 downto 24) <= swc_dbg.free(4).ob_pgadr(5 downto 0);
  TRIG8(1)(31 downto 30) <= (others=>'0');

  TRIG9(1)(0)            <= swc_dbg.free(5).free;
  TRIG9(1)(1)            <= swc_dbg.free(5).free_done;
  TRIG9(1)(2)            <= swc_dbg.free(5).ffree;
  TRIG9(1)(3)            <= swc_dbg.free(5).ffree_done;
  TRIG9(1)(9 downto 4)   <= swc_dbg.free(5).pgadr(5 downto 0);
  TRIG9(1)(10)           <= swc_dbg.free(5).last_ucnt;
  TRIG9(1)(13 downto 11) <= swc_dbg.free(5).fsm;
  TRIG9(1)(14)           <= swc_dbg.free(5).ib_ffree;
  TRIG9(1)(15)           <= swc_dbg.free(5).ib_ffree_done;
  TRIG9(1)(21 downto 16) <= swc_dbg.free(5).ib_pgadr(5 downto 0);
  TRIG9(1)(22)           <= swc_dbg.free(5).ob_free;
  TRIG9(1)(23)           <= swc_dbg.free(5).ob_free_done;
  TRIG9(1)(29 downto 24) <= swc_dbg.free(5).ob_pgadr(5 downto 0);
  TRIG9(1)(31 downto 30) <= (others=>'0');

  TRIG10(1)(0)           <= swc_dbg.free(8).free;
  TRIG10(1)(1)           <= swc_dbg.free(8).free_done;
  TRIG10(1)(2)           <= swc_dbg.free(8).ffree;
  TRIG10(1)(3)           <= swc_dbg.free(8).ffree_done;
  TRIG10(1)(9 downto 4)  <= swc_dbg.free(8).pgadr(5 downto 0);
  TRIG10(1)(10)          <= swc_dbg.free(8).last_ucnt;
  TRIG10(1)(13 downto 11)<= swc_dbg.free(8).fsm;
  TRIG10(1)(14)          <= swc_dbg.free(8).ib_ffree;
  TRIG10(1)(15)          <= swc_dbg.free(8).ib_ffree_done;
  TRIG10(1)(21 downto 16)<= swc_dbg.free(8).ib_pgadr(5 downto 0);
  TRIG10(1)(22)          <= swc_dbg.free(8).ob_free;
  TRIG10(1)(23)          <= swc_dbg.free(8).ob_free_done;
  TRIG10(1)(29 downto 24)<= swc_dbg.free(8).ob_pgadr(5 downto 0);
  TRIG10(1)(31 downto 30)<= (others=>'0');

  -------------------------------------------------------------------
  ---- bank 2
  TRIG0(2)(5 downto 0) <= swc_dbg.mmu.palloc.free_pages(5 downto 0);
  TRIG0(2)(11 downto 6)  <= swc_dbg.ib(0).page_start(5 downto 0);
  TRIG0(2)(17 downto 12) <= swc_dbg.ib(0).page_inter(5 downto 0);
  TRIG0(2)(23 downto 18) <= swc_dbg.ib(1).page_start(5 downto 0);
  TRIG0(2)(29 downto 24) <= swc_dbg.ib(1).page_inter(5 downto 0);
  TRIG1(2)(5 downto 0)   <= swc_dbg.ib(6).page_start(5 downto 0);
  TRIG1(2)(11 downto 6)  <= swc_dbg.ib(6).page_inter(5 downto 0);
  TRIG1(2)(17 downto 12) <= swc_dbg.ib(7).page_start(5 downto 0);
  TRIG1(2)(23 downto 18) <= swc_dbg.ib(7).page_inter(5 downto 0);
  TRIG1(2)(24)           <= swc_dbg.ib(0).pta_transfer;
  TRIG1(2)(25)           <= swc_dbg.ib(1).pta_transfer;
  TRIG1(2)(26)           <= swc_dbg.ib(6).pta_transfer;
  TRIG1(2)(27)           <= swc_dbg.ib(7).pta_transfer;
  TRIG1(2)(28)           <= swc_dbg.ib(0).rcv_stuck;
  TRIG1(2)(29)           <= swc_dbg.ib(1).rcv_stuck;
  TRIG1(2)(30)           <= swc_dbg.ib(6).rcv_stuck;
  TRIG1(2)(31)           <= swc_dbg.ib(7).rcv_stuck;

  TRIG2(2)(0)            <= swc_dbg.free(0).ffree;
  TRIG2(2)(1)            <= swc_dbg.free(0).ffree_done;
  TRIG2(2)(7 downto 2)   <= swc_dbg.free(0).pgadr(5 downto 0);
  TRIG2(2)(8)            <= swc_dbg.free(0).last_ucnt;
  TRIG2(2)(11 downto 9)  <= swc_dbg.free(0).fsm;
  TRIG2(2)(12)           <= swc_dbg.free(1).ffree;
  TRIG2(2)(13)           <= swc_dbg.free(1).ffree_done;
  TRIG2(2)(19 downto 14) <= swc_dbg.free(1).pgadr(5 downto 0);
  TRIG2(2)(20)           <= swc_dbg.free(1).last_ucnt;
  TRIG2(2)(23 downto 21) <= swc_dbg.free(1).fsm;
  TRIG2(2)(24)           <= swc_dbg.ib(0).pck_sof;
  TRIG2(2)(25)           <= swc_dbg.ib(0).pck_eof;
  TRIG2(2)(26)           <= swc_dbg.ib(1).pck_sof;
  TRIG2(2)(27)           <= swc_dbg.ib(1).pck_eof;
  TRIG2(2)(28)           <= swc_dbg.ib(6).pck_sof;
  TRIG2(2)(29)           <= swc_dbg.ib(6).pck_eof;
  TRIG2(2)(30)           <= swc_dbg.ib(7).pck_sof;
  TRIG2(2)(31)           <= swc_dbg.ib(7).pck_eof;

  TRIG3(2)(0)            <= swc_dbg.free(6).ffree;
  TRIG3(2)(1)            <= swc_dbg.free(6).ffree_done;
  TRIG3(2)(7 downto 2)   <= swc_dbg.free(6).pgadr(5 downto 0);
  TRIG3(2)(8)            <= swc_dbg.free(6).last_ucnt;
  TRIG3(2)(11 downto 9)  <= swc_dbg.free(6).fsm;
  TRIG3(2)(12)           <= swc_dbg.free(7).ffree;
  TRIG3(2)(13)           <= swc_dbg.free(7).ffree_done;
  TRIG3(2)(19 downto 14) <= swc_dbg.free(7).pgadr(5 downto 0);
  TRIG3(2)(20)           <= swc_dbg.free(7).last_ucnt;
  TRIG3(2)(23 downto 21) <= swc_dbg.free(7).fsm;
  TRIG3(2)(24)           <= swc_dbg.ib(0).force_free;
  TRIG3(2)(25)           <= swc_dbg.ib(0).force_free_done;
  TRIG3(2)(26)           <= swc_dbg.ib(1).force_free;
  TRIG3(2)(27)           <= swc_dbg.ib(1).force_free_done;
  TRIG3(2)(28)           <= swc_dbg.ib(6).force_free;
  TRIG3(2)(29)           <= swc_dbg.ib(6).force_free_done;
  TRIG3(2)(30)           <= swc_dbg.ib(7).force_free;
  TRIG3(2)(31)           <= swc_dbg.ib(7).force_free_done;

  TRIG4(2)(3 downto 0)   <= swc_dbg.ib(0).rcv_fsm;
  TRIG4(2)(7 downto 4)   <= swc_dbg.ib(0).trans_fsm;
  TRIG4(2)(11 downto 8)  <= swc_dbg.ib(0).ll_fsm;
  TRIG4(2)(17 downto 12) <= swc_dbg.ib(0).force_free_adr(5 downto 0);
  TRIG4(2)(23 downto 18) <= swc_dbg.ib(0).pta_pgadr(5 downto 0);
  TRIG4(2)(31 downto 24) <= swc_dbg.ib(0).pta_mask;

  TRIG5(2)(3 downto 0)   <= swc_dbg.ib(1).rcv_fsm;
  TRIG5(2)(7 downto 4)   <= swc_dbg.ib(1).trans_fsm;
  TRIG5(2)(11 downto 8)  <= swc_dbg.ib(1).ll_fsm;
  TRIG5(2)(17 downto 12) <= swc_dbg.ib(1).force_free_adr(5 downto 0);
  TRIG5(2)(23 downto 18) <= swc_dbg.ib(1).pta_pgadr(5 downto 0);
  TRIG5(2)(31 downto 24) <= swc_dbg.ib(1).pta_mask;

  TRIG6(2)(3 downto 0)   <= swc_dbg.ib(6).rcv_fsm;
  TRIG6(2)(7 downto 4)   <= swc_dbg.ib(6).trans_fsm;
  TRIG6(2)(11 downto 8)  <= swc_dbg.ib(6).ll_fsm;
  TRIG6(2)(17 downto 12) <= swc_dbg.ib(6).force_free_adr(5 downto 0);
  TRIG6(2)(23 downto 18) <= swc_dbg.ib(6).pta_pgadr(5 downto 0);
  TRIG6(2)(31 downto 24) <= swc_dbg.ib(6).pta_mask;

  TRIG7(2)(3 downto 0)   <= swc_dbg.ib(7).rcv_fsm;
  TRIG7(2)(7 downto 4)   <= swc_dbg.ib(7).trans_fsm;
  TRIG7(2)(11 downto 8)  <= swc_dbg.ib(7).ll_fsm;
  TRIG7(2)(17 downto 12) <= swc_dbg.ib(7).force_free_adr(5 downto 0);
  TRIG7(2)(23 downto 18) <= swc_dbg.ib(7).pta_pgadr(5 downto 0);
  TRIG7(2)(31 downto 24) <= swc_dbg.ib(7).pta_mask;

  TRIG8(2)(0) <= swc_dbg.ib(0).rtu_valid;
  TRIG8(2)(1) <= swc_dbg.ib(0).rtu_ack;
  TRIG8(2)(2) <= swc_dbg.ib(1).rtu_valid;
  TRIG8(2)(3) <= swc_dbg.ib(1).rtu_ack;
  TRIG8(2)(4) <= swc_dbg.ib(6).rtu_valid;
  TRIG8(2)(5) <= swc_dbg.ib(6).rtu_ack;
  TRIG8(2)(6) <= swc_dbg.ib(7).rtu_valid;
  TRIG8(2)(7) <= swc_dbg.ib(7).rtu_ack;
  TRIG8(2)(8)  <= swc_dbg.ib(0).cyc;
  TRIG8(2)(9)  <= swc_dbg.ib(0).stb;
  TRIG8(2)(10) <= swc_dbg.ib(0).tr_drop_stuck;
  TRIG8(2)(11) <= swc_dbg.ib(0).stall;
  TRIG8(2)(12) <= swc_dbg.ib(1).cyc;
  TRIG8(2)(13) <= swc_dbg.ib(1).stb;
  TRIG8(2)(14) <= swc_dbg.ib(1).tr_drop_stuck;
  TRIG8(2)(15) <= swc_dbg.ib(1).stall;
  TRIG8(2)(16) <= swc_dbg.ib(6).cyc;
  TRIG8(2)(17) <= swc_dbg.ib(6).stb;
  TRIG8(2)(18) <= swc_dbg.ib(6).tr_drop_stuck;
  TRIG8(2)(19) <= swc_dbg.ib(6).stall;
  TRIG8(2)(20) <= swc_dbg.ib(7).cyc;
  TRIG8(2)(21) <= swc_dbg.ib(7).stb;
  TRIG8(2)(22) <= swc_dbg.ib(7).tr_drop_stuck;
  TRIG8(2)(23) <= swc_dbg.ib(7).stall;
  TRIG8(2)(29 downto 24) <= swc_dbg.ib(7).ll_page(5 downto 0);
  TRIG8(2)(30) <= swc_dbg.ib(7).mpm_pg_req_d0;
  TRIG8(2)(31) <= swc_dbg.ib(7).mpm_dlast_d0;

  TRIG9(2)(5 downto 0)   <= swc_dbg.ib(7).ll_next_page(5 downto 0);
  TRIG9(2)(11 downto 6)  <= swc_dbg.ib(6).ll_page(5 downto 0);
  TRIG9(2)(17 downto 12) <= swc_dbg.ib(6).ll_next_page(5 downto 0);
  TRIG9(2)(23 downto 18) <= swc_dbg.ib(7).mpm_pg_adr(5 downto 0);
  TRIG9(2)(29 downto 24) <= swc_dbg.ib(6).mpm_pg_adr(5 downto 0);
  TRIG9(2)(30) <= swc_dbg.ib(6).mpm_pg_req_d0;
  TRIG9(2)(31) <= swc_dbg.ib(6).mpm_dlast_d0;

  -------------------------------------------------------------------
  -- bank 3
  TRIG0(3)(5 downto 0) <= swc_dbg.mmu.palloc.free_pages(5 downto 0);
  TRIG0(3)(11 downto 6)  <= swc_dbg.ib(0).page_start(5 downto 0);
  TRIG0(3)(17 downto 12) <= swc_dbg.ib(0).page_inter(5 downto 0);
  TRIG0(3)(23 downto 18) <= swc_dbg.ib(1).page_start(5 downto 0);
  TRIG0(3)(29 downto 24) <= swc_dbg.ib(1).page_inter(5 downto 0);
  TRIG1(3)(5 downto 0)   <= swc_dbg.ib(6).page_start(5 downto 0);
  TRIG1(3)(11 downto 6)  <= swc_dbg.ib(6).page_inter(5 downto 0);
  TRIG1(3)(17 downto 12) <= swc_dbg.ib(7).page_start(5 downto 0);
  TRIG1(3)(23 downto 18) <= swc_dbg.ib(7).page_inter(5 downto 0);
  TRIG1(3)(24)           <= swc_dbg.ib(0).rcv_stuck;
  TRIG1(3)(25)           <= swc_dbg.ib(1).rcv_stuck;
  TRIG1(3)(26)           <= swc_dbg.ib(6).rcv_stuck;
  TRIG1(3)(27)           <= swc_dbg.ib(7).rcv_stuck;
  TRIG1(3)(28) <= swc_dbg.ib(0).pages_same;
  TRIG1(3)(29) <= swc_dbg.ib(1).pages_same;
  TRIG1(3)(30) <= swc_dbg.ib(6).pages_same;
  TRIG1(3)(31) <= swc_dbg.ib(7).pages_same;

  TRIG2(3)(0)            <= swc_dbg.free(0).free;
  TRIG2(3)(1)            <= swc_dbg.free(0).free_done;
  TRIG2(3)(2)            <= swc_dbg.free(0).ffree;
  TRIG2(3)(3)            <= swc_dbg.free(0).ffree_done;
  TRIG2(3)(10 downto 4)  <= swc_dbg.ib(0).rp_ll_entry_size;
  TRIG2(3)(13 downto 11) <= swc_dbg.free(0).fsm;
  TRIG2(3)(14)           <= swc_dbg.free(0).ib_ffree;
  TRIG2(3)(15)           <= swc_dbg.free(0).ib_ffree_done;
  TRIG2(3)(21 downto 16) <= swc_dbg.free(0).ib_pgadr(5 downto 0);
  TRIG2(3)(22)           <= swc_dbg.ib(0).ll_start;
  TRIG2(3)(23)           <= swc_dbg.ib(0).ll_start_ack;
  TRIG2(3)(24)           <= swc_dbg.ib(0).pckstart_in_adv;
  TRIG2(3)(25)           <= swc_dbg.ib(0).pckinter_in_adv;
  TRIG2(3)(30)           <= swc_dbg.ib(0).ll_wr;
  TRIG2(3)(31)           <= swc_dbg.ib(0).ll_wr_done;

  TRIG3(3)(0)            <= swc_dbg.free(1).free;
  TRIG3(3)(1)            <= swc_dbg.free(1).free_done;
  TRIG3(3)(2)            <= swc_dbg.free(1).ffree;
  TRIG3(3)(3)            <= swc_dbg.free(1).ffree_done;
  TRIG3(3)(10 downto 4)  <= swc_dbg.ib(1).rp_ll_entry_size;
  TRIG3(3)(13 downto 11) <= swc_dbg.free(1).fsm;
  TRIG3(3)(14)           <= swc_dbg.free(1).ib_ffree;
  TRIG3(3)(15)           <= swc_dbg.free(1).ib_ffree_done;
  TRIG3(3)(21 downto 16) <= swc_dbg.free(1).ib_pgadr(5 downto 0);
  TRIG3(3)(22)           <= swc_dbg.ib(1).ll_start;
  TRIG3(3)(23)           <= swc_dbg.ib(1).ll_start_ack;
  TRIG3(3)(24)           <= swc_dbg.ib(1).pckstart_in_adv;
  TRIG3(3)(25)           <= swc_dbg.ib(1).pckinter_in_adv;
  TRIG3(3)(30)           <= swc_dbg.ib(1).ll_wr;
  TRIG3(3)(31)           <= swc_dbg.ib(1).ll_wr_done;

  TRIG4(3)(0)            <= swc_dbg.free(6).free;
  TRIG4(3)(1)            <= swc_dbg.free(6).free_done;
  TRIG4(3)(2)            <= swc_dbg.free(6).ffree;
  TRIG4(3)(3)            <= swc_dbg.free(6).ffree_done;
  TRIG4(3)(10 downto 4)  <= swc_dbg.ib(6).rp_ll_entry_size;
  TRIG4(3)(13 downto 11) <= swc_dbg.free(6).fsm;
  TRIG4(3)(14)           <= swc_dbg.free(6).ib_ffree;
  TRIG4(3)(15)           <= swc_dbg.free(6).ib_ffree_done;
  TRIG4(3)(21 downto 16) <= swc_dbg.free(6).ib_pgadr(5 downto 0);
  TRIG4(3)(22)           <= swc_dbg.ib(6).ll_start;
  TRIG4(3)(23)           <= swc_dbg.ib(6).ll_start_ack;
  TRIG4(3)(24)           <= swc_dbg.ib(6).pckstart_in_adv;
  TRIG4(3)(25)           <= swc_dbg.ib(6).pckinter_in_adv;
  TRIG4(3)(30)           <= swc_dbg.ib(6).ll_wr;
  TRIG4(3)(31)           <= swc_dbg.ib(6).ll_wr_done;

  TRIG5(3)(0)            <= swc_dbg.free(7).free;
  TRIG5(3)(1)            <= swc_dbg.free(7).free_done;
  TRIG5(3)(2)            <= swc_dbg.free(7).ffree;
  TRIG5(3)(3)            <= swc_dbg.free(7).ffree_done;
  TRIG5(3)(10 downto 4)  <= swc_dbg.ib(7).rp_ll_entry_size;
  TRIG5(3)(13 downto 11) <= swc_dbg.free(7).fsm;
  TRIG5(3)(14)           <= swc_dbg.free(7).ib_ffree;
  TRIG5(3)(15)           <= swc_dbg.free(7).ib_ffree_done;
  TRIG5(3)(21 downto 16) <= swc_dbg.free(7).ib_pgadr(5 downto 0);
  TRIG5(3)(22)           <= swc_dbg.ib(7).ll_start;
  TRIG5(3)(23)           <= swc_dbg.ib(7).ll_start_ack;
  TRIG5(3)(24)           <= swc_dbg.ib(7).pckstart_in_adv;
  TRIG5(3)(25)           <= swc_dbg.ib(7).pckinter_in_adv;
  TRIG5(3)(30)           <= swc_dbg.ib(7).ll_wr;
  TRIG5(3)(31)           <= swc_dbg.ib(7).ll_wr_done;

  TRIG6(3)(3 downto 0)   <= swc_dbg.ib(0).rcv_fsm;
  TRIG6(3)(7 downto 4)   <= swc_dbg.ib(0).trans_fsm;
  TRIG6(3)(11 downto 8)  <= swc_dbg.ib(0).ll_fsm;
  TRIG6(3)(12)           <= swc_dbg.ib(0).new_pck_first_page;
  TRIG6(3)(13)           <= swc_dbg.ib(0).mpm_dlast_d0;
  TRIG6(3)(14)           <= swc_dbg.ib(0).mpm_pg_req_d0;
  TRIG6(3)(15)           <= swc_dbg.ib(0).rp_rcv_fpage;
  TRIG6(3)(16)           <= swc_dbg.mll.rv(0).rd_valid;
  TRIG6(3)(17)           <= swc_dbg.mll.rv(0).req_o;
  TRIG6(3)(23 downto 18) <= swc_dbg.mll.rv(0).rd_adr(5 downto 0);
  TRIG6(3)(29 downto 24) <= swc_dbg.ib(0).ll_page(5 downto 0);
  TRIG6(3)(30)           <= swc_dbg.ib(0).ll_page_valid;
  TRIG6(3)(31)           <= swc_dbg.ib(0).ll_eof;

  TRIG7(3)(3 downto 0)   <= swc_dbg.ib(1).rcv_fsm;
  TRIG7(3)(7 downto 4)   <= swc_dbg.ib(1).trans_fsm;
  TRIG7(3)(11 downto 8)  <= swc_dbg.ib(1).ll_fsm;
  TRIG7(3)(12)           <= swc_dbg.ib(1).new_pck_first_page;
  TRIG7(3)(13)           <= swc_dbg.ib(1).mpm_dlast_d0;
  TRIG7(3)(14)           <= swc_dbg.ib(1).mpm_pg_req_d0;
  TRIG7(3)(15)           <= swc_dbg.ib(1).rp_rcv_fpage;
  TRIG7(3)(16)           <= swc_dbg.mll.rv(1).rd_valid;
  TRIG7(3)(17)           <= swc_dbg.mll.rv(1).req_o;
  TRIG7(3)(23 downto 18) <= swc_dbg.mll.rv(1).rd_adr(5 downto 0);
  TRIG7(3)(29 downto 24) <= swc_dbg.ib(1).ll_page(5 downto 0);
  TRIG7(3)(30)           <= swc_dbg.ib(1).ll_page_valid;
  TRIG7(3)(31)           <= swc_dbg.ib(1).ll_eof;

  TRIG8(3)(3 downto 0)   <= swc_dbg.ib(6).rcv_fsm;
  TRIG8(3)(7 downto 4)   <= swc_dbg.ib(6).trans_fsm;
  TRIG8(3)(11 downto 8)  <= swc_dbg.ib(6).ll_fsm;
  TRIG8(3)(12)           <= swc_dbg.ib(6).new_pck_first_page;
  TRIG8(3)(13)           <= swc_dbg.ib(6).mpm_dlast_d0;
  TRIG8(3)(14)           <= swc_dbg.ib(6).mpm_pg_req_d0;
  TRIG8(3)(15)           <= swc_dbg.ib(6).rp_rcv_fpage;
  TRIG8(3)(16)           <= swc_dbg.mll.rv(6).rd_valid;
  TRIG8(3)(17)           <= swc_dbg.mll.rv(6).req_o;
  TRIG8(3)(23 downto 18) <= swc_dbg.mll.rv(6).rd_adr(5 downto 0);
  TRIG8(3)(29 downto 24) <= swc_dbg.ib(6).ll_page(5 downto 0);
  TRIG8(3)(30)           <= swc_dbg.ib(6).ll_page_valid;
  TRIG8(3)(31)           <= swc_dbg.ib(6).ll_eof;

  TRIG9(3)(3 downto 0)   <= swc_dbg.ib(7).rcv_fsm;
  TRIG9(3)(7 downto 4)   <= swc_dbg.ib(7).trans_fsm;
  TRIG9(3)(11 downto 8)  <= swc_dbg.ib(7).ll_fsm;
  TRIG9(3)(12)           <= swc_dbg.ib(7).new_pck_first_page;
  TRIG9(3)(13)           <= swc_dbg.ib(7).mpm_dlast_d0;
  TRIG9(3)(14)           <= swc_dbg.ib(7).mpm_pg_req_d0;
  TRIG9(3)(15)           <= swc_dbg.ib(7).rp_rcv_fpage;
  TRIG9(3)(16)           <= swc_dbg.mll.rv(7).rd_valid;
  TRIG9(3)(17)           <= swc_dbg.mll.rv(7).req_o;
  TRIG9(3)(23 downto 18) <= swc_dbg.mll.rv(7).rd_adr(5 downto 0);
  TRIG9(3)(29 downto 24) <= swc_dbg.ib(7).ll_page(5 downto 0);
  TRIG9(3)(30)           <= swc_dbg.ib(7).ll_page_valid;
  TRIG9(3)(31)           <= swc_dbg.ib(7).ll_eof;

  TRIG10(3)(0)           <= swc_dbg.ob(0).cycle_frozen;
  TRIG10(3)(1)           <= swc_dbg.ob(1).cycle_frozen;
  TRIG10(3)(2)           <= swc_dbg.ob(6).cycle_frozen;
  TRIG10(3)(3)           <= swc_dbg.ob(7).cycle_frozen;
  TRIG10(3)(4)           <= swc_dbg.ib(0).finish_rcv;
  TRIG10(3)(5)           <= swc_dbg.ib(0).pck_eof;
  TRIG10(3)(6)           <= swc_dbg.ib(0).pck_err;
  TRIG10(3)(7)           <= swc_dbg.ib(0).mem_full_dump;
  TRIG10(3)(8)           <= swc_dbg.ib(0).in_pck_dvalid;
  TRIG10(3)(9)           <= swc_dbg.ib(1).finish_rcv;
  TRIG10(3)(10)          <= swc_dbg.ib(1).pck_eof;
  TRIG10(3)(11)          <= swc_dbg.ib(1).pck_err;
  TRIG10(3)(12)          <= swc_dbg.ib(1).mem_full_dump;
  TRIG10(3)(13)          <= swc_dbg.ib(1).in_pck_dvalid;
  TRIG10(3)(14)          <= swc_dbg.ib(6).finish_rcv;
  TRIG10(3)(15)          <= swc_dbg.ib(6).pck_eof;
  TRIG10(3)(16)          <= swc_dbg.ib(6).pck_err;
  TRIG10(3)(17)          <= swc_dbg.ib(6).mem_full_dump;
  TRIG10(3)(18)          <= swc_dbg.ib(6).in_pck_dvalid;
  TRIG10(3)(19)          <= swc_dbg.ib(7).finish_rcv;
  TRIG10(3)(20)          <= swc_dbg.ib(7).pck_eof;
  TRIG10(3)(21)          <= swc_dbg.ib(7).pck_err;
  TRIG10(3)(22)          <= swc_dbg.ib(7).mem_full_dump;
  TRIG10(3)(23)          <= swc_dbg.ib(7).in_pck_dvalid;

  --bank 4
  TRIG0(4)(5 downto 0) <= swc_dbg.mmu.palloc.free_pages(5 downto 0);
  TRIG0(4)(11 downto 6)  <= swc_dbg.ib(0).page_start(5 downto 0);
  TRIG0(4)(17 downto 12) <= swc_dbg.ib(0).page_inter(5 downto 0);
  TRIG0(4)(23 downto 18) <= swc_dbg.ib(1).page_start(5 downto 0);
  TRIG0(4)(29 downto 24) <= swc_dbg.ib(1).page_inter(5 downto 0);
  TRIG1(4)(5 downto 0)   <= swc_dbg.ib(6).page_start(5 downto 0);
  TRIG1(4)(11 downto 6)  <= swc_dbg.ib(6).page_inter(5 downto 0);
  TRIG1(4)(17 downto 12) <= swc_dbg.ib(7).page_start(5 downto 0);
  TRIG1(4)(23 downto 18) <= swc_dbg.ib(7).page_inter(5 downto 0);
  TRIG1(4)(24)           <= swc_dbg.ib(0).rcv_stuck;
  TRIG1(4)(25)           <= swc_dbg.ib(1).rcv_stuck;
  TRIG1(4)(26)           <= swc_dbg.ib(6).rcv_stuck;
  TRIG1(4)(27)           <= swc_dbg.ib(7).rcv_stuck;
  TRIG1(4)(28) <= swc_dbg.ib(0).new_pck_first_page;
  TRIG1(4)(29) <= swc_dbg.ib(1).new_pck_first_page;
  TRIG1(4)(30) <= swc_dbg.ib(6).new_pck_first_page;
  TRIG1(4)(31) <= swc_dbg.ib(7).new_pck_first_page;

  TRIG2(4)(2 downto 0)    <= swc_dbg.ib(0).alloc_fsm;
  TRIG2(4)(6 downto 3)    <= swc_dbg.ib(0).rcv_fsm;
  TRIG2(4)(10 downto 7)   <= swc_dbg.ib(0).trans_fsm;
  TRIG2(4)(14 downto 11)  <= swc_dbg.ib(0).ll_fsm;
  TRIG2(4)(15)            <= swc_dbg.ib(0).ll_wr;
  TRIG2(4)(16)            <= swc_dbg.ib(0).ll_wr_done;
  TRIG2(4)(17)            <= swc_dbg.ib(0).ll_page_valid;
  TRIG2(4)(18)            <= swc_dbg.ib(0).ll_eof;
  TRIG2(4)(24 downto  19) <= swc_dbg.ib(0).ll_page(5 downto 0);
  TRIG2(4)(30 downto 25)  <= swc_dbg.ib(0).ll_next_page(5 downto 0);
  TRIG2(4)(31)            <= swc_dbg.ib(0).mpm_dlast_d0;

  TRIG3(4)(2 downto 0)    <= swc_dbg.ib(1).alloc_fsm;
  TRIG3(4)(6 downto 3)    <= swc_dbg.ib(1).rcv_fsm;
  TRIG3(4)(10 downto 7)   <= swc_dbg.ib(1).trans_fsm;
  TRIG3(4)(14 downto 11)  <= swc_dbg.ib(1).ll_fsm;
  TRIG3(4)(15)            <= swc_dbg.ib(1).ll_wr;
  TRIG3(4)(16)            <= swc_dbg.ib(1).ll_wr_done;
  TRIG3(4)(17)            <= swc_dbg.ib(1).ll_page_valid;
  TRIG3(4)(18)            <= swc_dbg.ib(1).ll_eof;
  TRIG3(4)(24 downto  19) <= swc_dbg.ib(1).ll_page(5 downto 0);
  TRIG3(4)(30 downto 25)  <= swc_dbg.ib(1).ll_next_page(5 downto 0);
  TRIG3(4)(31)            <= swc_dbg.ib(1).mpm_dlast_d0;

  TRIG4(4)(2 downto 0)    <= swc_dbg.ib(6).alloc_fsm;
  TRIG4(4)(6 downto 3)    <= swc_dbg.ib(6).rcv_fsm;
  TRIG4(4)(10 downto 7)   <= swc_dbg.ib(6).trans_fsm;
  TRIG4(4)(14 downto 11)  <= swc_dbg.ib(6).ll_fsm;
  TRIG4(4)(15)            <= swc_dbg.ib(6).ll_wr;
  TRIG4(4)(16)            <= swc_dbg.ib(6).ll_wr_done;
  TRIG4(4)(17)            <= swc_dbg.ib(6).ll_page_valid;
  TRIG4(4)(18)            <= swc_dbg.ib(6).ll_eof;
  TRIG4(4)(24 downto  19) <= swc_dbg.ib(6).ll_page(5 downto 0);
  TRIG4(4)(30 downto 25)  <= swc_dbg.ib(6).ll_next_page(5 downto 0);
  TRIG4(4)(31)            <= swc_dbg.ib(6).mpm_dlast_d0;

  TRIG5(4)(2 downto 0)    <= swc_dbg.ib(7).alloc_fsm;
  TRIG5(4)(6 downto 3)    <= swc_dbg.ib(7).rcv_fsm;
  TRIG5(4)(10 downto 7)   <= swc_dbg.ib(7).trans_fsm;
  TRIG5(4)(14 downto 11)  <= swc_dbg.ib(7).ll_fsm;
  TRIG5(4)(15)            <= swc_dbg.ib(7).ll_wr;
  TRIG5(4)(16)            <= swc_dbg.ib(7).ll_wr_done;
  TRIG5(4)(17)            <= swc_dbg.ib(7).ll_page_valid;
  TRIG5(4)(18)            <= swc_dbg.ib(7).ll_eof;
  TRIG5(4)(24 downto  19) <= swc_dbg.ib(7).ll_page(5 downto 0);
  TRIG5(4)(30 downto 25)  <= swc_dbg.ib(7).ll_next_page(5 downto 0);
  TRIG5(4)(31)            <= swc_dbg.ib(7).mpm_dlast_d0;

  TRIG6(4)(0)           <= swc_dbg.ib(0).force_free;
  TRIG6(4)(1)           <= swc_dbg.ib(0).force_free_done;
  TRIG6(4)(2)           <= swc_dbg.ib(1).force_free;
  TRIG6(4)(3)           <= swc_dbg.ib(1).force_free_done;
  TRIG6(4)(4)           <= swc_dbg.ib(6).force_free;
  TRIG6(4)(5)           <= swc_dbg.ib(6).force_free_done;
  TRIG6(4)(6)           <= swc_dbg.ib(7).force_free;
  TRIG6(4)(7)           <= swc_dbg.ib(7).force_free_done;
  TRIG6(4)(8)           <= swc_dbg.ib(0).ll_pckstart_stored;
  TRIG6(4)(9)           <= swc_dbg.ib(1).ll_pckstart_stored;
  TRIG6(4)(10)          <= swc_dbg.ib(6).ll_pckstart_stored;
  TRIG6(4)(11)          <= swc_dbg.ib(7).ll_pckstart_stored;
  TRIG6(4)(12)          <= swc_dbg.ib(0).ffree_mask;
  TRIG6(4)(13)          <= swc_dbg.ib(1).ffree_mask;
  TRIG6(4)(14)          <= swc_dbg.ib(6).ffree_mask;
  TRIG6(4)(15)          <= swc_dbg.ib(7).ffree_mask;
  TRIG6(4)(16)          <= swc_dbg.ib(0).tr_drop_stuck;
  TRIG6(4)(17)          <= swc_dbg.ib(1).tr_drop_stuck;
  TRIG6(4)(18)          <= swc_dbg.ib(6).tr_drop_stuck;
  TRIG6(4)(19)          <= swc_dbg.ib(7).tr_drop_stuck;
  TRIG6(4)(20)          <= swc_dbg.ib(0).pck_sof;
  TRIG6(4)(21)          <= swc_dbg.ib(0).pck_eof;
  TRIG6(4)(22)          <= swc_dbg.ib(1).pck_sof;
  TRIG6(4)(23)          <= swc_dbg.ib(1).pck_eof;
  TRIG6(4)(24)          <= swc_dbg.ib(6).pck_sof;
  TRIG6(4)(25)          <= swc_dbg.ib(6).pck_eof;
  TRIG6(4)(26)          <= swc_dbg.ib(7).pck_sof;
  TRIG6(4)(27)          <= swc_dbg.ib(7).pck_eof;
  TRIG6(4)(28)          <= swc_dbg.ib(0).tr_wait_stuck;
  TRIG6(4)(29)          <= swc_dbg.ib(1).tr_wait_stuck;
  TRIG6(4)(30)          <= swc_dbg.ib(6).tr_wait_stuck;
  TRIG6(4)(31)          <= swc_dbg.ib(7).tr_wait_stuck;

  TRIG7(4)(5 downto 0)  <= swc_dbg.ib(0).cur_pckstart(5 downto 0);
  TRIG7(4)(11 downto 6) <= swc_dbg.ib(0).mpm_pg_adr(5 downto 0);
  TRIG7(4)(17 downto 12)<= swc_dbg.ib(1).cur_pckstart(5 downto 0);
  TRIG7(4)(23 downto 18)<= swc_dbg.ib(1).mpm_pg_adr(5 downto 0);
  TRIG7(4)(24)          <= swc_dbg.ib(0).rtu_valid;
  TRIG7(4)(25)          <= swc_dbg.ib(0).rtu_ack;
  TRIG7(4)(26)          <= swc_dbg.ib(1).rtu_valid;
  TRIG7(4)(27)          <= swc_dbg.ib(1).rtu_ack;
  TRIG7(4)(28)          <= swc_dbg.ib(6).rtu_valid;
  TRIG7(4)(29)          <= swc_dbg.ib(6).rtu_ack;
  TRIG7(4)(30)          <= swc_dbg.ib(7).rtu_valid;
  TRIG7(4)(31)          <= swc_dbg.ib(7).rtu_ack;

  TRIG8(4)(5 downto 0)  <= swc_dbg.ib(6).cur_pckstart(5 downto 0);
  TRIG8(4)(11 downto 6) <= swc_dbg.ib(6).mpm_pg_adr(5 downto 0);
  TRIG8(4)(17 downto 12)<= swc_dbg.ib(7).cur_pckstart(5 downto 0);
  TRIG8(4)(23 downto 18)<= swc_dbg.ib(7).mpm_pg_adr(5 downto 0);
  TRIG8(4)(24)          <= swc_dbg.ob(0).cycle_frozen;
  TRIG8(4)(25)          <= swc_dbg.ob(1).cycle_frozen;
  TRIG8(4)(26)          <= swc_dbg.ob(6).cycle_frozen;
  TRIG8(4)(27)          <= swc_dbg.ob(7).cycle_frozen;
  TRIG8(4)(28)          <= swc_dbg.ib(0).mpm_pg_req_d0;
  TRIG8(4)(29)          <= swc_dbg.ib(1).mpm_pg_req_d0;
  TRIG8(4)(30)          <= swc_dbg.ib(6).mpm_pg_req_d0;
  TRIG8(4)(31)          <= swc_dbg.ib(7).mpm_pg_req_d0;

  TRIG9(4)(0)           <= swc_dbg.ib(0).mpm_dvalid;
  TRIG9(4)(1)           <= swc_dbg.ib(1).mpm_dvalid;
  TRIG9(4)(2)           <= swc_dbg.ib(6).mpm_dvalid;
  TRIG9(4)(3)           <= swc_dbg.ib(7).mpm_dvalid;
  TRIG9(4)(10 downto 4)  <= swc_dbg.ib(0).page_word_cnt;
  TRIG9(4)(17 downto 11) <= swc_dbg.ib(1).page_word_cnt;
  TRIG9(4)(24 downto 18) <= swc_dbg.ib(6).page_word_cnt;
  TRIG9(4)(31 downto 25) <= swc_dbg.ib(7).page_word_cnt;

  TRIG10(4)(0)          <= swc_dbg.ib(0).pck_err;
  TRIG10(4)(1)          <= swc_dbg.ib(1).pck_err;
  TRIG10(4)(2)          <= swc_dbg.ib(6).pck_err;
  TRIG10(4)(3)          <= swc_dbg.ib(7).pck_err;
  TRIG10(4)(4)          <= swc_dbg.ib(0).rtu_sof_bug;
  TRIG10(4)(5)          <= swc_dbg.ib(0).current_drop;
  TRIG10(4)(6)          <= swc_dbg.ib(0).almost_full_i;
  TRIG10(4)(7)          <= swc_dbg.ib(1).rtu_sof_bug;
  TRIG10(4)(8)          <= swc_dbg.ib(1).current_drop;
  TRIG10(4)(9)         <= swc_dbg.ib(1).almost_full_i;
  TRIG10(4)(10)         <= swc_dbg.ib(6).rtu_sof_bug;
  TRIG10(4)(11)         <= swc_dbg.ib(6).current_drop;
  TRIG10(4)(12)         <= swc_dbg.ib(6).almost_full_i;
  TRIG10(4)(13)         <= swc_dbg.ib(7).rtu_sof_bug;
  TRIG10(4)(14)         <= swc_dbg.ib(7).current_drop;
  TRIG10(4)(15)         <= swc_dbg.ib(7).almost_full_i;
  TRIG10(4)(23 downto 16) <= swc_dbg.ib(0).rtu_rsp_cnt;
  TRIG10(4)(31 downto 24) <= swc_dbg.ib(0).sof_cnt;

  TRIG11(4)(0)   <= or_reduce(std_logic_vector(dbg_rtu_bug(0)));
  TRIG11(4)(1)   <= or_reduce(std_logic_vector(dbg_rtu_bug(1)));
  TRIG11(4)(2)   <= or_reduce(std_logic_vector(dbg_rtu_bug(6)));
  TRIG11(4)(3)   <= or_reduce(std_logic_vector(dbg_rtu_bug(7)));
  TRIG11(4)(4)   <= swc_dbg.ib(0).pck_sof_rcv_reg;
  TRIG11(4)(5)   <= swc_dbg.ib(0).pck_sof_ack;
  TRIG11(4)(6)   <= swc_dbg.ib(1).pck_sof_rcv_reg;
  TRIG11(4)(7)   <= swc_dbg.ib(1).pck_sof_ack;
  TRIG11(4)(8)   <= swc_dbg.ib(6).pck_sof_rcv_reg;
  TRIG11(4)(9)   <= swc_dbg.ib(6).pck_sof_ack;
  TRIG11(4)(10)  <= swc_dbg.ib(7).pck_sof_rcv_reg;
  TRIG11(4)(11)  <= swc_dbg.ib(7).pck_sof_ack;
  TRIG11(4)(12)  <= swc_dbg.ib(0).sof_normal;
  TRIG11(4)(13)  <= swc_dbg.ib(0).sof_on_stall;
  TRIG11(4)(14)  <= swc_dbg.ib(0).sof_delayed;
  TRIG11(4)(15)  <= swc_dbg.ib(0).eof_normal;
  TRIG11(4)(16)  <= swc_dbg.ib(0).eof_on_pause;
  TRIG11(4)(17)  <= swc_dbg.ib(1).sof_normal;
  TRIG11(4)(18)  <= swc_dbg.ib(1).sof_on_stall;
  TRIG11(4)(19)  <= swc_dbg.ib(1).sof_delayed;
  TRIG11(4)(20)  <= swc_dbg.ib(1).eof_normal;
  TRIG11(4)(21)  <= swc_dbg.ib(1).eof_on_pause;
  TRIG11(4)(22)  <= swc_dbg.ib(6).sof_normal;
  TRIG11(4)(23)  <= swc_dbg.ib(6).sof_on_stall;
  TRIG11(4)(24)  <= swc_dbg.ib(6).sof_delayed;
  TRIG11(4)(25)  <= swc_dbg.ib(6).eof_normal;
  TRIG11(4)(26)  <= swc_dbg.ib(6).eof_on_pause;
  TRIG11(4)(27)  <= swc_dbg.ib(7).sof_normal;
  TRIG11(4)(28)  <= swc_dbg.ib(7).sof_on_stall;
  TRIG11(4)(29)  <= swc_dbg.ib(7).sof_delayed;
  TRIG11(4)(30)  <= swc_dbg.ib(7).eof_normal;
  TRIG11(4)(31)  <= swc_dbg.ib(7).eof_on_pause;

  --bank 5
  TRIG0(5)(5 downto 0) <= swc_dbg.mmu.palloc.free_pages(5 downto 0);
  TRIG0(5)(11 downto 6)  <= swc_dbg.ib(0).page_start(5 downto 0);
  TRIG0(5)(17 downto 12) <= swc_dbg.ib(0).page_inter(5 downto 0);
  TRIG0(5)(23 downto 18) <= swc_dbg.ib(1).page_start(5 downto 0);
  TRIG0(5)(29 downto 24) <= swc_dbg.ib(1).page_inter(5 downto 0);
  TRIG0(5)(31 downto 30) <= (others=>'0');

  TRIG1(5)(5 downto 0)   <= swc_dbg.ib(6).page_start(5 downto 0);
  TRIG1(5)(11 downto 6)  <= swc_dbg.ib(6).page_inter(5 downto 0);
  TRIG1(5)(17 downto 12) <= swc_dbg.ib(7).page_start(5 downto 0);
  TRIG1(5)(23 downto 18) <= swc_dbg.ib(7).page_inter(5 downto 0);
  TRIG1(5)(24)           <= swc_dbg.ib(0).rcv_stuck;
  TRIG1(5)(25)           <= swc_dbg.ib(1).rcv_stuck;
  TRIG1(5)(26)           <= swc_dbg.ib(6).rcv_stuck;
  TRIG1(5)(27)           <= swc_dbg.ib(7).rcv_stuck;
  TRIG1(5)(28)           <= swc_dbg.ob(8).cycle_frozen;
  TRIG1(5)(29)           <= swc_dbg.ob(1).cycle_frozen;
  TRIG1(5)(30)           <= swc_dbg.ob(6).cycle_frozen;
  TRIG1(5)(31)           <= swc_dbg.ob(7).cycle_frozen;

  TRIG2(5)(3 downto 0)   <= swc_dbg.ob(8).send_fsm;
  TRIG2(5)(6 downto 4)   <= swc_dbg.ob(8).prep_fsm(2 downto 0);
  TRIG2(5)(10 downto 7)  <= swc_dbg.ob(1).send_fsm;
  TRIG2(5)(13 downto 11) <= swc_dbg.ob(1).prep_fsm(2 downto 0);
  TRIG2(5)(17 downto 14) <= swc_dbg.ob(6).send_fsm;
  TRIG2(5)(20 downto 18) <= swc_dbg.ob(6).prep_fsm(2 downto 0);
  TRIG2(5)(24 downto 21) <= swc_dbg.ob(7).send_fsm;
  TRIG2(5)(27 downto 25) <= swc_dbg.ob(7).prep_fsm(2 downto 0);
  TRIG2(5)(28) <= swc_dbg.ob(8).mpm_pgreq;
  TRIG2(5)(29) <= swc_dbg.ob(1).mpm_pgreq;
  TRIG2(5)(30) <= swc_dbg.ob(6).mpm_pgreq;
  TRIG2(5)(31) <= swc_dbg.ob(7).mpm_pgreq;

  TRIG3(5)(5 downto 0)   <= swc_dbg.mpm.read.ll_adr(5 downto 0);
  TRIG3(5)(15 downto 6)  <= swc_dbg.mpm.read.io(8).words_xmitted;
  TRIG3(5)(25 downto 16) <= swc_dbg.mpm.read.io(8).words_total; --last_pg_start_ptr;
  TRIG3(5)(26)         <= swc_dbg.mpm.read.io(1).fetch_ack;
  TRIG3(5)(27)         <= swc_dbg.mpm.read.io(6).fetch_first;
  TRIG3(5)(28)         <= swc_dbg.mpm.read.io(6).fetch_ack;
  TRIG3(5)(29)         <= swc_dbg.mpm.read.io(7).fetch_first;
  TRIG3(5)(30)         <= swc_dbg.mpm.read.io(7).fetch_ack;

  TRIG4(5)(0)          <= swc_dbg.mpm.read.io(8).ll_req;
  TRIG4(5)(1)          <= swc_dbg.mpm.read.io(8).ll_grant;
  TRIG4(5)(7 downto 2) <= swc_dbg.mpm.read.io(8).ll_adr(5 downto 0);
  TRIG4(5)(14 downto 8)<= swc_dbg.mpm.read.io(8).fetch_pg_words;
  TRIG4(5)(15)         <= swc_dbg.mpm.read.io(8).ll_eof;
  TRIG4(5)(16)         <= swc_dbg.mpm.read.io(8).fetch_first;
  TRIG4(5)(17)         <= swc_dbg.mpm.read.io(8).fetch_ack;
  TRIG4(5)(24 downto 18) <= swc_dbg.mpm.read.io(8).ll_size;
  TRIG4(5)(25)         <= swc_dbg.mpm.read.io(1).fetch_first;
  TRIG4(5)(26)         <= swc_dbg.ob(8).data_error;
  TRIG4(5)(27)         <= swc_dbg.ob(8).mpm_dlast;
  TRIG4(5)(28)         <= swc_dbg.mpm.read.io(8).pre_fetch;
  TRIG4(5)(29)         <= swc_dbg.mpm.read.io(8).last_int;
  TRIG4(5)(30)         <= swc_dbg.mpm.read.io(8).rport_dreq;
  TRIG4(5)(31)         <= swc_dbg.mpm.read.io(8).rport_pg_req;

  TRIG5(5)(0)          <= swc_dbg.mpm.read.io(1).ll_req;
  TRIG5(5)(1)          <= swc_dbg.mpm.read.io(1).ll_grant;
  TRIG5(5)(7 downto 2) <= swc_dbg.mpm.read.io(1).ll_adr(5 downto 0);
  TRIG5(5)(14 downto 8)<= swc_dbg.mpm.read.io(1).fetch_pg_words;
  TRIG5(5)(15)         <= swc_dbg.mpm.read.io(1).ll_eof;
  TRIG5(5)(16)           <= swc_dbg.ib(1).ll_wr;
  TRIG5(5)(17)           <= swc_dbg.ib(1).ll_wr_done;
  TRIG5(5)(18)           <= swc_dbg.ib(1).ll_page_valid;
  TRIG5(5)(19)           <= swc_dbg.ib(1).ll_eof;
  TRIG5(5)(25 downto 20) <= swc_dbg.ib(1).ll_page(5 downto 0);
  TRIG5(5)(26)         <= swc_dbg.ob(1).data_error;
  TRIG5(5)(27)         <= swc_dbg.ob(1).mpm_dlast;
  TRIG5(5)(28)         <= swc_dbg.mpm.read.io(1).pre_fetch;
  TRIG5(5)(29)         <= swc_dbg.mpm.read.io(1).last_int;
  TRIG5(5)(30)         <= swc_dbg.mpm.read.io(1).rport_dreq;
  TRIG5(5)(31)         <= swc_dbg.mpm.read.io(1).rport_pg_req;

  TRIG6(5)(0)          <= swc_dbg.mpm.read.io(6).ll_req;
  TRIG6(5)(1)          <= swc_dbg.mpm.read.io(6).ll_grant;
  TRIG6(5)(7 downto 2) <= swc_dbg.mpm.read.io(6).ll_adr(5 downto 0);
  TRIG6(5)(13 downto 8)<= swc_dbg.mpm.read.io(6).ll_next_page(5 downto 0);
  TRIG6(5)(14)         <= swc_dbg.mpm.read.io(6).ll_eof;
  TRIG6(5)(15)         <= swc_dbg.mpm.read.io(6).ll_valid;
  TRIG6(5)(25 downto 16) <= swc_dbg.mpm.read.io(6).words_total;
  TRIG6(5)(26)         <= swc_dbg.ob(6).data_error;
  TRIG6(5)(27)         <= swc_dbg.ob(6).mpm_dlast;
  TRIG6(5)(28)         <= swc_dbg.mpm.read.io(6).pre_fetch;
  TRIG6(5)(29)         <= swc_dbg.mpm.read.io(6).last_int;
  TRIG6(5)(30)         <= swc_dbg.mpm.read.io(6).rport_dreq;
  TRIG6(5)(31)         <= swc_dbg.mpm.read.io(6).rport_pg_req;

  TRIG7(5)(0)          <= swc_dbg.mpm.read.io(7).ll_req;
  TRIG7(5)(1)          <= swc_dbg.mpm.read.io(7).ll_grant;
  TRIG7(5)(7 downto 2) <= swc_dbg.mpm.read.io(7).ll_adr(5 downto 0);
  TRIG7(5)(13 downto 8)<= swc_dbg.mpm.read.io(7).ll_next_page(5 downto 0);
  TRIG7(5)(14)         <= swc_dbg.mpm.read.io(7).ll_eof;
  TRIG7(5)(15)         <= swc_dbg.mpm.read.io(7).ll_valid;
  TRIG7(5)(25 downto 16) <= swc_dbg.mpm.read.io(7).words_total;
  TRIG7(5)(26)         <= swc_dbg.ob(7).data_error;
  TRIG7(5)(27)         <= swc_dbg.ob(7).mpm_dlast;
  TRIG7(5)(28)         <= swc_dbg.mpm.read.io(7).pre_fetch;
  TRIG7(5)(29)         <= swc_dbg.mpm.read.io(7).last_int;
  TRIG7(5)(30)         <= swc_dbg.mpm.read.io(7).rport_dreq;
  TRIG7(5)(31)         <= swc_dbg.mpm.read.io(7).rport_pg_req;

  TRIG8(5)(2 downto 0) <= swc_dbg.mpm.read.io(8).page_fsm;
  TRIG8(5)(5 downto 3) <= swc_dbg.mpm.read.io(1).page_fsm;
  TRIG8(5)(8 downto 6) <= swc_dbg.mpm.read.io(6).page_fsm;
  TRIG8(5)(11 downto 9)<= swc_dbg.mpm.read.io(7).page_fsm;
  TRIG8(5)(18 downto 12) <= swc_dbg.mpm.read.io(6).fetch_pg_words;
  TRIG8(5)(25 downto 19) <= swc_dbg.mpm.read.io(7).fetch_pg_words;

  TRIG9(5)(6 downto 0)   <= swc_dbg.mpm.read.io(1).ll_size;
  TRIG9(5)(13 downto 7)  <= swc_dbg.mpm.read.io(6).ll_size;
  TRIG9(5)(20 downto 14) <= swc_dbg.mpm.read.io(7).ll_size;
  TRIG9(5)(31 downto 22) <= swc_dbg.mpm.read.io(7).words_xmitted;

  TRIG10(5)(9 downto 0)   <= swc_dbg.mpm.read.io(1).words_xmitted;
  TRIG10(5)(19 downto 10) <= swc_dbg.mpm.read.io(1).words_total;
  TRIG10(5)(29 downto 20) <= swc_dbg.mpm.read.io(6).words_xmitted;

  ----------------------------------------------------------------
  TRIG0(6)(5 downto 0)   <= swc_dbg.mmu.palloc.free_pages(5 downto 0);
  TRIG0(6)(11 downto 6)  <= swc_dbg.ib(0).page_start(5 downto 0);
  TRIG0(6)(17 downto 12) <= swc_dbg.ib(0).page_inter(5 downto 0);
  TRIG0(6)(23 downto 18) <= swc_dbg.ib(1).page_start(5 downto 0);
  TRIG0(6)(29 downto 24) <= swc_dbg.ib(1).page_inter(5 downto 0);
  TRIG1(6)(5 downto 0)   <= swc_dbg.ib(6).page_start(5 downto 0);
  TRIG1(6)(11 downto 6)  <= swc_dbg.ib(6).page_inter(5 downto 0);
  TRIG1(6)(17 downto 12) <= swc_dbg.ib(7).page_start(5 downto 0);
  TRIG1(6)(23 downto 18) <= swc_dbg.ib(7).page_inter(5 downto 0);
  TRIG1(6)(24)           <= swc_dbg.ib(0).rcv_stuck;
  TRIG1(6)(25)           <= swc_dbg.ib(1).rcv_stuck;
  TRIG1(6)(26)           <= swc_dbg.ib(6).rcv_stuck;
  TRIG1(6)(27)           <= swc_dbg.ib(7).rcv_stuck;
  TRIG1(6)(28) <= swc_dbg.ib(0).pages_same;
  TRIG1(6)(29) <= swc_dbg.ib(1).pages_same;
  TRIG1(6)(30) <= swc_dbg.ib(6).pages_same;
  TRIG1(6)(31) <= swc_dbg.ib(7).pages_same;

  TRIG2(6)(0)            <= swc_dbg.free(0).free;
  TRIG2(6)(1)            <= swc_dbg.free(0).free_done;
  TRIG2(6)(2)            <= swc_dbg.free(0).ffree;
  TRIG2(6)(3)            <= swc_dbg.free(0).ffree_done;
  TRIG2(6)(9 downto 4)   <= swc_dbg.free(0).pgadr(5 downto 0);
  TRIG2(6)(10)           <= swc_dbg.free(1).free;
  TRIG2(6)(11)           <= swc_dbg.free(1).free_done;
  TRIG2(6)(12)           <= swc_dbg.free(1).ffree;
  TRIG2(6)(13)           <= swc_dbg.free(1).ffree_done;
  TRIG2(6)(19 downto 14) <= swc_dbg.free(1).pgadr(5 downto 0);
  TRIG2(6)(20)           <= swc_dbg.free(6).free;
  TRIG2(6)(21)           <= swc_dbg.free(6).free_done;
  TRIG2(6)(22)           <= swc_dbg.free(6).ffree;
  TRIG2(6)(23)           <= swc_dbg.free(6).ffree_done;
  TRIG2(6)(29 downto 24) <= swc_dbg.free(6).pgadr(5 downto 0);
  TRIG2(6)(31 downto 30) <= (others=>'0');

  TRIG3(6)(0)            <= swc_dbg.free(7).free;
  TRIG3(6)(1)            <= swc_dbg.free(7).free_done;
  TRIG3(6)(2)            <= swc_dbg.free(7).ffree;
  TRIG3(6)(3)            <= swc_dbg.free(7).ffree_done;
  TRIG3(6)(9 downto 4)   <= swc_dbg.free(7).pgadr(5 downto 0);
  TRIG3(6)(10)           <= swc_dbg.free(2).free;
  TRIG3(6)(11)           <= swc_dbg.free(2).free_done;
  TRIG3(6)(12)           <= swc_dbg.free(2).ffree;
  TRIG3(6)(13)           <= swc_dbg.free(2).ffree_done;
  TRIG3(6)(19 downto 14) <= swc_dbg.free(2).pgadr(5 downto 0);
  TRIG3(6)(20)           <= swc_dbg.free(3).free;
  TRIG3(6)(21)           <= swc_dbg.free(3).free_done;
  TRIG3(6)(22)           <= swc_dbg.free(3).ffree;
  TRIG3(6)(23)           <= swc_dbg.free(3).ffree_done;
  TRIG3(6)(29 downto 24) <= swc_dbg.free(3).pgadr(5 downto 0);
  TRIG3(6)(31 downto 30) <= (others=>'0');

  TRIG4(6)(0)            <= swc_dbg.free(4).free;
  TRIG4(6)(1)            <= swc_dbg.free(4).free_done;
  TRIG4(6)(2)            <= swc_dbg.free(4).ffree;
  TRIG4(6)(3)            <= swc_dbg.free(4).ffree_done;
  TRIG4(6)(9 downto 4)   <= swc_dbg.free(4).pgadr(5 downto 0);
  TRIG4(6)(10)           <= swc_dbg.free(5).free;
  TRIG4(6)(11)           <= swc_dbg.free(5).free_done;
  TRIG4(6)(12)           <= swc_dbg.free(5).ffree;
  TRIG4(6)(13)           <= swc_dbg.free(5).ffree_done;
  TRIG4(6)(19 downto 14) <= swc_dbg.free(5).pgadr(5 downto 0);
  TRIG4(6)(20)           <= swc_dbg.free(8).free;
  TRIG4(6)(21)           <= swc_dbg.free(8).free_done;
  TRIG4(6)(22)           <= swc_dbg.free(8).ffree;
  TRIG4(6)(23)           <= swc_dbg.free(8).ffree_done;
  TRIG4(6)(29 downto 24) <= swc_dbg.free(8).pgadr(5 downto 0);
  TRIG4(6)(31 downto 30) <= (others=>'0');

  TRIG5(6)(0)           <= swc_dbg.ib(0).pta_transfer;
  TRIG5(6)(1)           <= swc_dbg.ib(0).pta_transfer_ack;
  TRIG5(6)(7 downto 2)  <= swc_dbg.ib(0).pta_pgadr(5 downto 0);
  TRIG5(6)(15 downto 8) <= swc_dbg.ib(0).pta_mask;
  TRIG5(6)(16)          <= swc_dbg.ob(0).pta_transfer_valid;
  TRIG5(6)(17)          <= swc_dbg.ob(0).pta_ack;
  TRIG5(6)(23 downto 18)<= swc_dbg.ob(0).pta_pgadr(5 downto 0);
  TRIG5(6)(31 downto 24)<= (others=>'0');

  TRIG6(6)(0)           <= swc_dbg.ib(1).pta_transfer;
  TRIG6(6)(1)           <= swc_dbg.ib(1).pta_transfer_ack;
  TRIG6(6)(7 downto 2)  <= swc_dbg.ib(1).pta_pgadr(5 downto 0);
  TRIG6(6)(15 downto 8) <= swc_dbg.ib(1).pta_mask;
  TRIG6(6)(16)          <= swc_dbg.ob(1).pta_transfer_valid;
  TRIG6(6)(17)          <= swc_dbg.ob(1).pta_ack;
  TRIG6(6)(23 downto 18)<= swc_dbg.ob(1).pta_pgadr(5 downto 0);
  TRIG6(6)(31 downto 24)<= (others=>'0');

  TRIG7(6)(0)           <= swc_dbg.ib(6).pta_transfer;
  TRIG7(6)(1)           <= swc_dbg.ib(6).pta_transfer_ack;
  TRIG7(6)(7 downto 2)  <= swc_dbg.ib(6).pta_pgadr(5 downto 0);
  TRIG7(6)(15 downto 8) <= swc_dbg.ib(6).pta_mask;
  TRIG7(6)(16)          <= swc_dbg.ob(6).pta_transfer_valid;
  TRIG7(6)(17)          <= swc_dbg.ob(6).pta_ack;
  TRIG7(6)(23 downto 18)<= swc_dbg.ob(6).pta_pgadr(5 downto 0);
  TRIG7(6)(31 downto 24)<= (others=>'0');

  TRIG8(6)(0)           <= swc_dbg.ib(7).pta_transfer;
  TRIG8(6)(1)           <= swc_dbg.ib(7).pta_transfer_ack;
  TRIG8(6)(7 downto 2)  <= swc_dbg.ib(7).pta_pgadr(5 downto 0);
  TRIG8(6)(15 downto 8) <= swc_dbg.ib(7).pta_mask;
  TRIG8(6)(16)          <= swc_dbg.ob(7).pta_transfer_valid;
  TRIG8(6)(17)          <= swc_dbg.ob(7).pta_ack;
  TRIG8(6)(23 downto 18)<= swc_dbg.ob(7).pta_pgadr(5 downto 0);
  TRIG8(6)(31 downto 24)<= (others=>'0');
  
  TRIG9(6)(0)            <= swc_dbg.ob(2).pta_transfer_valid;
  TRIG9(6)(1)            <= swc_dbg.ob(2).pta_ack;
  TRIG9(6)(7 downto 2)   <= swc_dbg.ob(2).pta_pgadr(5 downto 0);
  TRIG9(6)(8)            <= swc_dbg.ob(3).pta_transfer_valid;
  TRIG9(6)(9)            <= swc_dbg.ob(3).pta_ack;
  TRIG9(6)(15 downto 10) <= swc_dbg.ob(3).pta_pgadr(5 downto 0);
  TRIG9(6)(16)           <= swc_dbg.ob(4).pta_transfer_valid;
  TRIG9(6)(17)           <= swc_dbg.ob(4).pta_ack;
  TRIG9(6)(23 downto 18) <= swc_dbg.ob(4).pta_pgadr(5 downto 0);
  TRIG9(6)(24)           <= swc_dbg.ob(5).pta_transfer_valid;
  TRIG9(6)(25)           <= swc_dbg.ob(5).pta_ack;
  TRIG9(6)(31 downto 26) <= swc_dbg.ob(5).pta_pgadr(5 downto 0);

  TRIG10(6)(0)          <= swc_dbg.ob(8).pta_transfer_valid;
  TRIG10(6)(1)          <= swc_dbg.ob(8).pta_ack;
  TRIG10(6)(7 downto 2) <= swc_dbg.ob(8).pta_pgadr(5 downto 0);
  TRIG10(6)(31 downto 8) <= (others=>'0');

  ---------------------------------------------------
  -- bank 7
  TRIG0(7)(5 downto 0) <= swc_dbg.mmu.palloc.free_pages(5 downto 0);
  TRIG0(7)(11 downto 6)  <= swc_dbg.ib(0).page_start(5 downto 0);
  TRIG0(7)(17 downto 12) <= swc_dbg.ib(0).page_inter(5 downto 0);
  TRIG0(7)(23 downto 18) <= swc_dbg.ib(1).page_start(5 downto 0);
  TRIG0(7)(29 downto 24) <= swc_dbg.ib(1).page_inter(5 downto 0);
  TRIG1(7)(5 downto 0)   <= swc_dbg.ib(6).page_start(5 downto 0);
  TRIG1(7)(11 downto 6)  <= swc_dbg.ib(6).page_inter(5 downto 0);
  TRIG1(7)(17 downto 12) <= swc_dbg.ib(7).page_start(5 downto 0);
  TRIG1(7)(23 downto 18) <= swc_dbg.ib(7).page_inter(5 downto 0);
  TRIG1(7)(24)           <= swc_dbg.ib(0).rcv_stuck;
  TRIG1(7)(25)           <= swc_dbg.ib(1).rcv_stuck;
  TRIG1(7)(26)           <= swc_dbg.ib(6).rcv_stuck;
  TRIG1(7)(27)           <= swc_dbg.ib(7).rcv_stuck;
  TRIG1(7)(28)           <= swc_dbg.ib(0).rtu_sof_bug;
  TRIG1(7)(29)           <= swc_dbg.ib(1).rtu_sof_bug;
  TRIG1(7)(30)           <= swc_dbg.ib(6).rtu_sof_bug;
  TRIG1(7)(31)           <= swc_dbg.ib(7).rtu_sof_bug;

  TRIG2(7)(2 downto 0)    <= swc_dbg.ib(0).alloc_fsm;
  TRIG2(7)(6 downto 3)    <= swc_dbg.ib(0).rcv_fsm;
  TRIG2(7)(10 downto 7)   <= swc_dbg.ib(0).trans_fsm;
  TRIG2(7)(14 downto 11)  <= swc_dbg.ib(0).ll_fsm;
  TRIG2(7)(15)            <= swc_dbg.ib(0).ll_wr;
  TRIG2(7)(16)            <= swc_dbg.ib(0).ll_wr_done;
  TRIG2(7)(17)            <= swc_dbg.ib(0).ll_page_valid;
  TRIG2(7)(18)            <= swc_dbg.ib(0).ll_eof;
  TRIG2(7)(24 downto  19) <= swc_dbg.ib(0).ll_page(5 downto 0);
  TRIG2(7)(30 downto 25)  <= swc_dbg.ib(0).ll_next_page(5 downto 0);
  TRIG2(7)(31)            <= swc_dbg.ib(0).mpm_dlast_d0;

  TRIG3(7)(2 downto 0)    <= swc_dbg.ib(1).alloc_fsm;
  TRIG3(7)(6 downto 3)    <= swc_dbg.ib(1).rcv_fsm;
  TRIG3(7)(10 downto 7)   <= swc_dbg.ib(1).trans_fsm;
  TRIG3(7)(14 downto 11)  <= swc_dbg.ib(1).ll_fsm;
  TRIG3(7)(15)            <= swc_dbg.ib(1).ll_wr;
  TRIG3(7)(16)            <= swc_dbg.ib(1).ll_wr_done;
  TRIG3(7)(17)            <= swc_dbg.ib(1).ll_page_valid;
  TRIG3(7)(18)            <= swc_dbg.ib(1).ll_eof;
  TRIG3(7)(24 downto  19) <= swc_dbg.ib(1).ll_page(5 downto 0);
  TRIG3(7)(30 downto 25)  <= swc_dbg.ib(1).ll_next_page(5 downto 0);
  TRIG3(7)(31)            <= swc_dbg.ib(1).mpm_dlast_d0;

  TRIG4(7)(2 downto 0)    <= swc_dbg.ib(6).alloc_fsm;
  TRIG4(7)(6 downto 3)    <= swc_dbg.ib(6).rcv_fsm;
  TRIG4(7)(10 downto 7)   <= swc_dbg.ib(6).trans_fsm;
  TRIG4(7)(14 downto 11)  <= swc_dbg.ib(6).ll_fsm;
  TRIG4(7)(15)            <= swc_dbg.ib(6).ll_wr;
  TRIG4(7)(16)            <= swc_dbg.ib(6).ll_wr_done;
  TRIG4(7)(17)            <= swc_dbg.ib(6).ll_page_valid;
  TRIG4(7)(18)            <= swc_dbg.ib(6).ll_eof;
  TRIG4(7)(24 downto  19) <= swc_dbg.ib(6).ll_page(5 downto 0);
  TRIG4(7)(30 downto 25)  <= swc_dbg.ib(6).ll_next_page(5 downto 0);
  TRIG4(7)(31)            <= swc_dbg.ib(6).mpm_dlast_d0;

  TRIG5(7)(2 downto 0)    <= swc_dbg.ib(7).alloc_fsm;
  TRIG5(7)(6 downto 3)    <= swc_dbg.ib(7).rcv_fsm;
  TRIG5(7)(10 downto 7)   <= swc_dbg.ib(7).trans_fsm;
  TRIG5(7)(14 downto 11)  <= swc_dbg.ib(7).ll_fsm;
  TRIG5(7)(15)            <= swc_dbg.ib(7).ll_wr;
  TRIG5(7)(16)            <= swc_dbg.ib(7).ll_wr_done;
  TRIG5(7)(17)            <= swc_dbg.ib(7).ll_page_valid;
  TRIG5(7)(18)            <= swc_dbg.ib(7).ll_eof;
  TRIG5(7)(24 downto  19) <= swc_dbg.ib(7).ll_page(5 downto 0);
  TRIG5(7)(30 downto 25)  <= swc_dbg.ib(7).ll_next_page(5 downto 0);
  TRIG5(7)(31)            <= swc_dbg.ib(7).mpm_dlast_d0;

  TRIG6(7)(7 downto 0)   <= swc_dbg.ib(0).rtu_rsp_cnt;
  TRIG6(7)(15 downto 8)  <= swc_dbg.ib(1).rtu_rsp_cnt;
  TRIG6(7)(23 downto 16) <= swc_dbg.ib(6).rtu_rsp_cnt;
  TRIG6(7)(31 downto 24) <= swc_dbg.ib(7).rtu_rsp_cnt;

  TRIG7(7)(7 downto 0)   <= swc_dbg.ib(0).sof_cnt;
  TRIG7(7)(15 downto 8)  <= swc_dbg.ib(1).sof_cnt;
  TRIG7(7)(23 downto 16) <= swc_dbg.ib(6).sof_cnt;
  TRIG7(7)(31 downto 24) <= swc_dbg.ib(7).sof_cnt;

  TRIG8(7)(7 downto 0)   <= std_logic_vector(dbg_rtu_cnt(0));
  TRIG8(7)(15 downto 8)  <= std_logic_vector(dbg_rtu_cnt(1));
  TRIG8(7)(23 downto 16) <= std_logic_vector(dbg_rtu_cnt(6));
  TRIG8(7)(31 downto 24) <= std_logic_vector(dbg_rtu_cnt(7));

  TRIG9(7)(7 downto 0)   <= std_logic_vector(dbg_rtu_bug(0));
  TRIG9(7)(15 downto 8)  <= std_logic_vector(dbg_rtu_bug(1));
  TRIG9(7)(23 downto 16) <= std_logic_vector(dbg_rtu_bug(6));
  TRIG9(7)(31 downto 24) <= std_logic_vector(dbg_rtu_bug(7));

  TRIG10(7)(0) <= swc_dbg.ib(0).rtu_valid;
  TRIG10(7)(1) <= swc_dbg.ib(0).rtu_ack;
  TRIG10(7)(2) <= swc_dbg.ib(1).rtu_valid;
  TRIG10(7)(3) <= swc_dbg.ib(1).rtu_ack;
  TRIG10(7)(4) <= swc_dbg.ib(6).rtu_valid;
  TRIG10(7)(5) <= swc_dbg.ib(6).rtu_ack;
  TRIG10(7)(6) <= swc_dbg.ib(7).rtu_valid;
  TRIG10(7)(7) <= swc_dbg.ib(7).rtu_ack;

  TRIG11(7)(7 downto 0)   <= swc_dbg.ib(0).dbg_bare_sof;
  TRIG11(7)(15 downto 8)  <= swc_dbg.ib(1).dbg_bare_sof;
  TRIG11(7)(23 downto 16) <= swc_dbg.ib(6).dbg_bare_sof;
  TRIG11(7)(31 downto 24) <= swc_dbg.ib(7).dbg_bare_sof;

--  ----------------------------- dbg_id0
--  TRIG0(0)(15    downto   0) <= phys_i(0).rx_data;            
--  TRIG0(0)(17    downto  16) <= phys_i(0).rx_k;
--  TRIG0(0)(              18) <= phys_i(0).rx_enc_err;
--  TRIG0(0)(20    downto  19) <= ep_dbg_k_array(7);
--
--  TRIG0(0)(              24) <= endpoint_src_out(0).cyc;
--  TRIG0(0)(              25) <= endpoint_src_out(0).stb;
--  TRIG0(0)(              26) <= endpoint_src_in(0).stall;
--  TRIG0(0)(              27) <= endpoint_src_in(0).err;
--  TRIG0(0)(              28) <= endpoint_src_in(0).ack;
--  TRIG0(0)(              29) <= endpoint_snk_in(7).cyc;
--  TRIG0(0)(              30) <= endpoint_snk_in(7).stb;
--  TRIG0(0)(              31) <= endpoint_snk_out(7).stall;
--
--
--  TRIG1(0)(29    downto   0) <= ep_dbg_fab_pipes_array(0)(29 downto 0); -- rx_path
--  TRIG1(0)(              30) <= endpoint_snk_out(7).ack;
--  TRIG1(0)(              31) <= endpoint_snk_out(7).err;
--
--  TRIG2(0)(11    downto   0) <= ep_dbg_fab_pipes_array(7)(41 downto 30); -- tx_path
--  TRIG2(0)(29    downto  20) <= dbg_n_regs(41 downto 32) ; -- unknow resources
--  TRIG2(0)(              30) <= phys_i(7).tx_enc_err;
--  TRIG2(0)(              31) <= phys_i(7).tx_disparity;
--
--
--  TRIG3(0)(15    downto   0) <= ep_dbg_data_array(7);
--  TRIG3(0)(23    downto  16) <= endpoint_snk_in(7).dat(7 downto 0);
--  
--  gen_18P_out_blk_states: if(g_num_ports = 18 ) generate 
--    TRIG3(0)(31    downto  24) <= dbg_n_regs(431 downto 424); --p7   for 18 ports: should be states of output block i SWcore
--    TRIG3(1)(31    downto  24) <= dbg_n_regs(431 downto 424); --p7   for 18 ports: should be states of output block i SWcore
--    TRIG3(2)(31    downto  24) <= dbg_n_regs(503 downto 496); --p16  for 18 ports: should be states of output block i SWcore
--    TRIG3(3)(31    downto  24) <= dbg_n_regs(511 downto 504); --p17 for 18 ports: should be states of output block i SWcore
--    TRIG3(4)(31    downto  24) <= dbg_n_regs(375 downto 368); --p0  for 18 ports: should be states of output block i SWcore
--    TRIG3(5)(31    downto  24) <= dbg_n_regs(383 downto 376); --p1 for 18 ports: should be states of output block i SWcore
--  end generate gen_18P_out_blk_states;
--  gen_8P_out_blk_states: if(g_num_ports = 8 ) generate 
--    TRIG3(0)(31    downto  24) <= dbg_n_regs(271 downto 264); -- for 8 ports: should be states of output block i SWcore
--    TRIG3(1)(31    downto  24) <= dbg_n_regs(271 downto 264); -- for 8 ports: should be states of output block i SWcore
--    TRIG3(4)(31    downto  24) <= dbg_n_regs(215 downto 208); --p0  for 18 ports: should be states of output block i SWcore
--    TRIG3(5)(31    downto  24) <= dbg_n_regs(223 downto 216); --p1 for 18 ports: should be states of output block i SWcore
--
--  end generate gen_8P_out_blk_states;
--
--  ----------------------------- dbg_id1
--  TRIG0(1)(15    downto   0) <= endpoint_snk_in(7).dat;             -- 0 -15
--  TRIG0(1)(17    downto  16) <= endpoint_snk_in(7).adr(1 downto 0); -- 16-17
--  TRIG0(1)(              18) <= endpoint_snk_out(7).ack;            -- 17
----   TRIG0(1)(20    downto  19) <= ep_dbg_k_array(7);
--  TRIG0(1)(28    downto  19) <= ep_dbg_tx_pcs_rd_array(7);-- pcs new
----   TRIG0(1)(              28) <= endpoint_snk_out(7).err;
--  TRIG0(1)(              29) <= endpoint_snk_in(7).cyc;             -- 29
--  TRIG0(1)(              30) <= endpoint_snk_in(7).stb;             -- 30
--  TRIG0(1)(              31) <= endpoint_snk_out(7).stall;          -- 31
--
--  TRIG1(1)(21    downto   0) <= ep_dbg_fab_pipes_array(7)(63 downto 42); -- tx_path: 32 - 53
--  TRIG1(1)(31    downto  22) <= ep_dbg_tx_pcs_wr_array(7);     -- pcs new: pcs tx write to FIFO  
--
--  TRIG2(1)(11    downto   0) <= ep_dbg_fab_pipes_array(7)(41 downto 30); -- tx_path : 64- 75
--  TRIG2(1)(29    downto  20) <= dbg_n_regs(41 downto 32) ; -- unknow resources
--  TRIG2(1)(              30) <= phys_i(7).tx_enc_err;
--  TRIG2(1)(              31) <= endpoint_snk_out(7).err;--phys_i(7).tx_disparity;
--
--  TRIG3(1)(15    downto   0) <= ep_dbg_data_array(7);
--  
--  gen_18P_chip: if(g_num_ports = 18 ) generate
--  ----------------------------- dbg_id2
--    TRIG0(2)(15    downto   0) <= endpoint_snk_in(16).dat;
--    TRIG0(2)(17    downto  16) <= endpoint_snk_in(16).adr(1 downto 0);
--    TRIG0(2)(              18) <= endpoint_snk_out(16).ack;
--    --TRIG0(2)(20    downto  19) <= ep_dbg_k_array(16);
--    TRIG0(2)(28    downto  19) <= ep_dbg_tx_pcs_rd_array(16); -- pcs new
----     TRIG0(2)(              28) <= endpoint_snk_out(16).err;
--    TRIG0(2)(              29) <= endpoint_snk_in(16).cyc;
--    TRIG0(2)(              30) <= endpoint_snk_in(16).stb;
--    TRIG0(2)(              31) <= endpoint_snk_out(16).stall;
--
--    TRIG1(2)(21    downto   0) <= ep_dbg_fab_pipes_array(16)(63 downto 42);
--    TRIG1(2)(31    downto  22) <= ep_dbg_tx_pcs_wr_array(16);     -- pcs new: pcs tx write to FIFO
--   
--    TRIG2(2)(11    downto   0) <= ep_dbg_fab_pipes_array(16)(41 downto 30); -- tx_path
--    TRIG2(2)(29    downto  20) <= dbg_n_regs(41 downto 32) ; -- unknow resources
--    TRIG2(2)(              30) <= phys_i(16).tx_enc_err;
--    TRIG2(2)(              31) <= endpoint_snk_out(16).err;
--
--    TRIG3(2)(15    downto   0) <= ep_dbg_data_array(16);
--
--  ----------------------------- dbg_id3
--    TRIG0(3)(15    downto   0) <= endpoint_snk_in(17).dat;
--    TRIG0(3)(17    downto  16) <= endpoint_snk_in(17).adr(1 downto 0);
--    TRIG0(3)(              18) <= endpoint_snk_out(17).ack;
----     TRIG0(3)(20    downto  19) <= ep_dbg_k_array(17);
--    TRIG0(3)(28    downto  19) <= ep_dbg_tx_pcs_rd_array(17); -- pcs new
----     TRIG0(3)(              28) <= endpoint_snk_out(17).err;
--    TRIG0(3)(              29) <= endpoint_snk_in(17).cyc;
--    TRIG0(3)(              30) <= endpoint_snk_in(17).stb;
--    TRIG0(3)(              31) <= endpoint_snk_out(17).stall;
--
--    TRIG1(3)(21    downto   0) <= ep_dbg_fab_pipes_array(17)(63 downto 42);
--    TRIG1(3)(31    downto  22) <= ep_dbg_tx_pcs_wr_array(17);     -- pcs new: pcs tx write to FIFO
--  
--    TRIG2(3)(11    downto   0) <= ep_dbg_fab_pipes_array(17)(41 downto 30); -- tx_path
--    TRIG2(3)(29    downto  20) <= dbg_n_regs(41 downto 32) ; -- unknow resources
--    TRIG2(3)(              30) <= phys_i(17).tx_enc_err;
--    TRIG2(3)(              31) <= endpoint_snk_out(17).err;--phys_i(17).tx_disparity;
--
--    TRIG3(3)(15    downto   0) <= ep_dbg_data_array(17);
--
--  ----------------------------- dbg_id4
--    TRIG0(4)(15    downto   0) <= endpoint_snk_in(0).dat;
--    TRIG0(4)(17    downto  16) <= endpoint_snk_in(0).adr(1 downto 0);
--    TRIG0(4)(              18) <= endpoint_snk_out(0).ack;
----     TRIG0(4)(20    downto  19) <= ep_dbg_k_array(0);
--    TRIG0(4)(28    downto  19) <= ep_dbg_tx_pcs_rd_array(0); -- pcs new
----     TRIG0(4)(              28) <= endpoint_snk_out(0).err;
--    TRIG0(4)(              29) <= endpoint_snk_in(0).cyc;
--    TRIG0(4)(              30) <= endpoint_snk_in(0).stb;
--    TRIG0(4)(              31) <= endpoint_snk_out(0).stall;
--
--    TRIG1(4)(21    downto   0) <= ep_dbg_fab_pipes_array(0)(63 downto 42);
--    TRIG1(4)(31    downto  22) <= ep_dbg_tx_pcs_wr_array(0);     -- pcs new: pcs tx write to FIFO
--  
--    TRIG2(4)(11    downto   0) <= ep_dbg_fab_pipes_array(0)(41 downto 30); -- tx_path
--    TRIG2(4)(29    downto  20) <= dbg_n_regs(41 downto 32) ; -- unknow resources
--    TRIG2(4)(              30) <= phys_i(0).tx_enc_err;
--    TRIG2(4)(              31) <= endpoint_snk_out(0).err;--phys_i(0).tx_disparity;
--
--    TRIG3(4)(15    downto   0) <= ep_dbg_data_array(0);
--
--  ----------------------------- dbg_id5
--    TRIG0(5)(15    downto   0) <= endpoint_snk_in(1).dat;
--    TRIG0(5)(17    downto  16) <= endpoint_snk_in(1).adr(1 downto 0);
--    TRIG0(5)(              18) <= endpoint_snk_out(1).ack;
----     TRIG0(5)(20    downto  19) <= ep_dbg_k_array(1);
--    TRIG0(5)(28    downto  19) <= ep_dbg_tx_pcs_rd_array(1); -- pcs new
----     TRIG0(5)(              28) <= endpoint_snk_out(1).err;
--    TRIG0(5)(              29) <= endpoint_snk_in(1).cyc;
--    TRIG0(5)(              30) <= endpoint_snk_in(1).stb;
--    TRIG0(5)(              31) <= endpoint_snk_out(1).stall;
--
--    TRIG1(5)(21    downto   0) <= ep_dbg_fab_pipes_array(1)(63 downto 42);
--    TRIG1(5)(31    downto  22) <= ep_dbg_tx_pcs_wr_array(1);     -- pcs new: pcs tx write to FIFO
--  
--    TRIG2(5)(11    downto   0) <= ep_dbg_fab_pipes_array(1)(41 downto 30); -- tx_path
--    TRIG2(5)(29    downto  20) <= dbg_n_regs(41 downto 32) ; -- unknow resources
--    TRIG2(5)(              30) <= phys_i(1).tx_enc_err;
--    TRIG2(5)(              31) <= endpoint_snk_out(1).err;--phys_i(1).tx_disparity;
--
--    TRIG3(5)(15    downto   0) <= ep_dbg_data_array(1);
--
--  end generate gen_18P_chip;
  
end rtl;
