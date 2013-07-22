-------------------------------------------------------------------------------
-- Title      : Resource Test and Optimization synthesis top  
-- Project    : WhiteRabbit switch
-------------------------------------------------------------------------------
-- File       : scb_top_synthesis.vhd
-- Author     : Maciej Lipinski
-- Company    : CERN BE-Co-HT
-- Created    : 2013-03-19
-- Last update: 2013-03-19
-- Platform   : FPGA-generic
-- Standard   : VHDL'87
-------------------------------------------------------------------------------
-- Description: 
-- Top entity which holds single-module tester - to check resource
-- utilization per-module. makes it easier to optimize resources.
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
-- 
-------------------------------------------------------------------------------
--
-- Copyright (c) 2013 Maciej Lipinski / CERN
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
-- 2013-03-19  1.0      mlipinsk Created

-------------------------------------------------------------------------------

library ieee;
use ieee.STD_LOGIC_1164.all;
use ieee.numeric_std.all;

use work.wishbone_pkg.all;
use work.gencores_pkg.all;
use work.wr_fabric_pkg.all;
use work.endpoint_pkg.all;
use work.wrsw_txtsu_pkg.all;


library UNISIM;
use UNISIM.vcomponents.all;


entity scb_top_synthesis is
  generic(
    g_cpu_addr_width : integer := 19;
    g_simulation     : boolean := false
    );
  port (
    sys_rst_n_i : in std_logic;         -- global reset

    -- Startup 25 MHz clock (from onboard 25 MHz oscillator)
    fpga_clk_25mhz_p_i : in std_logic;
    fpga_clk_25mhz_n_i : in std_logic;

    -- 125 MHz timing reference (from the AD9516 PLL output QDRII_CLK)
    fpga_clk_ref_p_i : in std_logic;
    fpga_clk_ref_n_i : in std_logic;

    -- 125+ MHz DMTD offset clock (from the CDCM62001 PLL output DMTDCLK_MAIN)
    fpga_clk_dmtd_p_i : in std_logic;
    fpga_clk_dmtd_n_i : in std_logic;

    -- 250/10 MHz aux clock for Swcore/rephasing AD9516 in master mode
    -- (from the AD9516 PLL output QDRII_200CLK)
    fpga_clk_aux_p_i : in std_logic;
    fpga_clk_aux_n_i : in std_logic;

    -------------------------------------------------------------------------------
    -- Atmel EBI bus
    -------------------------------------------------------------------------------
    cpu_clk_i   : in    std_logic;      -- clock (not used now)
    -- async chip select, active LOW
    cpu_cs_n_i  : in    std_logic;
    -- async write, active LOW
    cpu_wr_n_i  : in    std_logic;
    -- async read, active LOW
    cpu_rd_n_i  : in    std_logic;
    -- byte select, active  LOW (not used due to weird CPU pin layout - NBS2 line is
    -- shared with 100 Mbps Ethernet PHY)
    cpu_bs_n_i  : in    std_logic_vector(3 downto 0);
    -- address input
    cpu_addr_i  : in    std_logic_vector(g_cpu_addr_width-1 downto 0);
    -- data bus (bidirectional)
    cpu_data_b  : inout std_logic_vector(31 downto 0);
    -- async wait, active LOW
    cpu_nwait_o : out   std_logic;

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
    dac_main_sclk_o   : out std_logic;
    dac_main_data_o   : out std_logic;


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
    -- Clock fanout control
    -------------------------------------------------------------------------------
    clk_en_o  : out std_logic;
    clk_sel_o : out std_logic;


    -- DMTD clock divider selection (0 = 125 MHz, 1 = 62.5 MHz)
    clk_dmtd_divsel_o : out std_logic;

    -- UART source selection (FPGA/DBGU)
    uart_sel_o : out std_logic;

    ---------------------------------------------------------------------------
    -- GTX ports
    ---------------------------------------------------------------------------

    gtx0_3_clk_n_i : in std_logic;
    gtx0_3_clk_p_i : in std_logic;

    gtx4_7_clk_n_i : in std_logic;
    gtx4_7_clk_p_i : in std_logic;

    gtx8_11_clk_n_i : in std_logic;
    gtx8_11_clk_p_i : in std_logic;

    gtx12_15_clk_n_i : in std_logic;
    gtx12_15_clk_p_i : in std_logic;

    gtx16_19_clk_n_i : in std_logic;
    gtx16_19_clk_p_i : in std_logic;

    gtx_rxp_i : in std_logic_vector(17 downto 0);
    gtx_rxn_i : in std_logic_vector(17 downto 0);

    gtx_txp_o : out std_logic_vector(17 downto 0);
    gtx_txn_o : out std_logic_vector(17 downto 0);

    ---------------------------------------------------------------------------
    -- Mini-Backplane signals
    ---------------------------------------------------------------------------

    led_act_o : out std_logic_vector(17 downto 0);

    mbl_scl_b : inout std_logic_vector(1 downto 0);
    mbl_sda_b : inout std_logic_vector(1 downto 0);

    sensors_scl_b: inout std_logic;
    sensors_sda_b: inout std_logic
  );

end scb_top_synthesis;

architecture Behavioral of scb_top_synthesis is

  component IBUFGDS
    generic (
      DIFF_TERM  : boolean := true;
      IOSTANDARD : string  := "DEFAULT")  ;
    port (
      O  : out std_ulogic;
      I  : in  std_ulogic;
      IB : in  std_ulogic);
  end component;

  component BUFGMUX
    generic (
      CLK_SEL_TYPE : string := "SYNC");
    port (
      O  : out std_ulogic := '0';
      I0 : in  std_ulogic := '0';
      I1 : in  std_ulogic := '0';
      S  : in  std_ulogic := '0');
  end component;


  component module_test_top_bare is
    generic(
      g_top_in_bits     : integer := 6;
      g_top_out_bits    : integer := 6;
      g_module_name     : string  := ""
      );
    port (
      sys_rst_n_i     : in std_logic;        
      clk_startup_i   : in std_logic;
      clk_ref_i       : in std_logic;
      clk_dmtd_i      : in std_logic;
      clk_aux_i       : in std_logic;

      input_vector_i  : in std_logic_vector(g_top_in_bits   - 1 downto 0);
      output_vector_o : out std_logic_vector(g_top_out_bits - 1 downto 0)
     );
  end component;

  constant c_top_in_bits  : integer := 4                + -- cpu bits
                                       g_cpu_addr_width + -- cpu addr
                                       4                + -- cpu sel
                                       1                + -- pps
                                       2                + -- pll
                                       1                ; -- uart
                                      
                                      
  constant c_top_out_bits : integer := 2                + -- cpu bits
                                       1                + -- pps
                                       6                + -- dac
                                        5                + -- pll
                                       1                + -- uart
                                       4                + -- clock fanout ctr
                                       18               ; -- leds

  signal input_vector     : std_logic_vector(c_top_in_bits-1 downto 0);
  signal output_vector    : std_logic_vector(c_top_out_bits-1 downto 0);

  signal clk_sys, clk_ref, clk_25mhz, clk_aux , clk_dmtd : std_logic;
begin

  U_Buf_CLK_Startup : IBUFGDS
    generic map (
      DIFF_TERM  => true,
      IOSTANDARD => "LVDS_25")
    port map (
      O  => clk_25mhz,
      I  => fpga_clk_25mhz_p_i,
      IB => fpga_clk_25mhz_n_i);

  U_Buf_CLK_Ref : IBUFGDS
    generic map (
      DIFF_TERM  => true,
      IOSTANDARD => "LVDS_25")
    port map (
      O  => clk_ref,
      I  => fpga_clk_ref_p_i,
      IB => fpga_clk_ref_n_i);

  U_Buf_CLK_Sys : IBUFGDS
    generic map (
      DIFF_TERM  => true,
      IOSTANDARD => "LVDS_25")
    port map (
      O  => clk_aux,
      I  => fpga_clk_aux_p_i,
      IB => fpga_clk_aux_n_i);


  U_Buf_CLK_DMTD : IBUFGDS
    generic map (
      DIFF_TERM  => true,
      IOSTANDARD => "LVDS_25")
    port map (
      O  => clk_dmtd,
      I  => fpga_clk_dmtd_p_i,
      IB => fpga_clk_dmtd_n_i);

  TEST: module_test_top_bare
    generic map(
      g_top_in_bits     => c_top_in_bits,
      g_top_out_bits    => c_top_out_bits,
      g_module_name     => "Pstats"
      )
    port map(
      sys_rst_n_i     => sys_rst_n_i,
      clk_startup_i   => clk_ref,
      clk_ref_i       => clk_ref,
      clk_dmtd_i      => clk_dmtd,
      clk_aux_i       => clk_aux,

      input_vector_i  => input_vector,
      output_vector_o => output_vector
     );

 input_vector         <= cpu_clk_i        &
                         cpu_cs_n_i       &
                         cpu_wr_n_i       &
                         cpu_rd_n_i       &
                         cpu_bs_n_i       &
                         cpu_addr_i       &
                         pps_i            &
                         pll_status_i     &
                         pll_miso_i       &
                         uart_rxd_i       ;
    
  cpu_nwait_o         <= output_vector(0);
  cpu_irq_n_o         <= output_vector(1);
  pps_o               <= output_vector(2);
  dac_helper_sync_n_o <= output_vector(3);
  dac_helper_sclk_o   <= output_vector(4);
  dac_helper_data_o   <= output_vector(5);
  dac_main_sync_n_o   <= output_vector(6);
  dac_main_sclk_o     <= output_vector(7);
  dac_main_data_o     <= output_vector(8);
  pll_mosi_o          <= output_vector(9);
  pll_sck_o           <= output_vector(10);
  pll_cs_n_o          <= output_vector(11);
  pll_sync_n_o        <= output_vector(12);
  pll_reset_n_o       <= output_vector(13);
  uart_txd_o          <= output_vector(14);
  clk_en_o            <= output_vector(15);
  clk_sel_o           <= output_vector(16);
  clk_dmtd_divsel_o   <= output_vector(17);
  uart_sel_o          <= output_vector(18);
  led_act_o           <= output_vector(19+18-1 downto 19);

end Behavioral;


