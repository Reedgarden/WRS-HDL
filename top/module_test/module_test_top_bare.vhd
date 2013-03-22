-------------------------------------------------------------------------------
-- Title      : Resource Test and Optimization module top entity
-- Project    : WhiteRabbit switch
-------------------------------------------------------------------------------
-- File       : module_test_top_bare.vhd
-- Author     : Maciej Lipinski
-- Company    : CERN BE-Co-HT
-- Created    : 2013-03-19
-- Last update: 2013-03-22
-- Platform   : FPGA-generic
-- Standard   : VHDL'87
-------------------------------------------------------------------------------
-- Description: 
-- This entity holds different modules (DUTs) which are connected to 
-- input and outputs (to prevent synthesiz optimization). One module
-- shall be chosen for synthesis/simulation by generic. This module is
-- like a dummy wrapper which can bind input/output vector (connected in
-- "topper" entity to FPGA's I/O) of any width to  inputs/outputs of 
-- the DUT. This is done through fake_in_out module
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
-- 2013-03-22  2.0      greg.d   Expanded to be more generic
-------------------------------------------------------------------------------

library ieee;
use ieee.STD_LOGIC_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.CEIL;
use ieee.math_real.log2;
use work.swc_swcore_pkg.all;
use work.test_top_pkg.all;

library UNISIM;
use UNISIM.vcomponents.all;

entity module_test_top_bare is
  generic(
    g_top_in_bits   : integer := 6;
    g_top_out_bits  : integer := 6;
    g_module_name   : string  := ""
    );
  port (
    sys_rst_n_i     : in std_logic;         -- global reset
    clk_startup_i   : in std_logic;
    clk_ref_i       : in std_logic;
    clk_dmtd_i      : in std_logic;
    clk_aux_i       : in std_logic;

    input_vector_i  : in std_logic_vector(g_top_in_bits   - 1 downto 0);
    output_vector_o : out std_logic_vector(g_top_out_bits - 1 downto 0)
    );
end module_test_top_bare;

architecture rtl of module_test_top_bare is

  component fake_in_out
    generic(
      g_in_bits         : integer := 6;
      g_out_bits        : integer := 6
      );
    port (
      rst_n_i : in std_logic;         -- global reset
      clk_i     : in std_logic;

      input_vector_i  : in std_logic_vector(g_in_bits   - 1 downto 0);
      output_vector_o : out std_logic_vector(g_out_bits - 1 downto 0)
      );
    end component;
  
  signal DUT_in_vector  : std_logic_vector(f_invec_len(g_module_name) -1 downto 0);
  signal DUT_out_vector : std_logic_vector(f_outvec_len(g_module_name)-1 downto 0);

  signal sel_clk_sys, sel_clk_sys_int : std_logic;
begin

  --===================================================--
  --                   SW Core                         --
  --===================================================--
  GEN_SWcore: if g_module_name = "SWcore" generate
   DUT_SWC: swc_core_vectorized_top 
   generic map( 
     g_num_ports               => c_num_ports,
     g_in_bits                 => f_invec_len("SWcore"),  --c_swcore_input_bits,
     g_out_bits                => f_outvec_len("SWcore"), --c_swcore_output_bits,
     g_wb_data_width           => c_wb_data_width,
     g_wb_addr_width           => c_wb_addr_width,
     g_wb_sel_width            => c_wb_sel_width,
     g_prio_num                => c_prio_num,
     g_num_global_pause        => c_num_global_pause,
     g_prio_width              => c_prio_width
    )
   port map(
     clk_i                     => clk_ref_i,
     clk_mpm_core_i            => clk_aux_i,
     rst_n_i                   => sys_rst_n_i,
     input_vector_i            => DUT_in_vector, --swcore_input_vector,
     output_vector_o           => DUT_out_vector --swcore_output_vector
    );
 end generate;

   FAKE_IN: fake_in_out
    generic map(
      g_in_bits                => g_top_in_bits,
      g_out_bits               => f_invec_len(g_module_name) --c_swcore_input_bits
      )
    port map(
      rst_n_i                  => sys_rst_n_i,
      clk_i                    => clk_ref_i,
      input_vector_i           => input_vector_i,
      output_vector_o          => DUT_in_vector --swcore_input_vector
      );

   FAKE_OUT: fake_in_out
    generic map(
      g_in_bits                => f_outvec_len(g_module_name), --c_swcore_output_bits,
      g_out_bits               => g_top_out_bits
      )
    port map(
      rst_n_i                  => sys_rst_n_i,
      clk_i                    => clk_ref_i,
      input_vector_i           => DUT_out_vector, --swcore_output_vector,
      output_vector_o          => output_vector_o
      );

end rtl;
