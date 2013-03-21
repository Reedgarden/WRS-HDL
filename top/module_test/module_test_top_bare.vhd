-------------------------------------------------------------------------------
-- Title      : Resource Test and Optimization module top entity
-- Project    : WhiteRabbit switch
-------------------------------------------------------------------------------
-- File       : module_test_top_bare.vhd
-- Author     : Maciej Lipinski
-- Company    : CERN BE-Co-HT
-- Created    : 2013-03-19
-- Last update: 2013-03-19
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
-------------------------------------------------------------------------------

library ieee;
use ieee.STD_LOGIC_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.CEIL;
use ieee.math_real.log2;
use work.swc_swcore_pkg.all;

library UNISIM;
use UNISIM.vcomponents.all;

entity module_test_top_bare is
  generic(
    g_num_ports     : integer := 6;
    g_top_in_bits   : integer := 6;
    g_top_out_bits  : integer := 6;
      g_module_name : string  := ""
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

  
  constant c_wb_data_width           : integer := 16;
  constant c_wb_addr_width           : integer := 2;
  constant c_wb_sel_width            : integer := 2;
  constant c_prio_num                : integer := 8;
  constant c_num_global_pause        : integer := 2;
  constant c_prio_width              : integer := integer(CEIL(LOG2(real(c_prio_num-1)))) ;

  constant c_swcore_input_bits       : integer := g_num_ports*(
                                                  c_wb_data_width+ -- snk_data
                                                  c_wb_addr_width+ -- snk_addr
                                                  c_wb_sel_width + -- snk_sel
                                                  1              + -- snk_cyc
                                                  1              + -- snk_stb
                                                  1              + -- snk_we
                                                  1              + -- src_stall
                                                  1              + -- src_ack 
                                                  1              + -- src_err
                                                  1              + -- rtu_rsp_valid
                                                  g_num_ports    + -- rtu_dst_mask
                                                  1              + -- rtu_drop
                                                  c_prio_width   + -- rtu_prio
                                                  1              + -- pp_req
                                                  16             + -- pp_quanta
                                                  8             )+ -- pp_class
                                                  c_num_global_pause*(
                                                  1              + -- gp_req
                                                  16             + -- gp_quanta
                                                  8              + -- gp_class
                                                  g_num_ports   )+ -- gp_ports
                                                  1              ; -- shaper_drop
  constant c_swcore_output_bits      : integer := g_num_ports*(
                                                  c_wb_data_width+ -- src_data
                                                  c_wb_addr_width+ -- src_addr
                                                  c_wb_sel_width + -- src_sel
                                                  1              + -- src_cyc
                                                  1              + -- src_stb
                                                  1              + -- src_we
                                                  1              + -- snk_stall
                                                  1              + -- snk_ack 
                                                  1              + -- snk_err
                                                  1              + -- snk_rty
                                                  1             )+ -- rtu_rsp_ack
                                                  8              ; -- dbg
                                                  

  signal swcore_input_vector  : std_logic_vector(c_swcore_input_bits -1 downto 0);
  signal swcore_output_vector : std_logic_vector(c_swcore_output_bits-1 downto 0);

  signal sel_clk_sys, sel_clk_sys_int : std_logic;
begin

  
   SWcore: swc_core_vectorized_top 
   generic map( 
     g_num_ports               => g_num_ports,
     g_in_bits                 => c_swcore_input_bits,
     g_out_bits                => c_swcore_output_bits,
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
     input_vector_i            => swcore_input_vector,
     output_vector_o           => swcore_output_vector
    );

   FAKE_IN: fake_in_out
    generic map(
      g_in_bits                => g_top_in_bits,
      g_out_bits               => c_swcore_input_bits
      )
    port map(
      rst_n_i                  => sys_rst_n_i,
      clk_i                    => clk_ref_i,
      input_vector_i           => input_vector_i,
      output_vector_o          => swcore_input_vector
      );

   FAKE_OUT: fake_in_out
    generic map(
      g_in_bits                => c_swcore_output_bits,
      g_out_bits               => g_top_out_bits
      )
    port map(
      rst_n_i                  => sys_rst_n_i,
      clk_i                    => clk_ref_i,
      input_vector_i           => swcore_output_vector,
      output_vector_o          => output_vector_o
      );

end rtl;
