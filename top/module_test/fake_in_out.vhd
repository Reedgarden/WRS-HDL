-------------------------------------------------------------------------------
-- Title      : Simple shift register to fake inputs/outputs
-- Project    : WhiteRabbit switch
-------------------------------------------------------------------------------
-- File       : fake_in_out.vhd
-- Author     : Maciej Lipinski
-- Company    : CERN BE-Co-HT
-- Created    : 2013-03-19
-- Last update: 2013-03-19
-- Platform   : FPGA-generic
-- Standard   : VHDL'87
-------------------------------------------------------------------------------
-- Description: 
-- simple shift register which enables to connect vector of any widht to vector
-- of any width. This is used to prevent synthesis tool from optimization of
-- a module which we want to test for resources utilization
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


entity fake_in_out is
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
end fake_in_out;

architecture rtl of fake_in_out is

  signal output_vector : std_logic_vector(g_in_bits+g_out_bits - 1 downto 0);

begin 

  SHIFT_REG: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (rst_n_i = '0') then
        output_vector <= (others=>'0');
      else 
        output_vector(0) <= input_vector_i(0);

        for i in 1 to (g_in_bits+g_out_bits-1) loop
          if(i < g_in_bits) then
            output_vector(i) <= output_vector(i-1) xor input_vector_i(i);
          else
            output_vector(i) <= output_vector(i-1);
          end if;
        end loop;
      end if;
     end if;
  end process;

  output_vector_o <= output_vector(g_in_bits+g_out_bits - 1 downto g_in_bits);
  
end rtl;
