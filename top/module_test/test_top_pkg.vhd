-------------------------------------------------------------------------------
-- Title      : Resource Test and Optimization module package
-- Project    : WhiteRabbit switch
-------------------------------------------------------------------------------
-- File       : test_top_pkg.vhd
-- Author     : Grzegorz Daniluk
-- Company    : CERN BE-CO-HT
-- Created    : 2013-03-22
-- Last update: 2013-03-22
-- Platform   : FPGA-generic
-- Standard   : VHDL'87
-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
--
-- Copyright (c) 2013 Grzegorz Daniluk / CERN
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
use ieee.std_logic_1164.all;
use ieee.math_real.CEIL;
use ieee.math_real.log2;

package test_top_pkg is

  type t_strarr is array(natural range <>) of string(1 to 6);
  type t_lentype is array(natural range <>) of integer;

  --===================================================--
  --              SW Core parameters                   --
  --===================================================--

  constant c_num_ports               : integer := 18+1;
  constant c_wb_data_width           : integer := 16;
  constant c_wb_addr_width           : integer := 2;
  constant c_wb_sel_width            : integer := 2;
  constant c_prio_num                : integer := 8;
  constant c_num_global_pause        : integer := 2;
  constant c_prio_width              : integer := integer(CEIL(LOG2(real(c_prio_num-1)))) ;

  constant c_swcore_input_bits       : integer := c_num_ports*(
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
                                                  c_num_ports    + -- rtu_dst_mask
                                                  1              + -- rtu_drop
                                                  c_prio_width   + -- rtu_prio
                                                  1              + -- pp_req
                                                  16             + -- pp_quanta
                                                  8             )+ -- pp_class
                                                  c_num_global_pause*(
                                                  1              + -- gp_req
                                                  16             + -- gp_quanta
                                                  8              + -- gp_class
                                                  c_num_ports   )+ -- gp_ports
                                                  1              ; -- shaper_drop
  constant c_swcore_output_bits      : integer := c_num_ports*(
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

  --===================================================--
  --               Pstats parameters                   --
  --===================================================--
  constant c_pstats_input_bits  : integer := 0;
  constant c_pstats_output_bits : integer := 0;


  --===================================================--
  --              Test module parameters               --
  --===================================================--

  -- how many modules are supported
  constant c_nmods : integer := 2;
  -- names of the supported modules
  constant test_mods : t_strarr(0 to c_nmods-1) := ("SWcore", "Pstats");

  constant test_inlen : t_lentype(0 to c_nmods-1) := (
                                              c_swcore_input_bits,   -- SWcore
                                              c_pstats_input_bits    -- Pstats
                                            );
  constant test_outlen: t_lentype(0 to c_nmods-1) := (
                                              c_swcore_output_bits,  -- SWcore
                                              c_pstats_output_bits   -- Pstats
                                            );

  --===================================================--

  function f_modidx(name : string) return integer;
  function f_invec_len(name : string ) return integer;
  function f_outvec_len(name : string ) return integer;

end test_top_pkg;

package body test_top_pkg is

  function f_modidx(name : string) return integer is
    variable i : integer range 0 to c_nmods-1;
  begin
    for i in 0 to c_nmods-1 loop
      if name = test_mods(i) then
        exit;
      end if;
    end loop;
    return i;
  end f_modidx;

  function f_invec_len(name : string ) return integer is
  begin
    return test_inlen(f_modidx(name));
  end f_invec_len;

  function f_outvec_len(name : string ) return integer is
  begin
    return test_outlen(f_modidx(name));
  end f_outvec_len;


end test_top_pkg;
