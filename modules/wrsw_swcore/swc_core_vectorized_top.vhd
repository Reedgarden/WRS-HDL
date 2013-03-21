-------------------------------------------------------------------------------
-- Title      : Switch Core 
-- Project    : WhiteRabbit switch
-------------------------------------------------------------------------------
-- File       : swc_core.vhd
-- Author     : Maciej Lipinski
-- Company    : CERN BE-Co-HT
-- Created    : 2010-10-29
-- Last update: 2012-02-02
-- Platform   : FPGA-generic
-- Standard   : VHDL'87
-------------------------------------------------------------------------------
-- Description: 
--
-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
-- 
-------------------------------------------------------------------------------
--
-- Copyright (c) 2010 Maciej Lipinski / CERN
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
-- 2010-10-29  1.0      mlipinsk Created
-- 2012-02-02  2.0      mlipinsk generic-azed
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.CEIL;
use ieee.math_real.log2;

library work;
use work.swc_swcore_pkg.all;
use work.wr_fabric_pkg.all;
use work.wrsw_shared_types_pkg.all;

entity swc_core_vectorized_top is
  generic( 
    g_num_ports               : integer :=6; 
    g_in_bits                 : integer;
    g_out_bits                : integer;
    g_wb_data_width           : integer := 16;
    g_wb_addr_width           : integer := 2;
    g_wb_sel_width            : integer := 2;
    g_prio_num                : integer := 8;
    g_num_global_pause        : integer := 2;
    g_prio_width              : integer --: integer(CEIL(LOG2(real(g_prio_num-1))))    ;    
    );
  port (
    clk_i          : in std_logic;
    clk_mpm_core_i : in std_logic;
    rst_n_i        : in std_logic;

    input_vector_i  : in std_logic_vector(g_in_bits   - 1 downto 0);
    output_vector_o : out std_logic_vector(g_out_bits - 1 downto 0)

    );
end swc_core_vectorized_top;

architecture rtl of swc_core_vectorized_top is
  

    signal snk_dat_i   : std_logic_vector(g_wb_data_width*g_num_ports-1 downto 0);
    signal snk_adr_i   : std_logic_vector(g_wb_addr_width*g_num_ports-1 downto 0);
    signal snk_sel_i   : std_logic_vector(g_wb_sel_width *g_num_ports-1 downto 0);
    signal snk_cyc_i   : std_logic_vector(                g_num_ports-1 downto 0);
    signal snk_stb_i   : std_logic_vector(                g_num_ports-1 downto 0);
    signal snk_we_i    : std_logic_vector(                g_num_ports-1 downto 0);
    signal snk_stall_o : std_logic_vector(                g_num_ports-1 downto 0);
    signal snk_ack_o   : std_logic_vector(                g_num_ports-1 downto 0);
    signal snk_err_o   : std_logic_vector(                g_num_ports-1 downto 0);
    signal snk_rty_o   : std_logic_vector(                g_num_ports-1 downto 0);

    signal src_dat_o   : std_logic_vector(g_wb_data_width*g_num_ports-1  downto 0);
    signal src_adr_o   : std_logic_vector(g_wb_addr_width*g_num_ports-1 downto 0);
    signal src_sel_o   : std_logic_vector(g_wb_sel_width *g_num_ports-1 downto 0);
    signal src_cyc_o   : std_logic_vector(                g_num_ports-1 downto 0);
    signal src_stb_o   : std_logic_vector(                g_num_ports-1 downto 0);
    signal src_we_o    : std_logic_vector(                g_num_ports-1 downto 0);
    signal src_stall_i : std_logic_vector(                g_num_ports-1 downto 0);
    signal src_ack_i   : std_logic_vector(                g_num_ports-1 downto 0);
    signal src_err_i   : std_logic_vector(                g_num_ports-1 downto 0);

    signal rtu_rsp_valid_i     : std_logic_vector(g_num_ports               - 1 downto 0);
    signal rtu_rsp_ack_o       : std_logic_vector(g_num_ports               - 1 downto 0);
    signal rtu_dst_port_mask_i : std_logic_vector(g_num_ports * g_num_ports - 1 downto 0);
    signal rtu_drop_i          : std_logic_vector(g_num_ports               - 1 downto 0);
    signal rtu_prio_i          : std_logic_vector(g_num_ports * g_prio_width- 1 downto 0);

    signal gp_req_i            : std_logic_vector(g_num_global_pause        - 1 downto 0);
    signal gp_quanta_i         : std_logic_vector(g_num_global_pause*16     - 1 downto 0);
    signal gp_classes_i        : std_logic_vector(g_num_global_pause*8      - 1 downto 0);
    signal gp_ports_i          : std_logic_vector(g_num_global_pause*g_num_ports- 1 downto 0);

    signal pp_req_i            : std_logic_vector(g_num_ports               - 1 downto 0);
    signal pp_quanta_i         : std_logic_vector(g_num_ports*16            - 1 downto 0);
    signal pp_classes_i        : std_logic_vector(g_num_ports*8             - 1 downto 0);

    signal shaper_drop_at_hp_ena_i : std_logic;
    signal dbg_o : std_logic_vector(7 downto 0);
    

  begin --rtl

    U_Swcore : swc_core
      generic map (
        g_prio_num                        => g_prio_num ,--8,
        g_output_queue_num                => 8,
        g_max_pck_size                    => 10 * 1024,
        g_max_oob_size                    => 3,
        g_num_ports                       => g_num_ports,
        g_pck_pg_free_fifo_size           => 512,
        g_input_block_cannot_accept_data  => "drop_pck",
        g_output_block_per_queue_fifo_size=> 64,
        g_wb_data_width                   => g_wb_data_width, --16,
        g_wb_addr_width                   => g_wb_addr_width ,--2,
        g_wb_sel_width                    => g_wb_sel_width ,--2,
        g_wb_ob_ignore_ack                => false,
        g_mpm_mem_size                    => 67584,
        g_mpm_page_size                   => 66,
        g_mpm_ratio                       => 6, --f_swc_ratio,  --2
        g_mpm_fifo_size                   => 8,
        g_mpm_fetch_next_pg_in_advance    => false,
        g_drop_outqueue_head_on_full      => true,
        g_num_global_pause                => g_num_global_pause,
        g_num_dbg_vector_width            => 8)
      port map (
        clk_i          => clk_i,
        clk_mpm_core_i => clk_mpm_core_i,
        rst_n_i        => rst_n_i,

        snk_dat_i           => snk_dat_i,
        snk_adr_i           => snk_adr_i,
        snk_sel_i           => snk_sel_i,
        snk_cyc_i           => snk_cyc_i,
        snk_stb_i           => snk_stb_i,
        snk_we_i            => snk_we_i,
        snk_stall_o         => snk_stall_o,
        snk_ack_o           => snk_ack_o,
        snk_err_o           => snk_err_o,
        snk_rty_o           => snk_rty_o,

        src_dat_o           => src_dat_o,
        src_adr_o           => src_adr_o,
        src_sel_o           => src_sel_o,
        src_cyc_o           => src_cyc_o,
        src_stb_o           => src_stb_o,
        src_we_o            => src_we_o,
        src_stall_i         => src_stall_i,
        src_ack_i           => src_ack_i,
        src_err_i           => src_err_i,

        rtu_rsp_valid_i     => rtu_rsp_valid_i,
        rtu_rsp_ack_o       => rtu_rsp_ack_o,
        rtu_dst_port_mask_i => rtu_dst_port_mask_i,
        rtu_drop_i          => rtu_drop_i,
        rtu_prio_i          => rtu_prio_i,

        gp_req_i            => gp_req_i,
        gp_quanta_i         => gp_quanta_i,
        gp_classes_i        => gp_classes_i,
        gp_ports_i          => gp_ports_i,

        pp_req_i            => pp_req_i,
        pp_quanta_i         => pp_quanta_i,
        pp_classes_i        => pp_classes_i,

        dbg_o               => dbg_o,
        shaper_drop_at_hp_ena_i => shaper_drop_at_hp_ena_i
        );

   output_vector_o(g_wb_data_width*g_num_ports-1  downto 0)                            <= src_dat_o;
   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports-1  downto g_wb_data_width*g_num_ports)  <= src_adr_o; 
   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports)  <= src_sel_o;

   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports+
                                 1*g_num_ports-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports+
                                                         g_wb_sel_width *g_num_ports)  <= snk_stall_o;

   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports+
                                 2*g_num_ports-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports+
                                                         g_wb_sel_width *g_num_ports+
                                                                       1*g_num_ports)  <= snk_ack_o;

   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports+
                                 3*g_num_ports-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports+
                                                         g_wb_sel_width *g_num_ports+
                                                                       2*g_num_ports)  <= snk_err_o;

   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports+
                                 4*g_num_ports-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports+
                                                         g_wb_sel_width *g_num_ports+
                                                                       3*g_num_ports)  <= snk_rty_o;

   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports+
                                 5*g_num_ports-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports+
                                                         g_wb_sel_width *g_num_ports+
                                                                       4*g_num_ports)  <= src_cyc_o;

   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports+
                                 6*g_num_ports-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports+
                                                         g_wb_sel_width *g_num_ports+
                                                                       5*g_num_ports)  <= src_stb_o;

   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports+
                                 7*g_num_ports-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports+
                                                         g_wb_sel_width *g_num_ports+
                                                                       6*g_num_ports)  <= src_we_o;

   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports+
                                 8*g_num_ports-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports+
                                                         g_wb_sel_width *g_num_ports+
                                                                       7*g_num_ports)  <= rtu_rsp_ack_o;

   output_vector_o(g_wb_data_width*g_num_ports+
                   g_wb_addr_width*g_num_ports+
                   g_wb_sel_width *g_num_ports+
                                 8*g_num_ports+
                                             8-1  downto g_wb_data_width*g_num_ports+
                                                         g_wb_addr_width*g_num_ports+
                                                         g_wb_sel_width *g_num_ports+
                                                                       8*g_num_ports)  <= dbg_o;


   snk_dat_i           <= input_vector_i(g_wb_data_width*g_num_ports-1 downto 0);
   snk_adr_i           <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports-1 downto g_wb_data_width*g_num_ports);  
   snk_sel_i           <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports);  
   snk_cyc_i           <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      1 *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports); 
   snk_stb_i           <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      2 *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            1*g_num_ports);
   snk_we_i            <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      3 *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            2*g_num_ports);

   src_stall_i         <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      4 *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            3*g_num_ports);
   src_ack_i           <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      5 *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            4*g_num_ports);
   src_err_i           <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      6 *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            5*g_num_ports);

   rtu_rsp_valid_i     <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      7 *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            6*g_num_ports);
   rtu_drop_i          <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            7*g_num_ports);
   rtu_dst_port_mask_i <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            8*g_num_ports);
   rtu_prio_i          <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports+
                                            g_prio_width*g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            8*g_num_ports+
                                                                                 g_num_ports *g_num_ports);


   pp_quanta_i         <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports+
                                            g_prio_width*g_num_ports+
                                                      16*g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            8*g_num_ports+
                                                                                 g_num_ports *g_num_ports+
                                                                                 g_prio_width*g_num_ports);
   
  pp_classes_i        <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports+
                                            g_prio_width*g_num_ports+
                                                      16*g_num_ports+
                                                       8*g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            8*g_num_ports+
                                                                                 g_num_ports *g_num_ports+
                                                                                 g_prio_width*g_num_ports+
                                                                                           16*g_num_ports);

   pp_req_i            <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports+
                                            g_prio_width*g_num_ports+
                                                      16*g_num_ports+
                                                       8*g_num_ports+
                                                       1*g_num_ports-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            8*g_num_ports+
                                                                                 g_num_ports *g_num_ports+
                                                                                 g_prio_width*g_num_ports+
                                                                                           16*g_num_ports+
                                                                                            8*g_num_ports);

   gp_quanta_i         <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports+
                                            g_prio_width*g_num_ports+
                                                      16*g_num_ports+
                                                       8*g_num_ports+
                                                       1*g_num_ports+
                                                      16*g_num_global_pause-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            8*g_num_ports+
                                                                                 g_num_ports *g_num_ports+
                                                                                 g_prio_width*g_num_ports+
                                                                                           16*g_num_ports+
                                                                                            8*g_num_ports+
                                                                                            1*g_num_ports);



   gp_classes_i        <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports+
                                            g_prio_width*g_num_ports+
                                                      16*g_num_ports+
                                                       8*g_num_ports+
                                                       1*g_num_ports+
                                                      16*g_num_global_pause+
                                                       8*g_num_global_pause-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            8*g_num_ports+
                                                                                 g_num_ports *g_num_ports+
                                                                                 g_prio_width*g_num_ports+
                                                                                           16*g_num_ports+
                                                                                            8*g_num_ports+
                                                                                            1*g_num_ports+
                                                                                           16*g_num_global_pause);
   gp_ports_i          <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports+
                                            g_prio_width*g_num_ports+
                                                      16*g_num_ports+
                                                       8*g_num_ports+
                                                       1*g_num_ports+
                                                      16*g_num_global_pause+
                                                       8*g_num_global_pause+
                                             g_num_ports*g_num_global_pause-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            8*g_num_ports+
                                                                                 g_num_ports *g_num_ports+
                                                                                 g_prio_width*g_num_ports+
                                                                                           16*g_num_ports+
                                                                                            8*g_num_ports+
                                                                                            1*g_num_ports+
                                                                                           16*g_num_global_pause+
                                                                                            8*g_num_global_pause);
   gp_req_i            <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports+
                                            g_prio_width*g_num_ports+
                                                      16*g_num_ports+
                                                       8*g_num_ports+
                                                       1*g_num_ports+
                                                      16*g_num_global_pause+
                                                       8*g_num_global_pause+
                                             g_num_ports*g_num_global_pause+
                                                       1*g_num_global_pause-1 downto g_wb_data_width*g_num_ports+
                                                                              g_wb_addr_width*g_num_ports+
                                                                              g_wb_sel_width *g_num_ports+
                                                                                            8*g_num_ports+
                                                                                 g_num_ports *g_num_ports+
                                                                                 g_prio_width*g_num_ports+
                                                                                           16*g_num_ports+
                                                                                            8*g_num_ports+
                                                                                            1*g_num_ports+
                                                                                           16*g_num_global_pause+
                                                                                            8*g_num_global_pause+
                                                                                  g_num_ports*g_num_global_pause);

  shaper_drop_at_hp_ena_i <= input_vector_i(g_wb_data_width*g_num_ports+
                                         g_wb_addr_width*g_num_ports+
                                         g_wb_sel_width *g_num_ports+
                                                      8 *g_num_ports+
                                            g_num_ports *g_num_ports+
                                            g_prio_width*g_num_ports+
                                                      16*g_num_ports+
                                                       8*g_num_ports+
                                                       1*g_num_ports+
                                                      16*g_num_global_pause+
                                                       8*g_num_global_pause+
                                             g_num_ports*g_num_global_pause+
                                                       1*g_num_global_pause);
end rtl;
