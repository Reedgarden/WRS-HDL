---------------------------------------------------------------------------------------
-- Title          : Wishbone slave core for WR Switch Per-Port Statistic Counters
---------------------------------------------------------------------------------------
-- File           : pstats_wbgen2_pkg.vhd
-- Author         : auto-generated by wbgen2 from wrsw_pstats.wb
-- Created        : Thu Feb 14 16:41:42 2013
-- Standard       : VHDL'87
---------------------------------------------------------------------------------------
-- THIS FILE WAS GENERATED BY wbgen2 FROM SOURCE FILE wrsw_pstats.wb
-- DO NOT HAND-EDIT UNLESS IT'S ABSOLUTELY NECESSARY!
---------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package pstats_wbgen2_pkg is
  
  
  -- Input registers (user design -> WB slave)
  
  type t_pstats_in_registers is record
    cr_rd_en_i                               : std_logic;
    cnt_val_i                                : std_logic_vector(31 downto 0);
    irq_r1_port0_i                           : std_logic_vector(4 downto 0);
    irq_r1_port1_i                           : std_logic_vector(4 downto 0);
    irq_r1_port2_i                           : std_logic_vector(4 downto 0);
    irq_r1_port3_i                           : std_logic_vector(4 downto 0);
    irq_r2_port4_i                           : std_logic_vector(4 downto 0);
    irq_r2_port5_i                           : std_logic_vector(4 downto 0);
    irq_r2_port6_i                           : std_logic_vector(4 downto 0);
    irq_r2_port7_i                           : std_logic_vector(4 downto 0);
    dbg_evt_ov_i                             : std_logic_vector(7 downto 0);
    dbg_cnt_ov_i                             : std_logic_vector(7 downto 0);
    end record;
  
  constant c_pstats_in_registers_init_value: t_pstats_in_registers := (
    cr_rd_en_i => '0',
    cnt_val_i => (others => '0'),
    irq_r1_port0_i => (others => '0'),
    irq_r1_port1_i => (others => '0'),
    irq_r1_port2_i => (others => '0'),
    irq_r1_port3_i => (others => '0'),
    irq_r2_port4_i => (others => '0'),
    irq_r2_port5_i => (others => '0'),
    irq_r2_port6_i => (others => '0'),
    irq_r2_port7_i => (others => '0'),
    dbg_evt_ov_i => (others => '0'),
    dbg_cnt_ov_i => (others => '0')
    );
    
    -- Output registers (WB slave -> user design)
    
    type t_pstats_out_registers is record
      cr_rd_en_o                               : std_logic;
      cr_rd_en_load_o                          : std_logic;
      cr_port_o                                : std_logic_vector(4 downto 0);
      cr_addr_o                                : std_logic_vector(4 downto 0);
      dbg_clr_o                                : std_logic;
      end record;
    
    constant c_pstats_out_registers_init_value: t_pstats_out_registers := (
      cr_rd_en_o => '0',
      cr_rd_en_load_o => '0',
      cr_port_o => (others => '0'),
      cr_addr_o => (others => '0'),
      dbg_clr_o => '0'
      );
    function "or" (left, right: t_pstats_in_registers) return t_pstats_in_registers;
    function f_x_to_zero (x:std_logic) return std_logic;
    function f_x_to_zero (x:std_logic_vector) return std_logic_vector;
end package;

package body pstats_wbgen2_pkg is
function f_x_to_zero (x:std_logic) return std_logic is
begin
if(x = 'X' or x = 'U') then
return '0';
else
return x;
end if; 
end function;
function f_x_to_zero (x:std_logic_vector) return std_logic_vector is
variable tmp: std_logic_vector(x'length-1 downto 0);
begin
for i in 0 to x'length-1 loop
if(x(i) = 'X' or x(i) = 'U') then
tmp(i):= '0';
else
tmp(i):=x(i);
end if; 
end loop; 
return tmp;
end function;
function "or" (left, right: t_pstats_in_registers) return t_pstats_in_registers is
variable tmp: t_pstats_in_registers;
begin
tmp.cr_rd_en_i := f_x_to_zero(left.cr_rd_en_i) or f_x_to_zero(right.cr_rd_en_i);
tmp.cnt_val_i := f_x_to_zero(left.cnt_val_i) or f_x_to_zero(right.cnt_val_i);
tmp.irq_r1_port0_i := f_x_to_zero(left.irq_r1_port0_i) or f_x_to_zero(right.irq_r1_port0_i);
tmp.irq_r1_port1_i := f_x_to_zero(left.irq_r1_port1_i) or f_x_to_zero(right.irq_r1_port1_i);
tmp.irq_r1_port2_i := f_x_to_zero(left.irq_r1_port2_i) or f_x_to_zero(right.irq_r1_port2_i);
tmp.irq_r1_port3_i := f_x_to_zero(left.irq_r1_port3_i) or f_x_to_zero(right.irq_r1_port3_i);
tmp.irq_r2_port4_i := f_x_to_zero(left.irq_r2_port4_i) or f_x_to_zero(right.irq_r2_port4_i);
tmp.irq_r2_port5_i := f_x_to_zero(left.irq_r2_port5_i) or f_x_to_zero(right.irq_r2_port5_i);
tmp.irq_r2_port6_i := f_x_to_zero(left.irq_r2_port6_i) or f_x_to_zero(right.irq_r2_port6_i);
tmp.irq_r2_port7_i := f_x_to_zero(left.irq_r2_port7_i) or f_x_to_zero(right.irq_r2_port7_i);
tmp.dbg_evt_ov_i := f_x_to_zero(left.dbg_evt_ov_i) or f_x_to_zero(right.dbg_evt_ov_i);
tmp.dbg_cnt_ov_i := f_x_to_zero(left.dbg_cnt_ov_i) or f_x_to_zero(right.dbg_cnt_ov_i);
return tmp;
end function;
end package body;
