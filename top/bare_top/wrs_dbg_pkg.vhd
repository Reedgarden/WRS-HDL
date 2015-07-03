library ieee;
use ieee.std_logic_1164.all;

package wrs_dbg_pkg is

  -- SWCore Input Block types
  type t_dbg_swc_ib is record 
    rtu_valid : std_logic;
    alloc_fsm : std_logic_vector(2 downto 0);
    trans_fsm : std_logic_vector(3 downto 0);
    rcv_fsm   : std_logic_vector(3 downto 0);
    ll_fsm    : std_logic_vector(3 downto 0);
  end record;
  type t_dbg_swc_ib_array is array (natural range <>) of t_dbg_swc_ib;

  -- SWCore Output Block types
  type t_dbg_swc_ob is record
    send_fsm : std_logic_vector(3 downto 0);
    prep_fsm : std_logic_vector(3 downto 0);
  end record;
  type t_dbg_swc_ob_array is array (natural range <>) of t_dbg_swc_ob;
  
  -- SWCore MMU types
  type t_dbg_swc_palloc is record
    free_pages : std_logic_vector(10 downto 0);
    q_write : std_logic;
    q_read  : std_logic;
    res_almost_full : std_logic_vector(2 downto 0);
  end record;
  type t_dbg_swc_mmu is record
    palloc : t_dbg_swc_palloc;
  end record;

  -- General SWCore types
  type t_dbg_swc is record
    ib  : t_dbg_swc_ib_array(17 downto 0);
    ob  : t_dbg_swc_ob_array(17 downto 0);
    mmu : t_dbg_swc_mmu;
  end record;

  -- RTU types
  type t_dbg_rtu_port is record
    fsm : std_logic_vector(2 downto 0);
    rsp_valid : std_logic;
    rsp_ack   : std_logic;
    full_match_full : std_logic;
    full_match_wr   : std_logic;
    full_match_done : std_logic;
    fast_match_wr   : std_logic;
    fast_rd_valid   : std_logic;
    full_valid      : std_logic;
    full_aboard_d   : std_logic;
    new_req_at_full_rsp : std_logic;
    rq_valid        : std_logic;
    fast_valid_reg  : std_logic;
    full_valid_reg  : std_logic;
  end record;
  type t_dbg_rtu_port_array is array (natural range <>) of t_dbg_rtu_port;

  type t_dbg_rtu is record
    rport : t_dbg_rtu_port_array(17 downto 0);
  end record;

end wrs_dbg_pkg;

package body wrs_dbg_pkg is
end wrs_dbg_pkg;
