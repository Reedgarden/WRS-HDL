library ieee;
use ieee.std_logic_1164.all;

package wrs_dbg_pkg is

  -- SWCore Input Block types
  type t_dbg_swc_ib is record 
    rtu_valid : std_logic;
    rtu_ack   : std_logic;
    alloc_fsm : std_logic_vector(2 downto 0);
    trans_fsm : std_logic_vector(3 downto 0);
    rcv_fsm   : std_logic_vector(3 downto 0);
    ll_fsm    : std_logic_vector(3 downto 0);
    mem_full_dump   : std_logic;
    finish_rcv      : std_logic;
    force_free      : std_logic;
    force_free_done : std_logic;
    force_free_adr  : std_logic_vector(9 downto 0);
    pckstart_in_adv : std_logic;
    pckinter_in_adv : std_logic;
    ll_wr         : std_logic;
    ll_wr_done    : std_logic;
    ll_page       : std_logic_vector(9 downto 0);
    ll_next_page  : std_logic_vector(9 downto 0);
    ll_page_valid : std_logic;
    ll_eof        : std_logic;
    ll_size       : std_logic_vector(6 downto 0);
    alloc_req     : std_logic;
    alloc_done    : std_logic;
    alloc_page    : std_logic_vector(9 downto 0);
    page_start    : std_logic_vector(9 downto 0);
    page_inter    : std_logic_vector(9 downto 0);
    cur_pckstart  : std_logic_vector(9 downto 0);
    pta_pgadr     : std_logic_vector(9 downto 0);
    pta_transfer  : std_logic;
    pta_transfer_ack : std_logic;
    pta_mask      : std_logic_vector(7 downto 0);
    pck_sof       : std_logic;
    pck_eof       : std_logic;
    pck_err       : std_logic;
    cyc           : std_logic;
    stb           : std_logic;
    ack           : std_logic;
    stall         : std_logic;
    mpm_pg_req_d0 : std_logic;
    mpm_dlast_d0  : std_logic;
    mpm_pg_adr    : std_logic_vector(9 downto 0);
    rcv_stuck     : std_logic;
    tr_drop_stuck : std_logic;
    tr_wait_stuck : std_logic;
    pages_same    : std_logic;
    ll_pckstart_stored : std_logic;
    ffree_mask    : std_logic;
    new_pck_first_page : std_logic;
    rp_rcv_fpage  : std_logic;
    rp_ll_entry_size : std_logic_vector(6 downto 0);
    in_pck_dvalid : std_logic;
    page_word_cnt : std_logic_vector(6 downto 0);
    mpm_dvalid    : std_logic;
    mpm_data      : std_logic_vector(17 downto 0);
    sof_on_stall  : std_logic;
    sof_delayed   : std_logic;
    sof_normal    : std_logic;
    rtu_hp        : std_logic;
    current_drop  : std_logic;
    res_info_almost_full : std_logic;
    almost_full_i : std_logic;
    rtu_sof_bug   : std_logic;
    rtu_rsp_cnt   : std_logic_vector(7 downto 0);
    sof_cnt   : std_logic_vector(7 downto 0);
    dbg_bare_sof : std_logic_vector(7 downto 0);
    pck_sof_reg   : std_logic;
    pck_sof_ack   : std_logic;
    eof_normal    : std_logic;
    eof_on_pause  : std_logic;
    pck_sof_rcv_reg : std_logic;
    accept_rtu    : std_logic;
    pcknew_reg    : std_logic;
  end record;
  type t_dbg_swc_ib_array is array (natural range <>) of t_dbg_swc_ib;

  -- SWCore Output Block types
  type t_dbg_swc_ob is record
    send_fsm : std_logic_vector(3 downto 0);
    prep_fsm : std_logic_vector(3 downto 0);
    free     : std_logic;
    free_done : std_logic;
    free_adr  : std_logic_vector(9 downto 0);
    cycle_frozen : std_logic;
    mpm_pgreq : std_logic;
    pta_transfer_valid : std_logic;
    pta_pgadr : std_logic_vector(9 downto 0);
    pta_ack   : std_logic;
    obq_full  : std_logic;
    data_error  : std_logic;
    mpm_dlast   : std_logic;
  end record;
  type t_dbg_swc_ob_array is array (natural range <>) of t_dbg_swc_ob;
  
  -- SWCore MMU types
  type t_dbg_swc_palloc is record
    free_pages : std_logic_vector(10 downto 0);
    q_write : std_logic;
    q_read  : std_logic;
    res_almost_full : std_logic;
    res_full  : std_logic;
    rd_ptr    : std_logic_vector(9 downto 0);
    wr_ptr    : std_logic_vector(9 downto 0);
    in_pg     : std_logic_vector(9 downto 0);
    out_pg    : std_logic_vector(9 downto 0);
    grant_port_msk : std_logic_vector(18 downto 0);
    out_nomem_d1  : std_logic;
    alloc_done    : std_logic;
    alloc_req_d0  : std_logic;
    pg_adv_valid  : std_logic;
    dbg_trig      : std_logic;
  end record;
  type t_dbg_swc_mmu is record
    palloc : t_dbg_swc_palloc;
    p0_req_alloc : std_logic;
    p1_req_alloc : std_logic;
    p6_req_alloc : std_logic;
    p7_req_alloc : std_logic;
    p0_arb_req   : std_logic;
    p0_arb_grant : std_logic;
    p1_arb_req   : std_logic;
    p1_arb_grant : std_logic;
    p6_arb_req   : std_logic;
    p6_arb_grant : std_logic;
    p7_arb_req   : std_logic;
    p7_arb_grant : std_logic;
    grant_ib_d0  : std_logic_vector(8 downto 0);
  end record;

  -- SWCore Free Module types
  type t_dbg_swc_free is record
    free      : std_logic;
    free_done : std_logic;
    ffree     : std_logic;
    ffree_done: std_logic;
    pgadr     : std_logic_vector(9 downto 0);
    fsm       : std_logic_vector(2 downto 0);
    last_ucnt : std_logic;
    ib_ffree  : std_logic;
    ib_ffree_done : std_logic;
    ib_pgadr  : std_logic_vector(9 downto 0);
    ob_free   : std_logic;
    ob_free_done : std_logic;
    ob_pgadr  : std_logic_vector(9 downto 0);
  end record;
  type t_dbg_swc_free_array is array (natural range<>) of t_dbg_swc_free;

  type t_dbg_swc_mll_rv is record
    read_req  : std_logic;
    v_read    : std_logic;
    nv_read   : std_logic;
    v_write   : std_logic;
    rd_valid  : std_logic;
    req_o     : std_logic;
    rd_adr    : std_logic_vector(9 downto 0);
  end record;
  type t_dbg_swc_mll_rv_array is array (natural range<>) of t_dbg_swc_mll_rv;

  type t_dbg_swc_mll is record
    rv  : t_dbg_swc_mll_rv_array(18 downto 0);
    fpck_grant_d1 : std_logic_vector(18 downto 0);
  end record;

  -- MPM types
  type t_dbg_mpm_read_io is record
    ll_req   : std_logic;
    ll_grant : std_logic;
    ll_adr   : std_logic_vector(9 downto 0);
    ll_next_page : std_logic_vector(9 downto 0);
    ll_eof   : std_logic;
    ll_valid : std_logic;
    page_fsm : std_logic_vector(2 downto 0);
    words_xmitted     : std_logic_vector(9 downto 0);
    last_pg_start_ptr : std_logic_vector(9 downto 0);
    long_rst_at_abort : std_logic;
    --
    rport_pg_req     : std_logic;
    rport_pg_valid_i : std_logic;
    pre_fetch        : std_logic;
    last_int         : std_logic;
    rport_dreq       : std_logic;
    words_total      : std_logic_vector(9 downto 0);
    fetch_first      : std_logic;
    fetch_ack        : std_logic;
    ll_size          : std_logic_vector(6 downto 0);
    fetch_pg_words   : std_logic_vector(6 downto 0);
  end record;
  type t_dbg_mpm_read_io_array is array (natural range<>) of t_dbg_mpm_read_io;

  type t_dbg_mpm_read is record
    ll_adr : std_logic_vector(9 downto 0);
    io     : t_dbg_mpm_read_io_array(18 downto 0);
  end record;

  type t_dbg_mpm is record
    read : t_dbg_mpm_read;
  end record;

  -- General SWCore types
  type t_dbg_swc is record
    ib  : t_dbg_swc_ib_array(18 downto 0);
    ob  : t_dbg_swc_ob_array(18 downto 0);
    mmu : t_dbg_swc_mmu;
    free: t_dbg_swc_free_array(18 downto 0);
    mll : t_dbg_swc_mll;
    mpm : t_dbg_mpm;
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
