library ieee;
use ieee.std_logic_1164.all;

entity pstats_vectorized_top is
  generic (
    g_nports    : integer := 8;
    g_cnt_pp    : integer := 16;
    g_in_bits   : integer;
    g_out_bits  : integer
  );
  port (
    rst_n_i : in std_logic;
    clk_i   : in std_logic;
    input_vector_i  : in  std_logic_vector(g_in_bits-1 downto 0);
    output_vector_o : out std_logic_vector(g_out_bits-1 downto 0)
  );
end pstats_vectorized_top;

architecture rtl of pstats_vectorized_top is

  component wrsw_pstats
  generic(
    g_nports    : integer := 2;
    g_cnt_pp    : integer := 16;
    g_cnt_pw    : integer := 4;
    --Layer 2
    g_L2_cnt_pw : integer := 4;
    g_keep_ov   : integer := 1);
  port(
    rst_n_i : in std_logic;
    clk_i   : in std_logic;

    events_i : in std_logic_vector(g_nports*g_cnt_pp-1 downto 0);

    wb_adr_i   : in  std_logic_vector(3 downto 0);
    wb_dat_i   : in  std_logic_vector(31 downto 0);
    wb_dat_o   : out std_logic_vector(31 downto 0);
    wb_cyc_i   : in  std_logic;
    wb_sel_i   : in  std_logic_vector(3 downto 0);
    wb_stb_i   : in  std_logic;
    wb_we_i    : in  std_logic;
    wb_ack_o   : out std_logic;
    wb_stall_o : out std_logic;
    wb_int_o   : out std_logic);
  end component;

  constant c_wbadr_base  : integer := 0;
  constant c_wbdati_base : integer := 4;
  constant c_wbcyc_base  : integer := c_wbdati_base+32;
  constant c_wbsel_base  : integer := c_wbcyc_base+1;
  constant c_wbstb_base  : integer := c_wbsel_base+4;
  constant c_wbwe_base   : integer := c_wbstb_base+1;
  constant c_events_base : integer := c_wbwe_base+1;

  constant c_wbdato_base : integer := 0;
  constant c_wback_base  : integer := c_wbdato_base+32;
  constant c_wbstall_base: integer := c_wback_base+1;
  constant c_wbint_base  : integer := c_wbstall_base+1;

begin

  U_PSTATS: wrsw_pstats
    generic map (
      g_nports    => g_nports,
      g_cnt_pp    => g_cnt_pp,
      g_cnt_pw    => 4,
      g_L2_cnt_pw => 4,
      g_keep_ov   => 0)
    port map (
      rst_n_i   => rst_n_i,
      clk_i     => clk_i,
      events_i  => input_vector_i(c_events_base+g_nports*g_cnt_pp-1 downto c_events_base),
      wb_adr_i  => input_vector_i(c_wbadr_base+3 downto c_wbadr_base),
      wb_dat_i  => input_vector_i(c_wbdati_base+31 downto c_wbdati_base),
      wb_dat_o  => output_vector_o(c_wbdato_base+31 downto c_wbdato_base),
      wb_cyc_i  => input_vector_i(c_wbcyc_base),
      wb_sel_i  => input_vector_i(c_wbsel_base+3 downto c_wbsel_base),
      wb_stb_i  => input_vector_i(c_wbstb_base),
      wb_we_i   => input_vector_i(c_wbwe_base),
      wb_ack_o  => output_vector_o(c_wback_base),
      wb_stall_o=> output_vector_o(c_wbstall_base),
      wb_int_o  => output_vector_o(c_wbint_base));
    
end rtl;
