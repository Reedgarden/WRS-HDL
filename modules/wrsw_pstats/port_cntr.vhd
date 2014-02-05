-------------------------------------------------------------------------------
-- Title      : One port statistics counter
-- Project    : White Rabbit Switch
-------------------------------------------------------------------------------
-- File       : port_cntr.vhd
-- Author     : Grzegorz Daniluk
-- Company    : CERN BE-CO-HT
-- Created    : 2012-12-20
-- Last update: 2013-07-24
-- Platform   : FPGA-generic
-- Standard   : VHDL
-------------------------------------------------------------------------------
-- Description:
-- Set of counters for a single Ethernet port of WR Switch. Inputs a vector of
-- events and increments counters stored in Block-RAM. Each word of RAM stores
-- g_cnt_pw counters so that they can be incremented all at once.
-------------------------------------------------------------------------------
-- Copyright (c) 2012, 2013 Grzegorz Daniluk / CERN
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author          Description
-- 2012-12-20  0.1      greg.d          Created
-- 2013-07-24  0.2      greg.d          Optimized to save FPGA resources
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_misc.all;
use ieee.numeric_std.all;

library work;
use work.genram_pkg.all;
use work.wishbone_pkg.all;
use work.gencores_pkg.all;

entity port_cntr is
  generic(
    g_cnt_pp  : integer := 64;          --number of counters per port
    g_cnt_pw  : integer := 8;           --number of counters per word
    g_keep_ov : integer := 1);
  port(
    rst_n_i : in std_logic;
    clk_i   : in std_logic;

    events_i : in std_logic_vector(g_cnt_pp-1 downto 0);

    --memory interface, only for reading
    ext_adr_i : in  std_logic_vector(f_log2_size((g_cnt_pp+g_cnt_pw-1)/g_cnt_pw)-1 downto 0) := (others => '0');
    ext_dat_o : out std_logic_vector(31 downto 0);

    --overflow events for 2nd layer
    ov_cnt_o : out std_logic_vector(((g_cnt_pp+g_cnt_pw-1)/g_cnt_pw)*g_cnt_pw-1 downto 0)  --c_evt_range

    --debug
    --dbg_evt_ov_o : out std_logic;
    --clr_flags_i  : in  std_logic := '0'
  );
end port_cntr;

architecture behav of port_cntr is

  constant c_rr_range   : integer := (g_cnt_pp+g_cnt_pw-1)/g_cnt_pw;
  constant c_evt_range  : integer := c_rr_range*g_cnt_pw;
  constant c_cnt_width  : integer := c_wishbone_data_width/g_cnt_pw;
  constant c_mem_adr_sz : integer := f_log2_size(c_rr_range);

  type   t_cnt_st is (SEL, WRITE);
  signal cnt_state : t_cnt_st;

  signal mem_dat_in  : std_logic_vector(31 downto 0);
  signal mem_dat_out : std_logic_vector(31 downto 0);
  signal mem_adr     : integer range 0 to c_rr_range-1;
  signal mem_adr_d1  : integer range 0 to c_rr_range-1;
  signal mem_adr_lv  : std_logic_vector(c_mem_adr_sz-1 downto 0);
  signal mem_wr      : std_logic;

  signal events_reg    : std_logic_vector(c_evt_range-1 downto 0);
  signal events_clr    : std_logic_vector(c_evt_range-1 downto 0);
  signal events_sub    : std_logic_vector(g_cnt_pw-1 downto 0);
  signal events_ored   : std_logic_vector(c_rr_range-1 downto 0);
  signal events_preg   : std_logic_vector(c_rr_range-1 downto 0);
  signal events_grant  : std_logic_vector(c_rr_range-1 downto 0);
  signal events_presub : std_logic_vector(g_cnt_pw-1 downto 0);

  signal cnt_ov : std_logic_vector(g_cnt_pw-1 downto 0);

  function f_onehot_decode
    (x : std_logic_vector) return integer is
  begin
    for i in 0 to x'length-1 loop
      if(x(i) = '1') then
        return i;
      end if;
    end loop;
    return 0;
  end f_onehot_decode;

begin

  RAM_A1 : generic_dpram
    generic map(
      g_data_width               => 32,
      g_size                     => c_rr_range,
      g_with_byte_enable         => false,
      g_addr_conflict_resolution => "read_first",
      g_dual_clock               => false)   
    port map(
      rst_n_i => rst_n_i,

      clka_i => clk_i,
      bwea_i => (others => '1'),
      wea_i  => '0',
      aa_i   => ext_adr_i,
      da_i   => (others => '0'),
      qa_o   => ext_dat_o,

      clkb_i => clk_i,
      bweb_i => (others => '1'),
      web_i  => mem_wr,
      ab_i   => mem_adr_lv,
      db_i   => mem_dat_in,
      qb_o   => mem_dat_out);

  mem_adr_lv <= std_logic_vector(to_unsigned(mem_adr, c_mem_adr_sz));

  --store events into temp register, and clear those already counted
  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if(rst_n_i = '0') then
        events_reg   <= (others => '0');
        -- dbg_evt_ov_o <= '0';
      else
        --clear counted events and store new events to be counted
        events_reg(g_cnt_pp-1 downto 0) <= (events_reg(g_cnt_pp-1 downto 0) xor
          events_clr(g_cnt_pp-1 downto 0)) or events_i(g_cnt_pp-1 downto 0);

        -- if(to_integer(unsigned((events_reg(g_cnt_pp-1 downto 0) xor events_clr(g_cnt_pp-1 downto 0))
        --   and events_i(g_cnt_pp-1 downto 0))) /= 0) then
        --     dbg_evt_ov_o <= '1';
        -- end if;

        -- if(clr_flags_i = '1') then
        --   dbg_evt_ov_o <= '0';
        -- end if;
      end if;
    end if;
  end process;


  GEN_EVT_ORED : for i in 0 to c_rr_range-1 generate
    events_ored(i) <= or_reduce(events_reg((i+1)*g_cnt_pw-1 downto i*g_cnt_pw));
  end generate;

  events_presub <= events_reg((f_onehot_decode(events_grant)+1)*g_cnt_pw-1 downto f_onehot_decode(events_grant)*g_cnt_pw);

	GEN_EVT_CLR: for i in 0 to c_rr_range-1 generate
		events_clr((i+1)*g_cnt_pw-1 downto i*g_cnt_pw) <= events_presub when(cnt_state=WRITE and events_grant(i)='1') else
																											(others=>'0');
	end generate;

  mem_adr <= f_onehot_decode(events_grant);

  process(clk_i)
  begin
    if rising_edge(clk_i) then
      if(rst_n_i = '0') then
        cnt_state   <= SEL;
        mem_wr      <= '0';
        events_sub  <= (others => '0');
        events_preg <= (others => '0');
      else


        case(cnt_state) is
          when SEL =>
            --check each segment of events_i starting from the one pointed by round robin
            mem_wr     <= '0';

            f_rr_arbitrate(events_ored, events_preg, events_grant);
            if(or_reduce(events_ored) = '1') then
              events_preg   <= events_grant;

              cnt_state <= WRITE;
            end if;

          when WRITE =>
            events_sub    <= events_presub;
            mem_wr     <= '1';
            cnt_state  <= SEL;
        end case;
      end if;
    end if;
  end process;

  GEN_INCR : for i in 0 to g_cnt_pw-1 generate

    KEEP_OV : if g_keep_ov = 1 generate
    mem_dat_in((i+1)*c_cnt_width-1 downto i*c_cnt_width) <=
               --counter overflow, don't increment
               mem_dat_out((i+1)*c_cnt_width-1 downto i*c_cnt_width)  when (unsigned(mem_dat_out((i+1)*c_cnt_width-1 downto i*c_cnt_width))+1 = 0) else
               --otherwise, normal situation, just increment
               std_logic_vector(unsigned(mem_dat_out((i+1)*c_cnt_width-1 downto i*c_cnt_width)) + 1) when (events_sub(i) = '1') else
               --no change
               mem_dat_out((i+1)*c_cnt_width-1 downto i*c_cnt_width);
    end generate;

    NKEEP_OV : if g_keep_ov = 0 generate
      mem_dat_in((i+1)*c_cnt_width-1 downto i*c_cnt_width) <=
                 --otherwise, normal situation, just increment
                 std_logic_vector(unsigned(mem_dat_out((i+1)*c_cnt_width-1 downto i*c_cnt_width)) + 1) when (events_sub(i) = '1') else
                 --no change
                 mem_dat_out((i+1)*c_cnt_width-1 downto i*c_cnt_width);
    end generate;


    process(clk_i)
    begin
      if rising_edge(clk_i) then
        if(rst_n_i = '0') then
          cnt_ov(i) <= '0';
        else
          if((unsigned(mem_dat_out((i+1)*c_cnt_width-1 downto i*c_cnt_width))+1 = 0) and mem_wr = '1' and events_sub(i) = '1') then
            cnt_ov(i) <= '1';
					else
          	cnt_ov(i) <= '0';
					end if;
        end if;
      end if;
    end process;

  end generate;

  -----------------------------------------------
  --overflow events for 2nd layer counter
  process(clk_i)  --delay mem_adr by 1 cycle to match ov_cnt_o signal
  begin
    if rising_edge(clk_i) then
      if(rst_n_i = '0') then
        mem_adr_d1 <= 0;
      else
        mem_adr_d1 <= mem_adr;
      end if;
    end if;
  end process;

  --multiplexer driven from mem_adr delayed by 1 cycle
	GEN_CNT_OV: for I in 0 to c_rr_range-1 generate
		ov_cnt_o((I+1)*g_cnt_pw-1 downto I*g_cnt_pw) <= cnt_ov when (mem_adr_d1 = I) else (others=>'0');
	end generate;


end behav;
