`define ADDR_PSTATS_CR                 6'h0
`define PSTATS_CR_RD_EN_OFFSET 0
`define PSTATS_CR_RD_EN 32'h00000001
`define PSTATS_CR_RD_IRQ_OFFSET 1
`define PSTATS_CR_RD_IRQ 32'h00000002
`define PSTATS_CR_PORT_OFFSET 8
`define PSTATS_CR_PORT 32'h00001f00
`define PSTATS_CR_ADDR_OFFSET 16
`define PSTATS_CR_ADDR 32'h001f0000
`define ADDR_PSTATS_L1_CNT_VAL         6'h4
`define ADDR_PSTATS_L2_CNT_VAL         6'h8
`define ADDR_PSTATS_INFO               6'hc
`define PSTATS_INFO_VER_OFFSET 0
`define PSTATS_INFO_VER 32'h000000ff
`define PSTATS_INFO_CPW_OFFSET 8
`define PSTATS_INFO_CPW 32'h0000ff00
`define PSTATS_INFO_CPP_OFFSET 16
`define PSTATS_INFO_CPP 32'hffff0000
`define ADDR_PSTATS_EIC_IDR            6'h20
`define PSTATS_EIC_IDR_PORT0_OFFSET 0
`define PSTATS_EIC_IDR_PORT0 32'h00000001
`define PSTATS_EIC_IDR_PORT1_OFFSET 1
`define PSTATS_EIC_IDR_PORT1 32'h00000002
`define PSTATS_EIC_IDR_PORT2_OFFSET 2
`define PSTATS_EIC_IDR_PORT2 32'h00000004
`define PSTATS_EIC_IDR_PORT3_OFFSET 3
`define PSTATS_EIC_IDR_PORT3 32'h00000008
`define PSTATS_EIC_IDR_PORT4_OFFSET 4
`define PSTATS_EIC_IDR_PORT4 32'h00000010
`define PSTATS_EIC_IDR_PORT5_OFFSET 5
`define PSTATS_EIC_IDR_PORT5 32'h00000020
`define PSTATS_EIC_IDR_PORT6_OFFSET 6
`define PSTATS_EIC_IDR_PORT6 32'h00000040
`define PSTATS_EIC_IDR_PORT7_OFFSET 7
`define PSTATS_EIC_IDR_PORT7 32'h00000080
`define PSTATS_EIC_IDR_PORT8_OFFSET 8
`define PSTATS_EIC_IDR_PORT8 32'h00000100
`define PSTATS_EIC_IDR_PORT9_OFFSET 9
`define PSTATS_EIC_IDR_PORT9 32'h00000200
`define PSTATS_EIC_IDR_PORT10_OFFSET 10
`define PSTATS_EIC_IDR_PORT10 32'h00000400
`define PSTATS_EIC_IDR_PORT11_OFFSET 11
`define PSTATS_EIC_IDR_PORT11 32'h00000800
`define PSTATS_EIC_IDR_PORT12_OFFSET 12
`define PSTATS_EIC_IDR_PORT12 32'h00001000
`define PSTATS_EIC_IDR_PORT13_OFFSET 13
`define PSTATS_EIC_IDR_PORT13 32'h00002000
`define PSTATS_EIC_IDR_PORT14_OFFSET 14
`define PSTATS_EIC_IDR_PORT14 32'h00004000
`define PSTATS_EIC_IDR_PORT15_OFFSET 15
`define PSTATS_EIC_IDR_PORT15 32'h00008000
`define PSTATS_EIC_IDR_PORT16_OFFSET 16
`define PSTATS_EIC_IDR_PORT16 32'h00010000
`define PSTATS_EIC_IDR_PORT17_OFFSET 17
`define PSTATS_EIC_IDR_PORT17 32'h00020000
`define ADDR_PSTATS_EIC_IER            6'h24
`define PSTATS_EIC_IER_PORT0_OFFSET 0
`define PSTATS_EIC_IER_PORT0 32'h00000001
`define PSTATS_EIC_IER_PORT1_OFFSET 1
`define PSTATS_EIC_IER_PORT1 32'h00000002
`define PSTATS_EIC_IER_PORT2_OFFSET 2
`define PSTATS_EIC_IER_PORT2 32'h00000004
`define PSTATS_EIC_IER_PORT3_OFFSET 3
`define PSTATS_EIC_IER_PORT3 32'h00000008
`define PSTATS_EIC_IER_PORT4_OFFSET 4
`define PSTATS_EIC_IER_PORT4 32'h00000010
`define PSTATS_EIC_IER_PORT5_OFFSET 5
`define PSTATS_EIC_IER_PORT5 32'h00000020
`define PSTATS_EIC_IER_PORT6_OFFSET 6
`define PSTATS_EIC_IER_PORT6 32'h00000040
`define PSTATS_EIC_IER_PORT7_OFFSET 7
`define PSTATS_EIC_IER_PORT7 32'h00000080
`define PSTATS_EIC_IER_PORT8_OFFSET 8
`define PSTATS_EIC_IER_PORT8 32'h00000100
`define PSTATS_EIC_IER_PORT9_OFFSET 9
`define PSTATS_EIC_IER_PORT9 32'h00000200
`define PSTATS_EIC_IER_PORT10_OFFSET 10
`define PSTATS_EIC_IER_PORT10 32'h00000400
`define PSTATS_EIC_IER_PORT11_OFFSET 11
`define PSTATS_EIC_IER_PORT11 32'h00000800
`define PSTATS_EIC_IER_PORT12_OFFSET 12
`define PSTATS_EIC_IER_PORT12 32'h00001000
`define PSTATS_EIC_IER_PORT13_OFFSET 13
`define PSTATS_EIC_IER_PORT13 32'h00002000
`define PSTATS_EIC_IER_PORT14_OFFSET 14
`define PSTATS_EIC_IER_PORT14 32'h00004000
`define PSTATS_EIC_IER_PORT15_OFFSET 15
`define PSTATS_EIC_IER_PORT15 32'h00008000
`define PSTATS_EIC_IER_PORT16_OFFSET 16
`define PSTATS_EIC_IER_PORT16 32'h00010000
`define PSTATS_EIC_IER_PORT17_OFFSET 17
`define PSTATS_EIC_IER_PORT17 32'h00020000
`define ADDR_PSTATS_EIC_IMR            6'h28
`define PSTATS_EIC_IMR_PORT0_OFFSET 0
`define PSTATS_EIC_IMR_PORT0 32'h00000001
`define PSTATS_EIC_IMR_PORT1_OFFSET 1
`define PSTATS_EIC_IMR_PORT1 32'h00000002
`define PSTATS_EIC_IMR_PORT2_OFFSET 2
`define PSTATS_EIC_IMR_PORT2 32'h00000004
`define PSTATS_EIC_IMR_PORT3_OFFSET 3
`define PSTATS_EIC_IMR_PORT3 32'h00000008
`define PSTATS_EIC_IMR_PORT4_OFFSET 4
`define PSTATS_EIC_IMR_PORT4 32'h00000010
`define PSTATS_EIC_IMR_PORT5_OFFSET 5
`define PSTATS_EIC_IMR_PORT5 32'h00000020
`define PSTATS_EIC_IMR_PORT6_OFFSET 6
`define PSTATS_EIC_IMR_PORT6 32'h00000040
`define PSTATS_EIC_IMR_PORT7_OFFSET 7
`define PSTATS_EIC_IMR_PORT7 32'h00000080
`define PSTATS_EIC_IMR_PORT8_OFFSET 8
`define PSTATS_EIC_IMR_PORT8 32'h00000100
`define PSTATS_EIC_IMR_PORT9_OFFSET 9
`define PSTATS_EIC_IMR_PORT9 32'h00000200
`define PSTATS_EIC_IMR_PORT10_OFFSET 10
`define PSTATS_EIC_IMR_PORT10 32'h00000400
`define PSTATS_EIC_IMR_PORT11_OFFSET 11
`define PSTATS_EIC_IMR_PORT11 32'h00000800
`define PSTATS_EIC_IMR_PORT12_OFFSET 12
`define PSTATS_EIC_IMR_PORT12 32'h00001000
`define PSTATS_EIC_IMR_PORT13_OFFSET 13
`define PSTATS_EIC_IMR_PORT13 32'h00002000
`define PSTATS_EIC_IMR_PORT14_OFFSET 14
`define PSTATS_EIC_IMR_PORT14 32'h00004000
`define PSTATS_EIC_IMR_PORT15_OFFSET 15
`define PSTATS_EIC_IMR_PORT15 32'h00008000
`define PSTATS_EIC_IMR_PORT16_OFFSET 16
`define PSTATS_EIC_IMR_PORT16 32'h00010000
`define PSTATS_EIC_IMR_PORT17_OFFSET 17
`define PSTATS_EIC_IMR_PORT17 32'h00020000
`define ADDR_PSTATS_EIC_ISR            6'h2c
`define PSTATS_EIC_ISR_PORT0_OFFSET 0
`define PSTATS_EIC_ISR_PORT0 32'h00000001
`define PSTATS_EIC_ISR_PORT1_OFFSET 1
`define PSTATS_EIC_ISR_PORT1 32'h00000002
`define PSTATS_EIC_ISR_PORT2_OFFSET 2
`define PSTATS_EIC_ISR_PORT2 32'h00000004
`define PSTATS_EIC_ISR_PORT3_OFFSET 3
`define PSTATS_EIC_ISR_PORT3 32'h00000008
`define PSTATS_EIC_ISR_PORT4_OFFSET 4
`define PSTATS_EIC_ISR_PORT4 32'h00000010
`define PSTATS_EIC_ISR_PORT5_OFFSET 5
`define PSTATS_EIC_ISR_PORT5 32'h00000020
`define PSTATS_EIC_ISR_PORT6_OFFSET 6
`define PSTATS_EIC_ISR_PORT6 32'h00000040
`define PSTATS_EIC_ISR_PORT7_OFFSET 7
`define PSTATS_EIC_ISR_PORT7 32'h00000080
`define PSTATS_EIC_ISR_PORT8_OFFSET 8
`define PSTATS_EIC_ISR_PORT8 32'h00000100
`define PSTATS_EIC_ISR_PORT9_OFFSET 9
`define PSTATS_EIC_ISR_PORT9 32'h00000200
`define PSTATS_EIC_ISR_PORT10_OFFSET 10
`define PSTATS_EIC_ISR_PORT10 32'h00000400
`define PSTATS_EIC_ISR_PORT11_OFFSET 11
`define PSTATS_EIC_ISR_PORT11 32'h00000800
`define PSTATS_EIC_ISR_PORT12_OFFSET 12
`define PSTATS_EIC_ISR_PORT12 32'h00001000
`define PSTATS_EIC_ISR_PORT13_OFFSET 13
`define PSTATS_EIC_ISR_PORT13 32'h00002000
`define PSTATS_EIC_ISR_PORT14_OFFSET 14
`define PSTATS_EIC_ISR_PORT14 32'h00004000
`define PSTATS_EIC_ISR_PORT15_OFFSET 15
`define PSTATS_EIC_ISR_PORT15 32'h00008000
`define PSTATS_EIC_ISR_PORT16_OFFSET 16
`define PSTATS_EIC_ISR_PORT16 32'h00010000
`define PSTATS_EIC_ISR_PORT17_OFFSET 17
`define PSTATS_EIC_ISR_PORT17 32'h00020000
