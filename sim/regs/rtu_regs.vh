`define ADDR_RTU_GCR                   16'h0
`define RTU_GCR_G_ENA_OFFSET 0
`define RTU_GCR_G_ENA 32'h00000001
`define RTU_GCR_MFIFOTRIG_OFFSET 1
`define RTU_GCR_MFIFOTRIG 32'h00000002
`define RTU_GCR_POLY_VAL_OFFSET 8
`define RTU_GCR_POLY_VAL 32'h00ffff00
`define ADDR_RTU_PSR                   16'h4
`define RTU_PSR_PORT_SEL_OFFSET 0
`define RTU_PSR_PORT_SEL 32'h000000ff
`define RTU_PSR_N_PORTS_OFFSET 8
`define RTU_PSR_N_PORTS 32'h0000ff00
`define ADDR_RTU_PCR                   16'h8
`define RTU_PCR_LEARN_EN_OFFSET 0
`define RTU_PCR_LEARN_EN 32'h00000001
`define RTU_PCR_PASS_ALL_OFFSET 1
`define RTU_PCR_PASS_ALL 32'h00000002
`define RTU_PCR_PASS_BPDU_OFFSET 2
`define RTU_PCR_PASS_BPDU 32'h00000004
`define RTU_PCR_FIX_PRIO_OFFSET 3
`define RTU_PCR_FIX_PRIO 32'h00000008
`define RTU_PCR_PRIO_VAL_OFFSET 4
`define RTU_PCR_PRIO_VAL 32'h00000070
`define RTU_PCR_B_UNREC_OFFSET 7
`define RTU_PCR_B_UNREC 32'h00000080
`define ADDR_RTU_EIC_IDR               16'h20
`define RTU_EIC_IDR_NEMPTY_OFFSET 0
`define RTU_EIC_IDR_NEMPTY 32'h00000001
`define ADDR_RTU_EIC_IER               16'h24
`define RTU_EIC_IER_NEMPTY_OFFSET 0
`define RTU_EIC_IER_NEMPTY 32'h00000001
`define ADDR_RTU_EIC_IMR               16'h28
`define RTU_EIC_IMR_NEMPTY_OFFSET 0
`define RTU_EIC_IMR_NEMPTY 32'h00000001
`define ADDR_RTU_EIC_ISR               16'h2c
`define RTU_EIC_ISR_NEMPTY_OFFSET 0
`define RTU_EIC_ISR_NEMPTY 32'h00000001
`define ADDR_RTU_UFIFO_R0              16'h30
`define RTU_UFIFO_R0_DMAC_LO_OFFSET 0
`define RTU_UFIFO_R0_DMAC_LO 32'hffffffff
`define ADDR_RTU_UFIFO_R1              16'h34
`define RTU_UFIFO_R1_DMAC_HI_OFFSET 0
`define RTU_UFIFO_R1_DMAC_HI 32'h0000ffff
`define ADDR_RTU_UFIFO_R2              16'h38
`define RTU_UFIFO_R2_SMAC_LO_OFFSET 0
`define RTU_UFIFO_R2_SMAC_LO 32'hffffffff
`define ADDR_RTU_UFIFO_R3              16'h3c
`define RTU_UFIFO_R3_SMAC_HI_OFFSET 0
`define RTU_UFIFO_R3_SMAC_HI 32'h0000ffff
`define ADDR_RTU_UFIFO_R4              16'h40
`define RTU_UFIFO_R4_VID_OFFSET 0
`define RTU_UFIFO_R4_VID 32'h00000fff
`define RTU_UFIFO_R4_PRIO_OFFSET 12
`define RTU_UFIFO_R4_PRIO 32'h00007000
`define RTU_UFIFO_R4_PID_OFFSET 16
`define RTU_UFIFO_R4_PID 32'h000f0000
`define RTU_UFIFO_R4_HAS_VID_OFFSET 20
`define RTU_UFIFO_R4_HAS_VID 32'h00100000
`define RTU_UFIFO_R4_HAS_PRIO_OFFSET 21
`define RTU_UFIFO_R4_HAS_PRIO 32'h00200000
`define ADDR_RTU_UFIFO_CSR             16'h44
`define RTU_UFIFO_CSR_EMPTY_OFFSET 17
`define RTU_UFIFO_CSR_EMPTY 32'h00020000
`define RTU_UFIFO_CSR_USEDW_OFFSET 0
`define RTU_UFIFO_CSR_USEDW 32'h0000007f
`define ADDR_RTU_MFIFO_R0              16'h48
`define RTU_MFIFO_R0_AD_SEL_OFFSET 0
`define RTU_MFIFO_R0_AD_SEL 32'h00000001
`define ADDR_RTU_MFIFO_R1              16'h4c
`define RTU_MFIFO_R1_AD_VAL_OFFSET 0
`define RTU_MFIFO_R1_AD_VAL 32'hffffffff
`define ADDR_RTU_MFIFO_CSR             16'h50
`define RTU_MFIFO_CSR_FULL_OFFSET 16
`define RTU_MFIFO_CSR_FULL 32'h00010000
`define RTU_MFIFO_CSR_EMPTY_OFFSET 17
`define RTU_MFIFO_CSR_EMPTY 32'h00020000
`define RTU_MFIFO_CSR_USEDW_OFFSET 0
`define RTU_MFIFO_CSR_USEDW 32'h0000003f
`define BASE_RTU_ARAM                  16'h4000
`define SIZE_RTU_ARAM                  32'h100
`define BASE_RTU_VLAN_TAB              16'h8000
`define SIZE_RTU_VLAN_TAB              32'h1000
