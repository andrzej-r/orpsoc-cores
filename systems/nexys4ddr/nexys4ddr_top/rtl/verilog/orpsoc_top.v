//
// ORPSoC top-level module for Nexys4-DDR board
//
// Copyright (C) 2015 Andrzej <ndrwrdck@gmail.com>
//
// Redistribution and use in source and non-source forms, with or without
// modification, are permitted provided that the following conditions are met:
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in non-source form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
// THIS WORK IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// WORK, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

`include "orpsoc-defines.v"

module orpsoc_top #
  (
   //parameter BOOTROM_FILE = "../src/nexys4ddr_top/sw/spi_uimage_loader.vh",
   parameter BOOTROM_FILE = "../src/nexys4ddr_top/sw/boot_loop.vh",
   //parameter BOOTROM_FILE = "../src/nexys4ddr_top/sw/hexloader.vh",
   //parameter BOOTROM_FILE = "../src/nexys4ddr_top/sw/led_blink.vh",
   parameter NEXYS4DDR_XADC_SIM_MONITOR_FILE = "../src/nexys4ddr_xadc/nexys4ddr_xadc/simulation/timing/nexys4ddr_xadc_sim.txt",
   parameter rom0_aw = 8,
   parameter uart0_aw = 3
   )
   (
    input         clk_i,
    input         rst_n_i,

    // Push-buttons
    input [4:0]   pbtn_i, //left,center,right,down,up

    // Switches
    input [15:0]  sw_i,

    // ADT7420 Temperature Sensor TWI Signals
    inout         i2c_temp_scl_io,
    inout         i2c_temp_sda_io,
    input         i2c_temp_int_i,
    input         i2c_temp_ct_i,

    // ADXL362 Accelerometer SPI Signals
    output        spi_accel_sclk_o,
    output        spi_accel_mosi_o,
    input         spi_accel_miso_i,
    output        spi_accel_cs_n_o,
    input         spi_accel_int1_i,
    input         spi_accel_int2_i,

    // USB-RS232 Interface
    input         uart_cts_i,
    output        uart_rts_o,
    input         uart_rxd_i,
    output        uart_txd_o,

    // 7-segment display
    inout  [7:0]  sseg_disp_seg_o,
    output [7:0]  sseg_disp_an_o, // active low

    // LD16 RGB led
    output        led_rgb1_red_o,
    output        led_rgb1_green_o,
    output        led_rgb1_blue_o,
    // LD17 RGB led
    output        led_rgb2_red_o,
    output        led_rgb2_green_o,
    output        led_rgb2_blue_o,

    //LEDs
    output [15:0] led_o,

    //VGA Signals
    output        vga_hs_o,
    output        vga_vs_o,
    output [3:0]  vga_red_o,
    output [3:0]  vga_green_o,
    output [3:0]  vga_blue_o,

    // ADMP421 Omnidirectional Microphone Signals
    output        mic_pdm_clk_o,
    input         mic_pdm_data_i,
    output        mic_pdm_lrsel_o,

    // Audio Out Signals
    inout         pwm_audio_o, // open-drain
    output        pwm_sdaudio_o,

    // PS2 Signals
    inout         ps2_clk, // open-drain
    inout         ps2_data, // open-drain

    // Pmod Headers
    inout [7:0]  ja_io,
    inout [7:0]  jb_io,
    inout [7:0]  jc_io,
    inout [7:0]  jd_io,

    // Quad SPI Flash
    output        spi_flash_cs_n_o,
    inout [3:0]   spi_flash_dq_io,
    //output        spi_flash_sck_o, // dedicated pin, access via STARTUPE2

    // SMSC Ethernet PHY
    output        eth0_refclk_o,
    output        eth0_rst_n_o,
    output        eth0_tx_en_o,
    output [1:0]  eth0_txd_o,
    input [1:0]   eth0_rxd_i,
    input         eth0_rx_er_i,
    input         eth0_crs_dv_i,
    input         eth0_intn_i,
    output        eth0_mdc_o,
    inout         eth0_mdio_io,

    // DDR2 interface signals
    output [12:0] ddr2_addr,
    output [2:0]  ddr2_ba,
    output        ddr2_ras_n,
    output        ddr2_cas_n,
    output        ddr2_we_n,
    output [0:0]  ddr2_ck_p,
    output [0:0]  ddr2_ck_n,
    output [0:0]  ddr2_cke,
    output [0:0]  ddr2_cs_n,
    output [1:0]  ddr2_dm,
    output [0:0]  ddr2_odt,
    inout [15:0]  ddr2_dq,
    inout [1:0]   ddr2_dqs_p,
    inout [1:0]   ddr2_dqs_n

    // JTAG
    // output              tdo_pad_o,
    // input               tms_pad_i,
    // input               tck_pad_i,
    // input               tdi_pad_i,
    );

   parameter      IDCODE_VALUE=32'h13631093;


   wire [15:0]    led_net;
   wire [7:0]     ja_net;
   wire [7:0]     jb_net;
   wire [7:0]     jc_net;
   wire [7:0]     jd_net;

   genvar         i;

   // ADT7420 Temperature Sensor TWI Signals
   //assign         i2c_temp_scl_io = 1'bz;
   //assign         i2c_temp_sda_io = 1'bz;

   // ADXL362 Accelerometer SPI Signals
   //assign         spi_accel_sclk_o = 1'b0;
   //assign         spi_accel_mosi_o = 1'b0;
   //assign         spi_accel_cs_n_o = 1'b1;

   // USB-RS232 Interface
   //assign        uart_rts_o = 1'b0;
   //assign        uart_txd_o = 1'b1;

   //VGA Signals
   //assign        vga_hs_o = 1'b0;
   //assign        vga_vs_o = 1'b0;
   //assign        vga_red_o = 4'b0;
   //assign        vga_green_o = 4'b0;
   //assign        vga_blue_o = 4'b0;

   // ADMP421 Omnidirectional Microphone Signals
   assign        mic_pdm_clk_o = 1'b0;
   assign        mic_pdm_lrsel_o = 1'b0;

   // Audio Out Signals
   assign        pwm_audio_o = 1'bz; // open-drain
   assign        pwm_sdaudio_o = 1'b0;

   // PS2 Signals
   assign        ps2_clk = 1'bz; // open-drain
   assign        ps2_data = 1'bz; // open-drain

   // Quad SPI Flash
   //assign        spi_flash_cs_n_o = 1'b0;
   //assign        spi_flash_dq_io = 4'bz;
   //assign        spi_flash_sck_o, // dedicated pin, access via STARTUPE2

   // SMSC Ethernet PHY
   assign        eth_txd_o = 2'b0;
   assign        eth_mdc_o = 1'b0;
   assign        eth_mdio_io = 1'bz;
   assign        eth_refclk_io = 1'b0;
   assign        eth_rst_n_o = 1'b0;
   assign        eth_txen_o = 1'b0;


   ////////////////////////////////////////////////////////////////////////
   // Clock and reset generation module
   ////////////////////////////////////////////////////////////////////////

   wire                 async_rst;
   wire                 wb_clk, wb_rst;
   wire                 dbg_tck;

   wire                 dvi_clk;

   wire                 ddr2_if_sys_clk;
   wire                 ddr2_if_ref_clk;
   wire                 ddr2_if_rst;
   wire                 clk100;

   nexys4ddr_clkgen clkgen0
     (
      .clk_i              (clk_i),
      .rst_n_i            (rst_n_i),
      .async_rst_o        (async_rst),
      //.wb_clk_o           (wb_clk),
      //.wb_rst_o           (wb_rst),
      .wb_clk_o           (),
      .wb_rst_o           (),
      .ddr2_if_sys_clk_o  (ddr2_if_sys_clk),
      .ddr2_if_ref_clk_o  (ddr2_if_ref_clk),
      .ddr2_if_rst_o      (ddr2_if_rst),
      .clk100_o           (clk100)
      );

   ////////////////////////////////////////////////////////////////////////
   // Modules interconnections
   ////////////////////////////////////////////////////////////////////////
`include "wb_intercon.vh"

   ////////////////////////////////////////////////////////////////////////
   // OR1K CPU
   ////////////////////////////////////////////////////////////////////////

   wire [31:0]          or1k_irq;

   wire [31:0]          or1k_dbg_dat_i;
   wire [31:0]          or1k_dbg_adr_i;
   wire                 or1k_dbg_we_i;
   wire                 or1k_dbg_stb_i;
   wire                 or1k_dbg_ack_o;
   wire [31:0]          or1k_dbg_dat_o;

   wire                 or1k_dbg_stall_i;
   wire                 or1k_dbg_ewt_i;
   wire [3:0]           or1k_dbg_lss_o;
   wire [1:0]           or1k_dbg_is_o;
   wire [10:0]          or1k_dbg_wp_o;
   wire                 or1k_dbg_bp_o;
   wire                 or1k_dbg_rst;

   wire                 sig_tick;

   wire                 or1k_rst;

`ifdef OR1200

   or1200_top #
     (
      .boot_adr(32'hf0000000)
      )
   or1200_top0
     (
      // Instruction bus, clocks, reset
      .iwb_clk_i                      (wb_clk),
      .iwb_rst_i                      (wb_rst),
      .iwb_ack_i                      (wb_s2m_or1k_i_ack),
      .iwb_err_i                      (wb_s2m_or1k_i_err),
      .iwb_rty_i                      (wb_s2m_or1k_i_rty),
      .iwb_dat_i                      (wb_s2m_or1k_i_dat),

      .iwb_cyc_o                      (wb_m2s_or1k_i_cyc),
      .iwb_adr_o                      (wb_m2s_or1k_i_adr),
      .iwb_stb_o                      (wb_m2s_or1k_i_stb),
      .iwb_we_o                       (wb_m2s_or1k_i_we),
      .iwb_sel_o                      (wb_m2s_or1k_i_sel),
      .iwb_dat_o                      (wb_m2s_or1k_i_dat),
      .iwb_cti_o                      (wb_m2s_or1k_i_cti),
      .iwb_bte_o                      (wb_m2s_or1k_i_bte),

      // Data bus, clocks, reset
      .dwb_clk_i                      (wb_clk),
      .dwb_rst_i                      (wb_rst),
      .dwb_ack_i                      (wb_s2m_or1k_d_ack),
      .dwb_err_i                      (wb_s2m_or1k_d_err),
      .dwb_rty_i                      (wb_s2m_or1k_d_rty),
      .dwb_dat_i                      (wb_s2m_or1k_d_dat),

      .dwb_cyc_o                      (wb_m2s_or1k_d_cyc),
      .dwb_adr_o                      (wb_m2s_or1k_d_adr),
      .dwb_stb_o                      (wb_m2s_or1k_d_stb),
      .dwb_we_o                       (wb_m2s_or1k_d_we),
      .dwb_sel_o                      (wb_m2s_or1k_d_sel),
      .dwb_dat_o                      (wb_m2s_or1k_d_dat),
      .dwb_cti_o                      (wb_m2s_or1k_d_cti),
      .dwb_bte_o                      (wb_m2s_or1k_d_bte),

      // Debug interface ports
      .dbg_stall_i                    (or1k_dbg_stall_i),
      .dbg_ewt_i                      (1'b0),
      .dbg_lss_o                      (or1k_dbg_lss_o),
      .dbg_is_o                       (or1k_dbg_is_o),
      .dbg_wp_o                       (or1k_dbg_wp_o),
      .dbg_bp_o                       (or1k_dbg_bp_o),

      .dbg_adr_i                      (or1k_dbg_adr_i),
      .dbg_we_i                       (or1k_dbg_we_i),
      .dbg_stb_i                      (or1k_dbg_stb_i),
      .dbg_dat_i                      (or1k_dbg_dat_i),
      .dbg_dat_o                      (or1k_dbg_dat_o),
      .dbg_ack_o                      (or1k_dbg_ack_o),

      .pm_clksd_o                     (),
      .pm_dc_gate_o                   (),
      .pm_ic_gate_o                   (),
      .pm_dmmu_gate_o                 (),
      .pm_immu_gate_o                 (),
      .pm_tt_gate_o                   (),
      .pm_cpu_gate_o                  (),
      .pm_wakeup_o                    (),
      .pm_lvolt_o                     (),

      // Core clocks, resets
      .clk_i                          (wb_clk),
      .rst_i                          (or1k_rst),

      .clmode_i                       (2'b00),

      // Interrupts
      .pic_ints_i                     (or1k_irq),
      .sig_tick                       (sig_tick),

      .pm_cpustall_i                  (1'b0)
      );
`endif

   wire exception;
`ifdef MOR1KX
   mor1kx
     #(
       .FEATURE_DEBUGUNIT         ("ENABLED"),
       .FEATURE_CMOV              ("ENABLED"),
       .FEATURE_INSTRUCTIONCACHE  ("ENABLED"),
       //.FEATURE_INSTRUCTIONCACHE  ("DISABLED"),
       //.OPTION_ICACHE_BLOCK_WIDTH (4), // use 4 to force wrap-4 bursts matching 128b DDR2 word
       .OPTION_ICACHE_BLOCK_WIDTH (5),
       .OPTION_ICACHE_SET_WIDTH   (8),
       .OPTION_ICACHE_WAYS        (2),
       .OPTION_ICACHE_LIMIT_WIDTH (32),
       //.FEATURE_IMMU              ("DISABLED"),
       .FEATURE_IMMU              ("ENABLED"),
       //.OPTION_IMMU_SET_WIDTH     (7),
       .FEATURE_DATACACHE         ("ENABLED"),
       //.FEATURE_DATACACHE         ("DISABLED"),
       //.OPTION_DCACHE_BLOCK_WIDTH (4), // use 4 to force wrap-4 bursts matching 128b DDR2 word
       .OPTION_DCACHE_BLOCK_WIDTH (5),
       .OPTION_DCACHE_SET_WIDTH   (8),
       .OPTION_DCACHE_WAYS        (2),
       .OPTION_DCACHE_LIMIT_WIDTH (31),
       .FEATURE_DMMU              ("ENABLED"),
       //.OPTION_DMMU_SET_WIDTH     (7),
       .OPTION_PIC_TRIGGER        ("LATCHED_LEVEL"),

       //.IBUS_WB_TYPE               ("READ_B3_BURSTING"), // do not use with Cappuccino
       .IBUS_WB_TYPE               ("B3_REGISTERED_FEEDBACK"),
       .DBUS_WB_TYPE               ("B3_REGISTERED_FEEDBACK"),
       //.OPTION_CPU0                ("PRONTO_ESPRESSO"),
       //.OPTION_CPU0                ("ESPRESSO"),
       .OPTION_CPU0                ("CAPPUCCINO"),
       .OPTION_RESET_PC            (32'hf0000000)
       )
   mor1kx0
     (
      .iwbm_adr_o            (wb_m2s_or1k_i_adr),
      .iwbm_stb_o            (wb_m2s_or1k_i_stb),
      .iwbm_cyc_o            (wb_m2s_or1k_i_cyc),
      .iwbm_sel_o            (wb_m2s_or1k_i_sel),
      .iwbm_we_o             (wb_m2s_or1k_i_we),
      .iwbm_cti_o            (wb_m2s_or1k_i_cti),
      .iwbm_bte_o            (wb_m2s_or1k_i_bte),
      .iwbm_dat_o            (wb_m2s_or1k_i_dat),

      .dwbm_adr_o            (wb_m2s_or1k_d_adr),
      .dwbm_stb_o            (wb_m2s_or1k_d_stb),
      .dwbm_cyc_o            (wb_m2s_or1k_d_cyc),
      .dwbm_sel_o            (wb_m2s_or1k_d_sel),
      .dwbm_we_o             (wb_m2s_or1k_d_we ),
      .dwbm_cti_o            (wb_m2s_or1k_d_cti),
      .dwbm_bte_o            (wb_m2s_or1k_d_bte),
      .dwbm_dat_o            (wb_m2s_or1k_d_dat),

      .clk                   (wb_clk),
      .rst                   (or1k_rst),

      .iwbm_err_i            (wb_s2m_or1k_i_err),
      .iwbm_ack_i            (wb_s2m_or1k_i_ack),
      .iwbm_dat_i            (wb_s2m_or1k_i_dat),
      .iwbm_rty_i            (wb_s2m_or1k_i_rty),

      .dwbm_err_i            (wb_s2m_or1k_d_err),
      .dwbm_ack_i            (wb_s2m_or1k_d_ack),
      .dwbm_dat_i            (wb_s2m_or1k_d_dat),
      .dwbm_rty_i            (wb_s2m_or1k_d_rty),

      .avm_d_address_o       (),
      .avm_d_byteenable_o    (),
      .avm_d_read_o          (),
      .avm_d_readdata_i      (32'h00000000),
      .avm_d_burstcount_o    (),
      .avm_d_write_o         (),
      .avm_d_writedata_o     (),
      .avm_d_waitrequest_i   (1'b0),
      .avm_d_readdatavalid_i (1'b0),

      .avm_i_address_o       (),
      .avm_i_byteenable_o    (),
      .avm_i_read_o          (),
      .avm_i_readdata_i      (32'h00000000),
      .avm_i_burstcount_o    (),
      .avm_i_waitrequest_i   (1'b0),
      .avm_i_readdatavalid_i (1'b0),

      .irq_i                 (or1k_irq),
      //.exception (exception),
      .du_addr_i             (or1k_dbg_adr_i[15:0]),
      .du_stb_i              (or1k_dbg_stb_i),
      .du_dat_i              (or1k_dbg_dat_i),
      .du_we_i               (or1k_dbg_we_i),
      .du_dat_o              (or1k_dbg_dat_o),
      .du_ack_o              (or1k_dbg_ack_o),
      .du_stall_i            (or1k_dbg_stall_i),
      .du_stall_o            (or1k_dbg_bp_o)
      );

`endif

   ////////////////////////////////////////////////////////////////////////
   // Debug Interface
   ////////////////////////////////////////////////////////////////////////

`ifdef DEBUG_JTAG
   wire                 dbg_if_tdo;
   wire                 jtag_tap_tdo;
   wire                 jtag_tap_shift_dr;
   wire                 jtag_tap_pause_dr;
   wire                 jtag_tap_update_dr;
   wire                 jtag_tap_capture_dr;
   wire                 jtag_tap_select;
   wire                 jtag_tap_reset;
   wire                 jtag_tap_drck;

   adbg_top dbg_if0
     (
      // OR1K interface
      .cpu0_clk_i     (wb_clk),
      .cpu0_rst_o     (or1k_dbg_rst),
      .cpu0_addr_o    (or1k_dbg_adr_i),
      .cpu0_data_o    (or1k_dbg_dat_i),
      .cpu0_stb_o     (or1k_dbg_stb_i),
      .cpu0_we_o      (or1k_dbg_we_i),
      .cpu0_data_i    (or1k_dbg_dat_o),
      .cpu0_ack_i     (or1k_dbg_ack_o),
      .cpu0_stall_o   (or1k_dbg_stall_i),
      .cpu0_bp_i      (or1k_dbg_bp_o),

      // TAP interface
      //.tck_i          (dbg_tck),
      //.tck_i          (jtag_tap_drck),
      .tck_i          (sw_i[1] ? jtag_tap_drck : dbg_tck),
      .tdi_i          (jtag_tap_tdo),
      .tdo_o          (dbg_if_tdo),
      //.rst_i          (wb_rst),
      //.rst_i          (jtag_tap_reset),
      .rst_i          (sw_i[0] ? wb_rst : jtag_tap_reset),
      .capture_dr_i   (jtag_tap_capture_dr),
      .shift_dr_i     (jtag_tap_shift_dr),
      .pause_dr_i     (jtag_tap_pause_dr),
      .update_dr_i    (jtag_tap_update_dr),
      //.debug_select_i (1'b1),
      .debug_select_i (jtag_tap_select | sw_i[2]),

      // Wishbone debug master
      .wb_clk_i       (wb_clk),
      .wb_rst_i       (wb_rst),
      .wb_dat_i       (wb_s2m_dbg_dat),
      .wb_ack_i       (wb_s2m_dbg_ack),
      .wb_err_i       (wb_s2m_dbg_err),

      .wb_adr_o       (wb_m2s_dbg_adr),
      .wb_dat_o       (wb_m2s_dbg_dat),
      .wb_cyc_o       (wb_m2s_dbg_cyc),
      .wb_stb_o       (wb_m2s_dbg_stb),
      .wb_sel_o       (wb_m2s_dbg_sel),
      .wb_we_o        (wb_m2s_dbg_we),
      .wb_cab_o       (),
      .wb_cti_o       (wb_m2s_dbg_cti),
      .wb_bte_o       (wb_m2s_dbg_bte),

      .wb_jsp_adr_i   (32'd0),
      .wb_jsp_dat_i   (32'd0),
      .wb_jsp_cyc_i   (1'b0),
      .wb_jsp_stb_i   (1'b0),
      .wb_jsp_sel_i   (4'h0),
      .wb_jsp_we_i    (1'b0),
      .wb_jsp_cab_i   (1'b0),
      .wb_jsp_cti_i   (3'd0),
      .wb_jsp_bte_i   (2'd0),
      .wb_jsp_dat_o   (),
      .wb_jsp_ack_o   (),
      .wb_jsp_err_o   (),

      .int_o          ()
      );
   assign or1k_rst = wb_rst; // | or1k_dbg_rst; // FIX
`else // !`ifdef DEBUG_JTAG
   assign or1k_rst = wb_rst; // | or1k_dbg_rst; // FIX
   // replace with a debug i/f
   assign or1k_dbg_stall_i = 0;
   assign or1k_dbg_adr_i = 0;
   assign or1k_dbg_we_i = 0;
   assign or1k_dbg_stb_i = 0;
   assign or1k_dbg_dat_i = 0;
`endif // !`ifdef DEBUG_JTAG

   ////////////////////////////////////////////////////////////////////////
   // GENERIC JTAG TAP
   ////////////////////////////////////////////////////////////////////////

`ifdef DEBUG_JTAG
 `ifdef JTAG_TAP_CUSTOM
   tap_top jtag_tap0
     (
      .tdo_pad_o                      (tdo_pad_o),
      .tms_pad_i                      (tms_pad_i),
      .tck_pad_i                      (dbg_tck),
      .trst_pad_i                     (async_rst),
      .tdi_pad_i                      (tdi_pad_i),

      .tdo_padoe_o                    (tdo_padoe_o),

      .tdo_o                          (jtag_tap_tdo),

      .shift_dr_o                     (jtag_tap_shift_dr),
      .pause_dr_o                     (jtag_tap_pause_dr),
      .update_dr_o                    (jtag_tap_update_dr),
      .capture_dr_o                   (jtag_tap_capture_dr),

      .extest_select_o                (),
      .sample_preload_select_o        (),
      .mbist_select_o                 (),
      .debug_select_o                 (jtag_tap_select),


      .bs_chain_tdi_i                 (1'b0),
      .mbist_tdi_i                    (1'b0),
      .debug_tdi_i                    (dbg_if_tdo)
      );
   assign jtag_tap_drck = dbg_tck;
 `endif //  `ifdef JTAG_TAP_CUSTOM

 `ifdef JTAG_TAP_XILINX
   // BSCANE2: Boundary-Scan User Instruction
   //          7 Series
   // Xilinx HDL Libraries Guide, version 14.3

   wire jtag_tap_runtest;
   BSCANE2
     #(
       .JTAG_CHAIN(1) // Value for USER command.
       )
   xilinx_jtag_tap0
     (
      .CAPTURE   (jtag_tap_capture_dr), // 1-bit output: CAPTURE output from TAP controller.
      .DRCK      (jtag_tap_drck),       // 1-bit output: Gated TCK output. When SEL is asserted, DRCK toggles when CAPTURE or SHIFT are asserted.
      .RESET     (jtag_tap_reset),      // Reset output for TAP controller.
      .RUNTEST   (jtag_tap_runtest),    // Output asserted when TAP controller is in Run Test/Idle state.
      .SEL       (jtag_tap_select),     // USER instruction active output.
      .SHIFT     (jtag_tap_shift_dr),   // SHIFT output from TAP controller
      .TCK       (dbg_tck),             // Test Clock output. Fabric connection to TAP Clock pin
      .TDI       (jtag_tap_tdo),        // Test Data Input (TDI) output from TAP controller.
      .TMS       (jtag_tap_tms),        // Test Mode Select output. Fabric connection to TAP.
      .UPDATE    (jtag_tap_update_dr),  // UPDATE output from TAP controller
      .TDO       (dbg_if_tdo)           // Test Data Output (TDO) input for USER function.
      );
   // End of BSCANE2_inst instantiation

   assign        jtag_tap_pause_dr = 1'b0;
 `endif //  `ifdef JTAG_TAP_XILINX
`endif //  `ifdef DEBUG_JTAG

   ////////////////////////////////////////////////////////////////////////
   // ROM
   ////////////////////////////////////////////////////////////////////////

`ifdef BOOTROM
   wb_bootrom #
     (
      .DEPTH    (1024),
      .MEMFILE  (BOOTROM_FILE)
      )
   bootrom
     (
      //Wishbone Master interface
      .wb_clk_i (wb_clk),
      .wb_rst_i (wb_rst),
      .wb_adr_i	(wb_m2s_rom0_adr),
      .wb_cyc_i	(wb_m2s_rom0_cyc),
      .wb_stb_i	(wb_m2s_rom0_stb),
      .wb_dat_o	(wb_s2m_rom0_dat),
      .wb_ack_o (wb_s2m_rom0_ack)
      );
`else
   assign wb_s2m_rom0_dat = 0;
   assign wb_s2m_rom0_ack = 0;
`endif
   assign wb_s2m_rom0_err = 1'b0;
   assign wb_s2m_rom0_rty = 1'b0;

   ////////////////////////////////////////////////////////////////////////
   // (Quad) SPI Flash Memory
   ////////////////////////////////////////////////////////////////////////

`ifdef SPI_FLASH
   wire spi_flash_sck;
   STARTUPE2 #
     (
      .PROG_USR("FALSE"),
      .SIM_CCLK_FREQ(10.0)
      )
   STARTUPE2_inst
     (
      .CFGCLK   (),
      .CFGMCLK  (),
      .EOS      (),
      .PREQ     (),
      .CLK      (1'b0),
      .GSR      (1'b0),
      .GTS      (1'b0),
      .KEYCLEARB(1'b0),
      .PACK     (1'b0),
      .USRCCLKO (spi_flash_sck),
      .USRCCLKTS(1'b0),
      .USRDONEO (1'b1),
      .USRDONETS(1'b0)
      );

   // For now, use a simple single-lane/single-rate SPI I/F
   simple_spi spi_flash
     (
      // Wishbone slave interface
      .clk_i          (wb_clk),
      .rst_i          (wb_rst),
      .adr_i          (wb_m2s_spi_flash_adr[2:0]),
      .dat_i          (wb_m2s_spi_flash_dat),
      .we_i           (wb_m2s_spi_flash_we),
      .stb_i          (wb_m2s_spi_flash_stb),
      .cyc_i          (wb_m2s_spi_flash_cyc),
      .dat_o          (wb_s2m_spi_flash_dat),
      .ack_o          (wb_s2m_spi_flash_ack),

      // Outputs
      .inta_o         (spi_flash_irq),
      .sck_o          (spi_flash_sck), //to STARTUPE2
      .ss_o           (spi_flash_cs_n_o),
      .mosi_o         (spi_flash_dq_io[0]),

      // Inputs
      .miso_i         (spi_flash_dq_io[1])
      );
   assign spi_flash_dq_io[3:2] = 2'bzz;
`else // !`ifdef SPI_FLASH
   assign spi_flash_dq_io[3:0] = 4'bzzzz;
   assign spi_flash_cs_n_o = 1'b1;
   assign  wb_s2m_spi_flash_dat = 0;
   assign  wb_s2m_spi_flash_ack = 0;
`endif // !`ifdef SPI_FLASH
   assign  wb_s2m_spi_flash_err = 0;
   assign  wb_s2m_spi_flash_rty = 0;


   ////////////////////////////////////////////////////////////////////////
   // DDR2 SDRAM Memory Controller
   ////////////////////////////////////////////////////////////////////////
   wire  [26:0]          app_addr;
   wire  [2:0]           app_cmd;
   wire                  app_en;
   wire  [127:0]         app_wdf_data;
   wire                  app_wdf_end;
   wire  [15:0]          app_wdf_mask;
   wire                  app_wdf_wren;
   wire  [127:0]         app_rd_data;
   wire                  app_rd_data_end;
   wire                  app_rd_data_valid;
   wire                  app_rdy;
   wire                  app_wdf_rdy;
   wire    [3:0]         ddr2_state;
   wire   [26:0]         ddr2_address_burst;
   wire                  ddr2_miss;

   wire ddr2_init_calib_complete;
`ifdef DDR2
   nexys4ddr_ddr2_wb #
     (
      .awc ( 4),
      .dwc (16)
      )
   ddr2_wb0
     (
      .wb_clk_o              (wb_clk),
      .wb_rst_o              (wb_rst),
      .async_rst_i           (async_rst),

      .wb_adr_i              (wb_m2s_ddr2_adr),
      .wb_bte_i              (wb_m2s_ddr2_bte),
      .wb_cti_i              (wb_m2s_ddr2_cti),
      .wb_cyc_i              (wb_m2s_ddr2_cyc),
      .wb_dat_i              (wb_m2s_ddr2_dat),
      .wb_sel_i              (wb_m2s_ddr2_sel),
      .wb_stb_i              (wb_m2s_ddr2_stb),
      .wb_we_i               (wb_m2s_ddr2_we),
      .wb_ack_o              (wb_s2m_ddr2_ack),
      .wb_err_o              (wb_s2m_ddr2_err),
      .wb_rty_o              (wb_s2m_ddr2_rty),
      .wb_dat_o              (wb_s2m_ddr2_dat),

      .wbc_adr_i             (wb_m2s_ddr2_cfg0_adr[5:2]),
      .wbc_dat_i             (wb_m2s_ddr2_cfg0_dat),
      .wbc_we_i              (wb_m2s_ddr2_cfg0_we),
      .wbc_cyc_i             (wb_m2s_ddr2_cfg0_cyc),
      .wbc_stb_i             (wb_m2s_ddr2_cfg0_stb),
      .wbc_dat_o             (wb_s2m_ddr2_cfg0_dat),
      .wbc_ack_o             (wb_s2m_ddr2_cfg0_ack),
      .wbc_err_o             (wb_s2m_ddr2_cfg0_err),
      .wbc_rty_o             (wb_s2m_ddr2_cfg0_rty),

      .sys_clk_i             (ddr2_if_ref_clk),
      //.sys_clk_i             (ddr2_if_sys_clk),
      .clk_ref_i             (ddr2_if_ref_clk),
      .init_calib_complete_o (ddr2_init_calib_complete),

      .app_addr              (app_addr),
      .app_cmd               (app_cmd),
      .app_en                (app_en),
      .app_wdf_data          (app_wdf_data),
      .app_wdf_end           (app_wdf_end),
      .app_wdf_mask          (app_wdf_mask),
      .app_wdf_wren          (app_wdf_wren),
      .app_rd_data           (app_rd_data),
      .app_rd_data_end       (app_rd_data_end),
      .app_rd_data_valid     (app_rd_data_valid),
      .app_rdy               (app_rdy),
      .app_wdf_rdy           (app_wdf_rdy),
      .state_o               (ddr2_state),
      .address_burst_o       (ddr2_address_burst),
      .miss_o                (ddr2_miss),

      .ddr2_addr             (ddr2_addr[12:0]),
      .ddr2_ba               (ddr2_ba),
      .ddr2_ras_n            (ddr2_ras_n),
      .ddr2_cas_n            (ddr2_cas_n),
      .ddr2_we_n             (ddr2_we_n),
      .ddr2_odt              (ddr2_odt[0]),
      .ddr2_cke              (ddr2_cke[0]),
      .ddr2_dm               (ddr2_dm),
      .ddr2_cs_n             (ddr2_cs_n[0]),
      .ddr2_ck_p             (ddr2_ck_p[0]),
      .ddr2_ck_n             (ddr2_ck_n[0]),
      .ddr2_dq               (ddr2_dq),
      .ddr2_dqs_p            (ddr2_dqs_p),
      .ddr2_dqs_n            (ddr2_dqs_n)
      );

`else // !`ifdef DDR2
   assign ddr2_init_calib_complete = 1'b0;
   assign wb_s2m_ddr2_dat = 0;
   assign wb_s2m_ddr2_ack = 0;
   assign wb_s2m_ddr2_err = 0;
   assign wb_s2m_ddr2_rty = 0;
   assign wb_s2m_ddr2_cfg0_dat = 0;
   assign wb_s2m_ddr2_cfg0_ack = 0;
   assign wb_s2m_ddr2_cfg0_err = 0;
   assign wb_s2m_ddr2_cfg0_rty = 0;
`endif // !`ifdef DDR2

   ////////////////////////////////////////////////////////////////////////
   // UART0
   ////////////////////////////////////////////////////////////////////////

   wire                 uart_irq;
`ifdef UART0
   uart_top uart16550_0
     (
      // Wishbone slave interface
      .wb_clk_i       (wb_clk),
      .wb_rst_i       (wb_rst),
      .wb_adr_i       (wb_m2s_uart0_adr[uart0_aw-1:0]),
      .wb_dat_i       (wb_m2s_uart0_dat),
      .wb_we_i        (wb_m2s_uart0_we),
      .wb_stb_i       (wb_m2s_uart0_stb),
      .wb_cyc_i       (wb_m2s_uart0_cyc),
      .wb_sel_i       (4'b0), // Not used in 8-bit mode
      .wb_dat_o       (wb_s2m_uart0_dat),
      .wb_ack_o       (wb_s2m_uart0_ack),

      // Outputs
      .int_o          (uart_irq),
      .stx_pad_o      (uart_txd_o),
      .rts_pad_o      (uart_rts_o),
      .dtr_pad_o      (),

      // Inputs
      .srx_pad_i      (uart_rxd_i),
      .cts_pad_i      (uart_cts_i),
      .dsr_pad_i      (1'b0),
      .ri_pad_i       (1'b0),
      .dcd_pad_i      (1'b0)
      );
`else // !`ifdef UART0
   assign uart_irq = 1'b0;
   assign wb_s2m_uart0_dat = 0;
   assign wb_s2m_uart0_ack = 0;
`endif // !`ifdef UART0
   assign wb_s2m_uart0_err = 0;
   assign wb_s2m_uart0_rty = 0;

   ////////////////////////////////////////////////////////////////////////
   // On-board temperature sensor
   ////////////////////////////////////////////////////////////////////////

`ifdef I2C_TEMP
   wire i2c_temp_int;
   wire i2c_temp_scl_oen;
   wire i2c_temp_sda_oen;
   wire i2c_temp_scl_o;
   wire i2c_temp_sda_o;
   assign i2c_temp_scl_io = i2c_temp_scl_oen ? 1'bz : i2c_temp_scl_o;
   assign i2c_temp_sda_io = i2c_temp_sda_oen ? 1'bz : i2c_temp_sda_o;

   i2c_master_top i2c_temp
     (
      // wishbone interface
      .wb_clk_i       (wb_clk),
      .wb_rst_i       (wb_rst),
      .arst_i         (~async_rst),
      .wb_adr_i       (wb_m2s_i2c_temp_adr[2:0]),
      .wb_dat_i       (wb_m2s_i2c_temp_dat),
      .wb_dat_o       (wb_s2m_i2c_temp_dat),
      .wb_we_i        (wb_m2s_i2c_temp_we),
      .wb_stb_i       (wb_m2s_i2c_temp_stb),
      .wb_cyc_i       (wb_m2s_i2c_temp_cyc),
      .wb_ack_o       (wb_s2m_i2c_temp_ack),
      .wb_inta_o      (i2c_temp_int),

      // i2c signals
      .scl_pad_i      (i2c_temp_scl_io),
      .scl_pad_o      (i2c_temp_scl_o),
      .scl_padoen_o   (i2c_temp_scl_oen),
      .sda_pad_i      (i2c_temp_sda_io),
      .sda_pad_o      (i2c_temp_sda_o),
      .sda_padoen_o   (i2c_temp_sda_oen)
      );
`else
   assign i2c_temp_scl_io = 1'bz;
   assign i2c_temp_sda_io = 1'bz;
   assign wb_s2m_i2c_temp_dat = 0;
   assign wb_s2m_i2c_temp_ack = 0;
`endif
   assign wb_s2m_i2c_temp_err = 0;
   assign wb_s2m_i2c_temp_rty = 0;

   ////////////////////////////////////////////////////////////////////////
   // SPI controller for Acceleration Sensor
   ////////////////////////////////////////////////////////////////////////

`ifdef SPI_ACCEL
   wire                 spi_accel_irq;

   assign  wb_s2m_spi_accel_err = 0;
   assign  wb_s2m_spi_accel_rty = 0;
   assign  spi_accel_hold_n_o = 1;
   assign  spi_accel_w_n_o = 1;

   simple_spi spi_accel
     (
      // Wishbone slave interface
      .clk_i  (wb_clk),
      .rst_i  (wb_rst),
      .adr_i  (wb_m2s_spi_accel_adr[2:0]),
      .dat_i  (wb_m2s_spi_accel_dat),
      .we_i   (wb_m2s_spi_accel_we),
      .stb_i  (wb_m2s_spi_accel_stb),
      .cyc_i  (wb_m2s_spi_accel_cyc),
      .dat_o  (wb_s2m_spi_accel_dat),
      .ack_o  (wb_s2m_spi_accel_ack),

      // Outputs
      .inta_o         (spi_accel_irq),
      .sck_o          (spi_accel_sclk_o),
      .ss_o           (spi_accel_cs_n_o),
      .mosi_o         (spi_accel_mosi_o),

      // Inputs
      .miso_i         (spi_accel_miso_i)
      );
`else
   assign wb_s2m_spi_accel_dat = 0;
   assign wb_s2m_spi_accel_ack = 0;
`endif //  `ifdef SPI_ACCEL
   assign wb_s2m_spi_accel_err = 0;
   assign wb_s2m_spi_accel_rty = 0;

   ////////////////////////////////////////////////////////////////////////
   // GPIO, switch inputs
   ////////////////////////////////////////////////////////////////////////

`ifdef GPIO_SW
   wb_gpio #(.n_bits(16)) gpio_sw
     (
      // GPIO bus
      .gpio_i         (sw_i),
      .gpio_o         (),
      .gpio_dir_o     (),
      // Wishbone slave interface
      .wb_adr_i       (wb_m2s_gpio_sw_adr[2]),
      .wb_dat_i       (wb_m2s_gpio_sw_dat),
      .wb_we_i        (wb_m2s_gpio_sw_we),
      .wb_cyc_i       (wb_m2s_gpio_sw_cyc),
      .wb_stb_i       (wb_m2s_gpio_sw_stb),
      .wb_cti_i       (wb_m2s_gpio_sw_cti),
      .wb_bte_i       (wb_m2s_gpio_sw_bte),
      .wb_dat_o       (wb_s2m_gpio_sw_dat),
      .wb_ack_o       (wb_s2m_gpio_sw_ack),
      .wb_err_o       (wb_s2m_gpio_sw_err),
      .wb_rty_o       (wb_s2m_gpio_sw_rty),

      .wb_clk         (wb_clk),
      .wb_rst         (wb_rst)
      );
`else // !`ifdef GPIO_SW
   assign wb_s2m_gpio_sw_dat = 0;
   assign wb_s2m_gpio_sw_ack = 0;
   assign wb_s2m_gpio_sw_err = 0;
   assign wb_s2m_gpio_sw_rty = 0;
`endif // !`ifdef GPIO_SW

   ////////////////////////////////////////////////////////////////////////
   // GPIO, push-button inputs
   ////////////////////////////////////////////////////////////////////////

`ifdef GPIO_PBTN
   wb_gpio #(.n_bits(5)) gpio_pbtn
     (
      // GPIO bus
      .gpio_i         (pbtn_i),
      .gpio_o         (),
      .gpio_dir_o     (),
      // Wishbone slave interface
      .wb_adr_i       (wb_m2s_gpio_pbtn_adr[2]),
      .wb_dat_i       (wb_m2s_gpio_pbtn_dat),
      .wb_we_i        (wb_m2s_gpio_pbtn_we),
      .wb_cyc_i       (wb_m2s_gpio_pbtn_cyc),
      .wb_stb_i       (wb_m2s_gpio_pbtn_stb),
      .wb_cti_i       (wb_m2s_gpio_pbtn_cti),
      .wb_bte_i       (wb_m2s_gpio_pbtn_bte),
      .wb_dat_o       (wb_s2m_gpio_pbtn_dat),
      .wb_ack_o       (wb_s2m_gpio_pbtn_ack),
      .wb_err_o       (wb_s2m_gpio_pbtn_err),
      .wb_rty_o       (wb_s2m_gpio_pbtn_rty),

      .wb_clk         (wb_clk),
      .wb_rst         (wb_rst)
      );
`else // !`ifdef GPIO_PBTN
   assign wb_s2m_gpio_pbtn_dat = 0;
   assign wb_s2m_gpio_pbtn_ack = 0;
   assign wb_s2m_gpio_pbtn_err = 0;
   assign wb_s2m_gpio_pbtn_rty = 0;
`endif // !`ifdef GPIO_PBTN

   ////////////////////////////////////////////////////////////////////////
   // GPIO, LED outputs
   ////////////////////////////////////////////////////////////////////////

`ifdef GPIO_LED
   wire [15:0]           gpio_led_in;
   wire [15:0]           gpio_led_out;
   wire [15:0]           gpio_led_dir;

   // Tristate logic for IO
   // 0 = input, 1 = output
   generate
      for (i = 0; i < 16; i = i+1) begin: gpio_led_tris
         assign led_o      [i] = gpio_led_dir[i] ? gpio_led_out[i] | led_net[i] : led_net[i];
         assign gpio_led_in[i] = gpio_led_dir[i] ? gpio_led_out[i] | led_net[i] : led_net[i];
      end
   endgenerate

   wb_gpio #(.n_bits(16)) gpio_led
     (
      // GPIO bus
      .gpio_i         (gpio_led_in),
      .gpio_o         (gpio_led_out),
      .gpio_dir_o     (gpio_led_dir),
      // Wishbone slave interface
      .wb_adr_i       (wb_m2s_gpio_led_adr[2]),
      .wb_dat_i       (wb_m2s_gpio_led_dat),
      .wb_we_i        (wb_m2s_gpio_led_we),
      .wb_cyc_i       (wb_m2s_gpio_led_cyc),
      .wb_stb_i       (wb_m2s_gpio_led_stb),
      .wb_cti_i       (wb_m2s_gpio_led_cti),
      .wb_bte_i       (wb_m2s_gpio_led_bte),
      .wb_dat_o       (wb_s2m_gpio_led_dat),
      .wb_ack_o       (wb_s2m_gpio_led_ack),
      .wb_err_o       (wb_s2m_gpio_led_err),
      .wb_rty_o       (wb_s2m_gpio_led_rty),

      .wb_clk         (wb_clk),
      .wb_rst         (wb_rst)
      );
`else // !`ifdef GPIO_LED
   assign wb_s2m_gpio_led_dat = 0;
   assign wb_s2m_gpio_led_ack = 0;
   assign wb_s2m_gpio_led_err = 0;
   assign wb_s2m_gpio_led_rty = 0;
`endif // !`ifdef GPIO_LED

   ////////////////////////////////////////////////////////////////////////
   // GPIO, Pmod Connectors
   ////////////////////////////////////////////////////////////////////////

`ifdef GPIO_PMOD
   wire [31:0]           gpio_pmod_in;
   wire [31:0]           gpio_pmod_out;
   wire [31:0]           gpio_pmod_dir;

   // Tristate logic for IO
   // 0 = input, 1 = output
   generate
      for (i =  0; i <  8; i = i+1) begin: gpio_pmod_tris_ja
         assign jd_io       [i]    = gpio_pmod_dir[i] ? gpio_pmod_out[i] | jd_net[i]    : 1'bz;
         assign gpio_pmod_in[i]    = gpio_pmod_dir[i] ? gpio_pmod_out[i]                : jd_io [i];
      end
      for (i =  8; i < 16; i = i+1) begin: gpio_pmod_tris_jb
         assign jc_io       [i-8]  = gpio_pmod_dir[i] ? gpio_pmod_out[i] | jc_net[i-8]  : 1'bz;
         assign gpio_pmod_in[i]    = gpio_pmod_dir[i] ? gpio_pmod_out[i]                : jc_io [i-8];
      end
      for (i = 16; i < 24; i = i+1) begin: gpio_pmod_tris_jc
         assign jb_io       [i-16] = gpio_pmod_dir[i] ? gpio_pmod_out[i] | jb_net[i-16] : 1'bz;
         assign gpio_pmod_in[i]    = gpio_pmod_dir[i] ? gpio_pmod_out[i]                : jb_io [i-16];
      end
      for (i = 24; i < 32; i = i+1) begin: gpio_pmod_tris_jd
         assign ja_io       [i-24] = gpio_pmod_dir[i] ? gpio_pmod_out[i] | ja_net[i-24] : 1'bz;
         assign gpio_pmod_in[i]    = gpio_pmod_dir[i] ? gpio_pmod_out[i]                : ja_io [i-24];
      end
   endgenerate

   wb_gpio #(.n_bits(32)) gpio_pmod
     (
      // GPIO bus
      .gpio_i         (gpio_pmod_in),
      .gpio_o         (gpio_pmod_out),
      .gpio_dir_o     (gpio_pmod_dir),
      // Wishbone slave interface
      .wb_adr_i       (wb_m2s_gpio_pmod_adr[2]),
      .wb_dat_i       (wb_m2s_gpio_pmod_dat),
      .wb_we_i        (wb_m2s_gpio_pmod_we),
      .wb_cyc_i       (wb_m2s_gpio_pmod_cyc),
      .wb_stb_i       (wb_m2s_gpio_pmod_stb),
      .wb_cti_i       (wb_m2s_gpio_pmod_cti),
      .wb_bte_i       (wb_m2s_gpio_pmod_bte),
      .wb_dat_o       (wb_s2m_gpio_pmod_dat),
      .wb_ack_o       (wb_s2m_gpio_pmod_ack),
      .wb_err_o       (wb_s2m_gpio_pmod_err),
      .wb_rty_o       (wb_s2m_gpio_pmod_rty),

      .wb_clk         (wb_clk),
      .wb_rst         (wb_rst)
      );
`else // !`ifdef GPIO_PMOD
   assign wb_s2m_gpio_pmod_dat = 0;
   assign wb_s2m_gpio_pmod_ack = 0;
   assign wb_s2m_gpio_pmod_err = 0;
   assign wb_s2m_gpio_pmod_rty = 0;
`endif // !`ifdef GPIO_PMOD

   ////////////////////////////////////////////////////////////////////////
   // Seven-segment LED display controller
   ////////////////////////////////////////////////////////////////////////

`ifdef SSEG_DISP
   // open-drain output with logic inversion
   wire [7:0]            sseg_disp_seg_oe;
   generate
      for (i = 0; i < 8; i = i+1) begin: sseg_disp_seg_tris
         assign sseg_disp_seg_o [i] = sseg_disp_seg_oe[i] ? 1'b0 : 1'bz;
      end
   endgenerate

   wire [7:0] sseg_disp_seg_sel;
   assign sseg_disp_an_o = ~sseg_disp_seg_sel;

   wb_sseg_ctrl
     #(
       .n_digits   (8),
`ifdef SIM
       .def_clk_div(4) // speed up simulation time
`else
       .def_clk_div(128)
`endif
       )
   sseg_ctrl0
     (
      .wb_clk_i       (wb_clk),
      .wb_rst_i       (wb_rst),
      .async_rst_i    (async_rst),
      // Wishbone slave interface
      .wb_adr_i       (wb_m2s_sseg_ctrl_adr[5:2]),
      .wb_dat_i       (wb_m2s_sseg_ctrl_dat),
      .wb_sel_i       (wb_m2s_sseg_ctrl_sel),
      .wb_we_i        (wb_m2s_sseg_ctrl_we),
      .wb_cyc_i       (wb_m2s_sseg_ctrl_cyc),
      .wb_stb_i       (wb_m2s_sseg_ctrl_stb),
      .wb_cti_i       (wb_m2s_sseg_ctrl_cti),
      .wb_bte_i       (wb_m2s_sseg_ctrl_bte),
      .wb_dat_o       (wb_s2m_sseg_ctrl_dat),
      .wb_ack_o       (wb_s2m_sseg_ctrl_ack),
      .wb_err_o       (wb_s2m_sseg_ctrl_err),
      .wb_rty_o       (wb_s2m_sseg_ctrl_rty),
      // display i/f
      .seg_o          (sseg_disp_seg_oe),
      .seg_sel_o      (sseg_disp_seg_sel),
      // frame sync irq
      .irq_o          ()
      );
`else // !`ifdef SSEG_DISP
   assign sseg_disp_seg_o = 8'bzzzz_zzzz;
   assign sseg_disp_an_o  = 8'b1111_1111;
   assign wb_s2m_sseg_ctrl_dat = 0;
   assign wb_s2m_sseg_ctrl_ack = 0;
   assign wb_s2m_sseg_ctrl_err = 0;
   assign wb_s2m_sseg_ctrl_rty = 0;
`endif // !`ifdef SSEG_DISP

   ////////////////////////////////////////////////////////////////////////
   // RGB LED controller
   ////////////////////////////////////////////////////////////////////////

`ifdef RGB_LED
   wb_rgb_led
     #(
       .n_leds     (2),
       .depth      (8),
       .dw         (24),
       .aw         (2),
`ifdef SIM
       .def_clk_div(4) // speed up simulation time
`else
       .def_clk_div(128)
`endif
       )
   rgb_leds
     (
      .wb_clk_i       (wb_clk),
      .wb_rst_i       (wb_rst),
      .async_rst_i    (async_rst),
      // Wishbone slave interface
      .wb_adr_i       (wb_m2s_rgb_led_adr[3:2]),
      .wb_dat_i       (wb_m2s_rgb_led_dat),
      .wb_sel_i       (wb_m2s_rgb_led_sel),
      .wb_we_i        (wb_m2s_rgb_led_we),
      .wb_cyc_i       (wb_m2s_rgb_led_cyc),
      .wb_stb_i       (wb_m2s_rgb_led_stb),
      .wb_cti_i       (wb_m2s_rgb_led_cti),
      .wb_bte_i       (wb_m2s_rgb_led_bte),
      .wb_dat_o       (wb_s2m_rgb_led_dat),
      .wb_ack_o       (wb_s2m_rgb_led_ack),
      .wb_err_o       (wb_s2m_rgb_led_err),
      .wb_rty_o       (wb_s2m_rgb_led_rty),
      // LED driver
      .rgb_led_o      ({led_rgb2_red_o, led_rgb2_green_o, led_rgb2_blue_o,
                        led_rgb1_red_o, led_rgb1_green_o, led_rgb1_blue_o})
      );
`else // !`ifdef RGB_LED
   assign wb_s2m_rgb_led_dat = 0;
   assign wb_s2m_rgb_led_ack = 0;
   assign wb_s2m_rgb_led_err = 0;
   assign wb_s2m_rgb_led_rty = 0;
`endif //  `ifdef RGB_LED

   ////////////////////////////////////////////////////////////////////////
   // VGA/LCD
   ////////////////////////////////////////////////////////////////////////

`ifdef VGA0
   wire            vga0_irq;
   wire            vga_pclk;
   wire            vga_pclk_buf;
   wire [7:0]      vga_r;
   wire [7:0]      vga_g;
   wire [7:0]      vga_b;
   wire            blank;
   wire            gate;
   reg [15:0]      hlen;
   reg [15:0]      vlen;

   vga_enh_top
     #(
       .LINE_FIFO_AWIDTH(10)
       )
   vga0
     (
      .wb_clk_i       (wb_clk),
      .wb_rst_i       (wb_rst),
      .rst_i          (1'b1),
      .wb_inta_o      (vga0_irq),
      // Wishbone slave connections
      .wbs_adr_i      (wb_m2s_vga0_adr),
      .wbs_dat_i      (wb_m2s_vga0_dat),
      .wbs_sel_i      (wb_m2s_vga0_sel),
      .wbs_we_i       (wb_m2s_vga0_we),
      .wbs_stb_i      (wb_m2s_vga0_stb),
      .wbs_cyc_i      (wb_m2s_vga0_cyc),
      .wbs_dat_o      (wb_s2m_vga0_dat),
      .wbs_ack_o      (wb_s2m_vga0_ack),
      .wbs_rty_o      (wb_s2m_vga0_rty),
      .wbs_err_o      (wb_s2m_vga0_err),
      // Wishbone master connections
      .wbm_adr_o      (wb_m2s_vga0_master_adr),
      .wbm_cti_o      (wb_m2s_vga0_master_cti),
      .wbm_bte_o      (wb_m2s_vga0_master_bte),
      .wbm_sel_o      (wb_m2s_vga0_master_sel),
      .wbm_we_o       (wb_m2s_vga0_master_we),
      .wbm_stb_o      (wb_m2s_vga0_master_stb),
      .wbm_cyc_o      (wb_m2s_vga0_master_cyc),
      .wbm_dat_i      (wb_s2m_vga0_master_dat),
      .wbm_ack_i      (wb_s2m_vga0_master_ack),
      .wbm_err_i      (wb_s2m_vga0_master_err),
      .clk_p_i        (vga_pclk_buf),
      .clk_p_o        (),
      .hsync_pad_o    (vga_hs_o),
      .vsync_pad_o    (vga_vs_o),
      .csync_pad_o    (),
      .blank_pad_o    (vga_blank),
      .r_pad_o        (vga_r),
      .g_pad_o        (vga_g),
      .b_pad_o        (vga_b)
      );

   assign vga_red_o   = vga_r[7:4];
   assign vga_green_o = vga_g[7:4];
   assign vga_blue_o  = vga_b[7:4];

   wire vga_clkfb;
   MMCM_ADV #
     (
      .BANDWIDTH            ("OPTIMIZED"),
      .CLKOUT4_CASCADE      ("FALSE"),
      .CLOCK_HOLD           ("FALSE"),
      .COMPENSATION         ("ZHOLD"),
      .STARTUP_WAIT         ("FALSE"),
      .DIVCLK_DIVIDE        (1),
      .CLKFBOUT_MULT_F      (16.000), // 800MHz
      .CLKFBOUT_PHASE       (0.000),
      .CLKFBOUT_USE_FINE_PS ("FALSE"),
      .CLKOUT0_DIVIDE_F     (32.000), // 25MHz VGA 640x480@60 Hz | 65MHz - 1024x768, 60Hz
      .CLKOUT0_PHASE        (0.000), //90.000
      .CLKOUT0_DUTY_CYCLE   (0.500),
      .CLKOUT0_USE_FINE_PS  ("FALSE"),
      .CLKIN1_PERIOD        (20.000),
      .REF_JITTER1          (0.010))
   mmcm_adv_vga
     (
      // Output clocks
      .CLKFBOUT             (vga_clkfb),
      .CLKFBOUTB            (),
      .CLKOUT0              (vga_pclk),
      .CLKOUT0B             (),
      .CLKOUT1              (),
      .CLKOUT1B             (),
      .CLKOUT2              (),
      .CLKOUT2B             (),
      .CLKOUT3              (),
      .CLKOUT3B             (),
      .CLKOUT4              (),
      .CLKOUT5              (),
      .CLKOUT6              (),
      // Input clock control
      .CLKFBIN              (vga_clkfb),
      .CLKIN1               (wb_clk),
      .CLKIN2               (1'b0),
      // Tied to always select the primary input clock
      .CLKINSEL             (1'b1),
      // Ports for dynamic reconfiguration
      .DADDR                (7'h0),
      .DCLK                 (1'b0),
      .DEN                  (1'b0),
      .DI                   (16'h0),
      .DO                   (),
      .DRDY                 (),
      .DWE                  (1'b0),
      // Ports for dynamic phase shift
      .PSCLK                (1'b0),
      .PSEN                 (1'b0),
      .PSINCDEC             (1'b0),
      .PSDONE               (),
      // Other control and status signals
      .LOCKED               (),
      .CLKINSTOPPED         (),
      .CLKFBSTOPPED         (),
      .PWRDWN               (1'b0),
      .RST                  ()
      );

      // clock buffers
   BUFG bufg_vga_pclk
     (
      .O (vga_pclk_buf),
      .I (vga_pclk)
      );

   //assign vga_pclk = wb_clk;
`else
   wire   vga0_irq    = 1'b0;
   assign vga_pclk    = 1'b0;
   assign vga_red_o   = 4'b0;
   assign vga_green_o = 4'b0;
   assign vga_blue_o  = 4'b0;
   assign vga_hs_o    = 1'b0;
   assign vga_vs_o    = 1'b0;

   assign wb_s2m_vga0_dat = 0;
   assign wb_s2m_vga0_ack = 0;
   assign wb_s2m_vga0_err = 0;
   assign wb_s2m_vga0_rty = 0;
   assign wb_m2s_vga0_master_adr = 0;
   assign wb_m2s_vga0_master_cti = 0;
   assign wb_m2s_vga0_master_bte = 0;
   assign wb_m2s_vga0_master_we = 0;
   assign wb_m2s_vga0_master_stb = 0;
   assign wb_m2s_vga0_master_cyc = 0;
`endif

   ////////////////////////////////////////////////////////////////////////
   // Ethernet
   ////////////////////////////////////////////////////////////////////////

`ifdef ETH0
   wire            eth0_irq;
   wire            eth0_100m = 1'b1;

   // MII
   // Tx
   wire [3:0]      eth0_mii_txd;
   wire            eth0_mii_tx_en;
   wire            eth0_mii_tx_er;
   wire            eth0_mii_tx_clk;
   // Rx
   wire            eth0_mii_rx_clk;
   wire [3:0]      eth0_mii_rxd;
   wire            eth0_mii_rx_dv;
   wire            eth0_mii_rx_er;
   wire            eth0_mii_coll;
   wire            eth0_mii_crs;

   //wire            eth0_speed;
   //wire            eth0_duplex;
   //wire            eth0_link;
   // Management interface wires
   wire            eth0_mdio_i;
   wire            eth0_mdio_o;
   wire            eth0_mdio_oe;

   // Tristate control for management interface
   assign eth0_mdio_io  = eth0_mdio_oe ? eth0_mdio_o : 1'bz;
   assign eth0_mdio_i   = eth0_mdio_io;

   //assign eth0_refclk_o = ~wb_clk; // 50MHz, 180Deg
   assign eth0_rst_n_o  = ~wb_rst;

   ethmac ethmac0
     (
      // Wishbone Slave interface
      .wb_clk_i       (wb_clk),
      .wb_rst_i       (wb_rst),
      .wb_adr_i       (wb_m2s_eth0_adr[11:2]),
      .wb_dat_i       (wb_m2s_eth0_dat),
      .wb_sel_i       (wb_m2s_eth0_sel),
      .wb_we_i        (wb_m2s_eth0_we),
      .wb_cyc_i       (wb_m2s_eth0_cyc),
      .wb_stb_i       (wb_m2s_eth0_stb),
      .wb_dat_o       (wb_s2m_eth0_dat),
      .wb_err_o       (wb_s2m_eth0_err),
      .wb_ack_o       (wb_s2m_eth0_ack),
      // Wishbone Master Interface
      .m_wb_adr_o     (wb_m2s_eth0_master_adr),
      .m_wb_sel_o     (wb_m2s_eth0_master_sel),
      .m_wb_we_o      (wb_m2s_eth0_master_we),
      .m_wb_dat_o     (wb_m2s_eth0_master_dat),
      .m_wb_cyc_o     (wb_m2s_eth0_master_cyc),
      .m_wb_stb_o     (wb_m2s_eth0_master_stb),
      .m_wb_cti_o     (wb_m2s_eth0_master_cti),
      .m_wb_bte_o     (wb_m2s_eth0_master_bte),
      .m_wb_dat_i     (wb_s2m_eth0_master_dat),
      .m_wb_ack_i     (wb_s2m_eth0_master_ack),
      .m_wb_err_i     (wb_s2m_eth0_master_err),

      // Ethernet MII interface
      // Transmit
      .mtxd_pad_o     (eth0_mii_txd),
      .mtxen_pad_o    (eth0_mii_tx_en),
      .mtxerr_pad_o   (eth0_mii_tx_er),
      .mtx_clk_pad_i  (eth0_mii_tx_clk),
      // Receive
      .mrx_clk_pad_i  (eth0_mii_rx_clk),
      .mrxd_pad_i     (eth0_mii_rxd),
      .mrxdv_pad_i    (eth0_mii_rx_dv),
      .mrxerr_pad_i   (eth0_mii_rx_er),
      .mcoll_pad_i    (eth0_mii_coll),
      .mcrs_pad_i     (eth0_mii_crs),
      // Management interface
      .md_pad_i       (eth0_mdio_i),
      .mdc_pad_o      (eth0_mdc_o),
      .md_pad_o       (eth0_mdio_o),
      .md_padoe_o     (eth0_mdio_oe),

      // Processor interrupt
      .int_o          (eth0_irq)
      );

/* -----\/----- EXCLUDED -----\/-----
   // MII to RMII adapter
   mii2rmii mii2rmii_0
     (
      .clk_i         (wb_clk),
      .rst_i         (wb_rst),
      .async_rst_i   (async_rst),
      .en_100M       (eth0_en_100m),

      // MII (MAC) side
      // Tx
      .mii_txd_i     (eth0_mii_txd),
      .mii_tx_en_i   (eth0_mii_tx_en),
      .mii_tx_er_i   (eth0_mii_tx_er),
      .mii_tx_clk_o  (eth0_mii_tx_clk),
      // Rx
      .mii_rx_clk_o  (eth0_mii_rx_clk),
      .mii_rxd_o     (eth0_mii_rxd),
      .mii_rx_dv_o   (eth0_mii_rx_dv),
      .mii_rx_er_o   (eth0_mii_rx_er),
      .mii_coll_o    (eth0_mii_coll),
      .mii_crs_o     (eth0_mii_crs),

      // RMII (PHY) side
      // Tx
      .rmii_txd_o    (eth0_txd_o),
      .rmii_tx_en_o  (eth0_tx_en_o),
      // Rx
      .rmii_rxd_i    (eth0_rxd_i),
      .rmii_rx_er_i  (eth0_rx_er_i),
      .rmii_crs_dv_i (eth0_crs_dv_i)
      );
 -----/\----- EXCLUDED -----/\----- */
 `ifndef SIM
   // Xilinx MII to RMII adapter (VHDL)
   mii_to_rmii
   mii2rmii_0
     (
      .ref_clk           (wb_clk),
      .rst_n             (~wb_rst),

      // MII (MAC) side
      // Tx
      .mac2rmii_txd      (eth0_mii_txd),
      .mac2rmii_tx_en    (eth0_mii_tx_en),
      .mac2rmii_tx_er    (eth0_mii_tx_er),
      .rmii2mac_tx_clk   (eth0_mii_tx_clk),
      // Rx
      .rmii2mac_rx_clk   (eth0_mii_rx_clk),
      .rmii2mac_rxd      (eth0_mii_rxd),
      .rmii2mac_rx_dv    (eth0_mii_rx_dv),
      .rmii2mac_rx_er    (eth0_mii_rx_er),
      .rmii2mac_col      (eth0_mii_coll),
      .rmii2mac_crs      (eth0_mii_crs),

      // RMII (PHY) side
      // Tx
      .rmii2phy_txd      (eth0_txd_o),
      .rmii2phy_tx_en    (eth0_tx_en_o),
      // Rx
      .phy2rmii_rxd      (eth0_rxd_i),
      .phy2rmii_rx_er    (eth0_rx_er_i),
      .phy2rmii_crs_dv   (eth0_crs_dv_i)
      );
 `endif // !`ifdef SIM

   MMCM_ADV #
     (
      .BANDWIDTH            ("OPTIMIZED"),
      .CLKOUT4_CASCADE      ("FALSE"),
      .CLOCK_HOLD           ("FALSE"),
      .COMPENSATION         ("ZHOLD"),
      .STARTUP_WAIT         ("FALSE"),
      .DIVCLK_DIVIDE        (1),
      .CLKFBOUT_MULT_F      (16.000), // 800MHz
      .CLKFBOUT_PHASE       (0.000),
      .CLKFBOUT_USE_FINE_PS ("FALSE"),
      .CLKOUT0_DIVIDE_F     (16.000), // 50MHz
      .CLKOUT0_PHASE        (45.000), //90.000
      .CLKOUT0_DUTY_CYCLE   (0.500),
      .CLKOUT0_USE_FINE_PS  ("FALSE"),
      .CLKIN1_PERIOD        (20.000),
      .REF_JITTER1          (0.010))
   mmcm_adv_0
     (
      // Output clocks
      .CLKFBOUT             (clkfb),
      .CLKFBOUTB            (),
      .CLKOUT0              (eth0_refclk_o),
      .CLKOUT0B             (),
      .CLKOUT1              (),
      .CLKOUT1B             (),
      .CLKOUT2              (),
      .CLKOUT2B             (),
      .CLKOUT3              (),
      .CLKOUT3B             (),
      .CLKOUT4              (),
      .CLKOUT5              (),
      .CLKOUT6              (),
      // Input clock control
      .CLKFBIN              (clkfb_buf),
      .CLKIN1               (wb_clk),
      .CLKIN2               (1'b0),
      // Tied to always select the primary input clock
      .CLKINSEL             (1'b1),
      // Ports for dynamic reconfiguration
      .DADDR                (7'h0),
      .DCLK                 (1'b0),
      .DEN                  (1'b0),
      .DI                   (16'h0),
      .DO                   (),
      .DRDY                 (),
      .DWE                  (1'b0),
      // Ports for dynamic phase shift
      .PSCLK                (1'b0),
      .PSEN                 (1'b0),
      .PSINCDEC             (1'b0),
      .PSDONE               (),
      // Other control and status signals
      .LOCKED               (),
      .CLKINSTOPPED         (),
      .CLKFBSTOPPED         (),
      .PWRDWN               (1'b0),
      .RST                  ()
      );

      // clock buffers
   BUFG bufg_clkfb
     (
      .O (clkfb_buf),
      .I (clkfb)
      );

`else
   assign eth0_irq = 0;
   assign eth0_txd_o = 0;
   assign eth0_tx_en_o = 0;
   assign eth0_mdc_pad_o = 0;
   assign eth0_md_pad_io = 0;
   assign eth0_rst_n_o = 0;
   assign wb_s2m_eth0_dat = 0;
   assign wb_s2m_eth0_ack = 0;
   assign wb_s2m_eth0_err = 0;
   assign wb_s2m_eth0_rty = 0;
   assign wb_m2s_eth0_master_adr = 0;
   assign wb_m2s_eth0_master_cti = 0;
   assign wb_m2s_eth0_master_bte = 0;
   assign wb_m2s_eth0_master_we = 0;
   assign wb_m2s_eth0_master_stb = 0;
   assign wb_m2s_eth0_master_cyc = 0;
   assign wb_m2s_eth0_master_dat = 0;
`endif

   wire            ps2_0_irq;

   assign wb_s2m_ps2_0_err = 0;
   assign wb_s2m_ps2_0_rty = 0;

   ////////////////////////////////////////////////////////////////////////
   // PS2_0
   ////////////////////////////////////////////////////////////////////////

`ifdef PS2_0
   wire            ps2_0_cycstb = wb_m2s_ps2_0_cyc & wb_m2s_ps2_0_stb;

   ps2_wb ps2_0
     (
      .wb_clk_i       (wb_clk),
      .wb_rst_i       (wb_rst),
      .wb_dat_i       (wb_m2s_ps2_0_dat),
      .wb_dat_o       (wb_s2m_ps2_0_dat),
      .wb_adr_i       (wb_m2s_ps2_0_adr[0]),
      .wb_stb_i       (ps2_0_cycstb),
      .wb_we_i        (wb_m2s_ps2_0_we),
      .wb_ack_o       (wb_s2m_ps2_0_ack),

      // IRQ output
      .irq_o          (ps2_0_irq),

      // PS2 signals
      .ps2_clk        (ps2_clk),
      .ps2_dat        (ps2_dat)
      );
`else
   assign ps2_0_irq = 0;
   assign wb_s2m_ps2_0_dat = 0;
   assign wb_s2m_ps2_0_ack = 0;
`endif // PS2_0
   assign wb_s2m_ps2_0_err = 0;
   assign wb_s2m_ps2_0_rty = 0;


   ////////////////////////////////////////////////////////////////////////
   // Xilinx XADC
   ////////////////////////////////////////////////////////////////////////

`ifdef XADC0
   wire xadc0_irq;

   nexys4ddr_xadc_wb
     #(
       .NEXYS4DDR_XADC_SIM_MONITOR_FILE (NEXYS4DDR_XADC_SIM_MONITOR_FILE),
       .dw(16),
       .aw(8)
       )
   xadc_wb0
     (
      // Analogue inputs
      //.xa_p_i       (xa_p_i),
      //.xa_n_i       (xa_p_i),

      // Wishbone Interface
      .wb_clk_i      (wb_clk),
      .wb_rst_i      (wb_rst),
      .async_rst_i   (async_rst),

      .wb_adr_i      (wb_m2s_xadc0_adr[9:2]),
      .wb_dat_i      (wb_m2s_xadc0_dat),
      .wb_sel_i      (wb_m2s_xadc0_sel),
      .wb_we_i       (wb_m2s_xadc0_we),
      .wb_cyc_i      (wb_m2s_xadc0_cyc),
      .wb_stb_i      (wb_m2s_xadc0_stb),
      .wb_cti_i      (wb_m2s_xadc0_cti),
      .wb_bte_i      (wb_m2s_xadc0_bte),
      .wb_dat_o      (wb_s2m_xadc0_dat),
      .wb_ack_o      (wb_s2m_xadc0_ack),
      .wb_err_o      (wb_s2m_xadc0_err),
      .wb_rty_o      (wb_s2m_xadc0_rty),

      // xadc interrupts
      .irq_o         (xadc0_irq)
    );
`else // !`ifdef XADC0
   assign xadc0_irq = 1'b0;
   assign wb_s2m_xadc0_dat = 0;
   assign wb_s2m_xadc0_ack = 0;
   assign wb_s2m_xadc0_err = 0;
   assign wb_s2m_xadc0_rty = 0;
`endif // !`ifdef XADC0


   ////////////////////////////////////////////////////////////////////////
   // diila - Device Independent Integrated Logic Analyzer
   ////////////////////////////////////////////////////////////////////////

   wire [31:0] diila_trig;
   wire [31:0] diila_data [5:0];

   assign diila_trig = {
		        29'h0,
                        //mor1kx0.mor1kx_cpu.cappuccino.mor1kx_cpu.mor1kx_ctrl_cappuccino.exception,
                        exception,
		        wb_m2s_dbg_stb,
		        wb_m2s_dbg_cyc
		        };

   assign diila_data[0] = wb_m2s_ddr2_adr;
   assign diila_data[1] = wb_m2s_ddr2_dat;
   assign diila_data[2] = wb_s2m_ddr2_dat;
   assign diila_data[3] = {
			   5'h0,
			   wb_m2s_or1k_i_cyc,	// 1
			   wb_m2s_or1k_i_stb,	// 1
			   wb_m2s_or1k_d_cyc,	// 1
			   wb_m2s_or1k_d_stb,	// 1
			   wb_m2s_diila_cyc,	// 1
			   1'b0, //wb_m2s_eth0_cyc,	// 1
			   1'b0, //wb_m2s_vga0_cyc,	// 1
			   1'b0, //wb_m2s_spi0_cyc,	// 1
			   1'b0, //wb_m2s_gpio0_cyc,	// 1
			   wb_m2s_uart0_cyc,	// 1
			   wb_s2m_dbg_ack,		// 1
			   wb_m2s_dbg_we,		// 1
			   wb_m2s_ddr2_stb,	// 1
			   wb_m2s_ddr2_cyc,	// 1
			   wb_m2s_ddr2_bte,	// 2
			   wb_m2s_ddr2_cti,	// 3
			   wb_m2s_ddr2_sel,	// 4
			   wb_m2s_ddr2_we,	// 1
			   wb_s2m_ddr2_ack,	// 1
			   wb_s2m_ddr2_err,	// 1
			   wb_s2m_ddr2_rty	// 1
			   };
   assign diila_data[4] = {
                           app_rd_data[99:96],
                           app_rd_data[67:64],
                           app_rd_data[35:32],
                           app_rd_data[3:0],
                           6'b0,
                           app_en, // 1
                           app_rdy, // 1
                           app_cmd, // 3
                           app_wdf_wren, // 1
                           app_wdf_rdy, // 1
                           app_wdf_end, // 1
                           app_rd_data_valid, // 1
                           app_rd_data_end // 1
                           };
                          //wb_m2s_dbg_adr;
   assign diila_data[5] = {
                           ddr2_miss, // 1
                           ddr2_state, // 4
                           ddr2_address_burst // 27
                           //app_addr // 27
                           };

   diila #
     (
      .DATA_WIDTH(32*6)
      )
   diila
     (
      .wb_rst_i             (wb_rst),
      .wb_clk_i             (wb_clk),
      .wb_dat_i             (wb_m2s_diila_dat),
      .wb_adr_i             (wb_m2s_diila_adr[23:2]),
      .wb_sel_i             (wb_m2s_diila_sel),
      .wb_we_i              (wb_m2s_diila_we),
      .wb_cyc_i             (wb_m2s_diila_cyc),
      .wb_stb_i             (wb_m2s_diila_stb),
      .wb_dat_o             (wb_s2m_diila_dat),
      .wb_ack_o             (wb_s2m_diila_ack),
      .wb_err_o             (wb_s2m_diila_err),
      .wb_rty_o             (wb_s2m_diila_rty),
      .storage_en           (1'b1/*diila_storage_en*/),
      .trig_i               (diila_trig),
      .data_i               ({
			      diila_data[0],
			      diila_data[1],
			      diila_data[2],
			      diila_data[3],
			      diila_data[4],
			      diila_data[5]
			      })
      );

   wire [15:0] dbg_cnt_0_strobe =
               {
                4'b0,
                or1k_irq,
                exception,
                wb_m2s_or1k_d_cyc & ~wb_m2s_or1k_d_we & wb_s2m_or1k_d_ack,
                wb_m2s_or1k_d_cyc &  wb_m2s_or1k_d_we & wb_s2m_or1k_d_ack,
                wb_m2s_eth0_master_cyc & wb_s2m_eth0_master_ack,
                wb_m2s_or1k_i_cyc & wb_s2m_or1k_i_ack,
                1'b1,
                app_en & app_rdy,
                app_rd_data_valid,
                app_wdf_wren & app_wdf_rdy,
                wb_m2s_ddr2_cyc & ~wb_m2s_ddr2_we & wb_s2m_ddr2_ack,
                wb_m2s_ddr2_cyc &  wb_m2s_ddr2_we & wb_s2m_ddr2_ack
                };

   dbg_cnt #
     (
      .n_counters (16)
      )
   dbg_cnt_0
     (
      .wb_clk_i             (wb_clk),
      .wb_rst_i             (wb_rst),
      .async_rst_i          (async_rst),
      .wb_dat_i             (wb_m2s_dbg_cnt_dat),
      .wb_adr_i             (wb_m2s_dbg_cnt_adr[6:2]),
      .wb_sel_i             (wb_m2s_dbg_cnt_sel),
      .wb_we_i              (wb_m2s_dbg_cnt_we),
      .wb_cyc_i             (wb_m2s_dbg_cnt_cyc),
      .wb_stb_i             (wb_m2s_dbg_cnt_stb),
      .wb_dat_o             (wb_s2m_dbg_cnt_dat),
      .wb_ack_o             (wb_s2m_dbg_cnt_ack),
      .wb_err_o             (wb_s2m_dbg_cnt_err),
      .wb_rty_o             (wb_s2m_dbg_cnt_rty),
      .cnt_strobe_i         (dbg_cnt_0_strobe),
      .irq_o                (dbg_cnt_0_irq)
      );


   ////////////////////////////////////////////////////////////////////////
   // Interrupt assignment
   ////////////////////////////////////////////////////////////////////////

   assign or1k_irq[0] = 0; // Non-maskable inside OR1K
   assign or1k_irq[1] = 0; // Non-maskable inside OR1K
   assign or1k_irq[2] = uart_irq;
   assign or1k_irq[3] = 0;
   assign or1k_irq[4] = eth0_irq;
   assign or1k_irq[5] = ps2_0_irq;
   assign or1k_irq[6] = 0; //spi_accel_irq;
   assign or1k_irq[7] = 0;
   assign or1k_irq[8] = vga0_irq;
   assign or1k_irq[9] = 0;
   assign or1k_irq[10] = 0;
   assign or1k_irq[11] = 0;
   assign or1k_irq[12] = 0; //ac97_irq;
   assign or1k_irq[13] = 0; //ps2_1_irq;
   assign or1k_irq[14] = 0; //ps2_2_irq;
   assign or1k_irq[15] = dbg_cnt_0_irq;
   assign or1k_irq[16] = 0;
   assign or1k_irq[17] = 0;
   assign or1k_irq[18] = 0;
   assign or1k_irq[19] = 0;
   assign or1k_irq[20] = 0;
   assign or1k_irq[21] = 0;
   assign or1k_irq[22] = 0;
   assign or1k_irq[23] = 0;
   assign or1k_irq[24] = 0;
   assign or1k_irq[25] = 0;
   assign or1k_irq[26] = 0;
   assign or1k_irq[27] = 0;
   assign or1k_irq[28] = 0;
   assign or1k_irq[29] = 0;
   assign or1k_irq[30] = 0;
   assign or1k_irq[31] = 0;

   wire enable = sw_i[15];
   reg [31:0]      count;
   always @(posedge wb_clk or posedge async_rst)
     if (async_rst)
       count <= 0;
     else if (enable)
       count <= count + 1;

   //assign led_net = 16'b0; //sw_i; //{enable, 3'b0, count[31:20]} | {wb_m2s_ddr2_dbus_adr[7:0], wb_m2s_ddr2_dbus_dat[7:0]} | wb_m2s_ddr2_ibus_adr[15:0];
   //assign led_net = sw_i | {16'b0};
   //assign led_net = sw_i | {wb_m2s_dbg_cyc, exception, 14'b0};
   //assign led_net = sw_i | {5'b0,dbg_tck,jtag_tap_drck,jtag_tap_runtest,jtag_tap_select,dbg_if_tdo,jtag_tap_tdo,jtag_tap_shift_dr,jtag_tap_pause_dr,jtag_tap_update_dr,jtag_tap_capture_dr,jtag_tap_reset};
   //assign led_net = sw_i | {3'b0,or1k_dbg_stall_i,or1k_dbg_bp_o, dbg_tck,jtag_tap_drck,jtag_tap_runtest,jtag_tap_select,dbg_if_tdo,jtag_tap_tdo,jtag_tap_shift_dr,jtag_tap_pause_dr,jtag_tap_update_dr,jtag_tap_capture_dr,jtag_tap_reset};

   assign led_net = sw_i | {wb_m2s_dbg_cyc, exception, 3'b0, eth0_mii_tx_en, eth0_mii_tx_er, eth0_mii_rx_dv, eth0_mii_rx_er, eth0_mii_coll, eth0_mii_crs, eth0_txd_o, eth0_tx_en_o, eth0_rxd_i, eth0_rx_er_i, eth0_crs_dv_i};

   assign ja_net = 8'b0;
   assign jb_net = 8'b0;
   assign jc_net = 8'b0;
   assign jd_net = 8'b0;

endmodule // orpsoc_top
