//
// ORPSoC testbench for Nexys4-DDR board
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

`include "timescale.v"
//`include "orpsoc-defines.v"

// Xilinx simulators do not support VPI
`ifndef ISIM_SIM
 `ifndef XSIM_SIM
  `define INCLUDE_ELF_LOADER
  `define INCLUDE_JTAG_VPI
 `endif
`endif

// if elf loader is not used or not available fall back to loading programs via WB master
`ifndef INCLUDE_ELF_LOADER
 //`define INCLUDE_WB_LOADER
`endif


module orpsoc_tb;

`ifdef INCLUDE_WB_LOADER
   // waits for sw_i[13] == 1'b1, then jumps to 0x100
   localparam BOOTROM_FILE = "../src/nexys4ddr_top/sw/boot_loop.vh";

   // program to download, must be processed by hex2dat.py
   //localparam WB_LOADER_MEM_FILE_NAME = "../src/nexys4ddr_top/sw/or1k-basic.dat";
   //localparam WB_LOADER_MEM_FILE_NAME = "../src/nexys4ddr_top/sw/mem_copy.dat";
   //localparam WB_LOADER_MEM_FILE_NAME = "../src/nexys4ddr_top/sw/mem_write.dat";
   //localparam WB_LOADER_MEM_FILE_NAME = "../src/nexys4ddr_top/sw/mem_access.dat";
   localparam WB_LOADER_MEM_FILE_NAME = "../src/nexys4ddr_top/sw/mem_size.dat";
   //localparam WB_LOADER_MEM_FILE_NAME = "../src/nexys4ddr_top/sw/hello.dat";
`else
   // jumps to 0x100
   localparam BOOTROM_FILE = "../src/nexys4ddr_top/sw/jump_to_0x100.vh";
   //localparam BOOTROM_FILE = "../src/nexys4ddr_top/sw/spi_uimage_loader.vh";
   //localparam BOOTROM_FILE = "../src/nexys4ddr_top/sw/boot_loop.vh";
`endif

   ////////////////////////////////////////////////////////////////////////
   // Generate clock (100MHz) and external reset
   ////////////////////////////////////////////////////////////////////////

   reg clk   = 0;
   reg rst_n = 0;

   always #5 clk <= ~clk;

   initial
     begin
        #100 rst_n <= 0;
        #200 rst_n <= 1;
     end

`ifdef INCLUDE_ELF_LOADER
   ////////////////////////////////////////////////////////////////////////
   // ELF program loading
   ////////////////////////////////////////////////////////////////////////

   integer mem_words;
   integer i;
   reg [31:0] mem_word;
   reg [1023:0] elf_file;

   initial begin
      if($value$plusargs("elf_load=%s", elf_file))
        begin
	   $elf_load_file(elf_file);

	   mem_words = $elf_get_size/4;
	   $display("ELF Loader. Loading %d words", mem_words);
	   for(i=0; i < mem_words; i = i+1)
             begin
                mem_word = $elf_read_32(i*4);
	        orpsoc_tb.dut.ddr2_wb0.ui_bfm_memory_0.mem[4*i+0] = mem_word[31:24];
	        orpsoc_tb.dut.ddr2_wb0.ui_bfm_memory_0.mem[4*i+1] = mem_word[23:16];
	        orpsoc_tb.dut.ddr2_wb0.ui_bfm_memory_0.mem[4*i+2] = mem_word[15: 8];
	        orpsoc_tb.dut.ddr2_wb0.ui_bfm_memory_0.mem[4*i+3] = mem_word[ 7: 0];
             end
        end
      else
	$display("No ELF file specified");
   end
`endif //  `ifdef INCLUDE_ELF_LOADER

   integer dump_on = 0;
   always @(posedge clk)
     begin
        //if (!dump_on && dut.mor1kx0.mor1kx_cpu.monitor_execute_pc == 32'h0100)
        if (!dump_on && dut.mor1kx0.mor1kx_cpu.monitor_execute_pc == 32'h01fb7b20)
          begin
             dump_on = 1;
             $dumpvars(0);
`define INCLUDE_MOR1KX_REGS
`ifdef INCLUDE_MOR1KX_REGS
             //$dumpvars(0, dut.mor1kx0);
             for (idx = 0; idx < 32; idx = idx + 1) begin
                $dumpvars(0, dut.mor1kx0.mor1kx_cpu.cappuccino.mor1kx_cpu.mor1kx_rf_cappuccino.rfa.mem[idx]);
                $dumpvars(0, dut.mor1kx0.mor1kx_cpu.cappuccino.mor1kx_cpu.mor1kx_rf_cappuccino.rfb.mem[idx]);
                $dumpvars(0, dut.mor1kx0.mor1kx_cpu.cappuccino.mor1kx_cpu.mor1kx_rf_cappuccino.rfspr_gen.rfspr.mem[idx]);
             end
`endif
          end // if (==32'h00000000)
        //if (dump_on && dut.mor1kx0.mor1kx_cpu.monitor_execute_pc == 32'h0110)
        if (dump_on &&
            (dut.mor1kx0.mor1kx_cpu.monitor_execute_pc == 32'h0200 ||
             dut.mor1kx0.mor1kx_cpu.monitor_execute_pc == 32'h1fb7bd0))
          $finish();
     end
   
   integer idx;
   initial begin
      //if($test$plusargs("lxt2")) begin
      //if($value$plusargs("testcase=%s", testcase))
      //  $dumpfile({testcase,".lxt"});
      //else
      //$dumpfile("testlog.vcd");
      $dumpfile("testlog.lxt");
      //$dumpfile("testlog.fst");
      //$dumpvars(4);
      //$dumpvars(0); // dump all signals
      //$dumpoff;
//`define INCLUDE_MOR1KX_REGS
//`ifdef INCLUDE_MOR1KX_REGS
//      //$dumpvars(0, dut.mor1kx0);
//      for (idx = 0; idx < 32; idx = idx + 1) begin
//         $dumpvars(0, dut.mor1kx0.mor1kx_cpu.cappuccino.mor1kx_cpu.mor1kx_rf_cappuccino.rfa.mem[idx]);
//         $dumpvars(0, dut.mor1kx0.mor1kx_cpu.cappuccino.mor1kx_cpu.mor1kx_rf_cappuccino.rfb.mem[idx]);
//         $dumpvars(0, dut.mor1kx0.mor1kx_cpu.cappuccino.mor1kx_cpu.mor1kx_rf_cappuccino.rfspr_gen.rfspr.mem[idx]);
//      end
//`endif
   end

   localparam                       uart_str_len = 80;
   reg         [uart_str_len*8-1:0] uart_str;
   
   localparam uart_baud_ns = 8680; // 115200bps
   //localparam uart_baud_ns = 8680 / 2;
   //localparam uart_baud_ns = 2000;
   initial begin
      @(posedge rst_n);

      for (idx = 0; idx < 4; idx = idx + 1)
        #uart_baud_ns;

      //uart_str = ":1001000018000000182000001840000018600000CF";
      ////uart_str = ":00000001FF";
      //for (idx = 43-1; idx >= 0; idx = idx - 1)
      //  uart_stim0.uart_tx_byte(uart_str[idx*8 +: 8]);
      uart_stim0.uart_tx_byte(8'h0d);
      for (idx = 0; idx < 10*30; idx = idx + 1)
      //for (idx = 0; idx < 10*3; idx = idx + 1)
        #uart_baud_ns;

      $dumpon;
      uart_str = "r 8";
      for (idx = 3-1; idx >= 0; idx = idx - 1)
        uart_stim0.uart_tx_byte(uart_str[idx*8 +: 8]);
      uart_stim0.uart_tx_byte(8'h0d);
      for (idx = 0; idx < 10*20; idx = idx + 1)
        #uart_baud_ns;

      uart_str = "w 0 deadbeef";
      for (idx = 12-1; idx >= 0; idx = idx - 1)
        uart_stim0.uart_tx_byte(uart_str[idx*8 +: 8]);
      uart_stim0.uart_tx_byte(8'h0d);
      for (idx = 0; idx < 10*20; idx = idx + 1)
        #uart_baud_ns;

      uart_str = "r 0";
      for (idx = 3-1; idx >= 0; idx = idx - 1)
        uart_stim0.uart_tx_byte(uart_str[idx*8 +: 8]);
      uart_stim0.uart_tx_byte(8'h0d);
      for (idx = 0; idx < 10*20; idx = idx + 1)
        #uart_baud_ns;
      //$finish;
      //uart_str = "\rr 0\rw 0 deadbeef\rr 0\r";
      //for (idx = uart_str_len-1; idx >= 0; idx = idx - 1)
      //  uart_stim0.uart_tx_byte(uart_str[idx*8 +: 8]);
    end

   ////////////////////////////////////////////////////////////////////////
   //
   // Add --vcd and --timeout options to the simulation
   //
   ////////////////////////////////////////////////////////////////////////
   vlog_tb_utils vlog_tb_utils0();


   ////////////////////////////////////////////////////////////////////////
   // JTAG VPI interface
   ////////////////////////////////////////////////////////////////////////

   glbl glbl();

`ifdef INCLUDE_JTAG_VPI
   reg enable_jtag_vpi;
   initial enable_jtag_vpi = 1'b1; //$test$plusargs("enable_jtag_vpi");

   initial glbl.JTAG_SEL1_GLBL = 1'b1;

   wire tdo, tdi, tms, tck;
   jtag_vpi jtag_vpi0
     (
      .tms              (tms),
      .tck              (tck),
      .tdi              (tdi),
      .tdo              (tdo),
      .enable           (enable_jtag_vpi),
      .init_done        (orpsoc_tb.dut.wb_rst)
      );

   JTAG_SIME2 #
     (
      .PART_NAME ("7A100T")
      )
   jtag_sime2_0
     (
      .TDO (tdo),
      .TCK (tck),
      .TDI (tdi),
      .TMS (tms)
      );
`endif //  `ifdef INCLUDE_JTAG_VPI

   ////////////////////////////////////////////////////////////////////////
   // DUT
   ////////////////////////////////////////////////////////////////////////

   //reg         clk_i;
   //reg         rst_n_i;

   // Push-buttons
   reg [4:0]   pbtn_i;

   // Switches
   reg [15:0]  sw_i;

   // ADT7420 Temperature Sensor TWI Signals
   wire        i2c_temp_scl_io;
   wire        i2c_temp_sda_io;
   reg         i2c_temp_int_i;
   reg         i2c_temp_ct_i;

   // ADXL362 Accelerometer SPI Signals
   wire        spi_accel_sclk_o;
   wire        spi_accel_mosi_o;
   reg         spi_accel_miso_i;
   wire        spi_accel_cs_n_o;
   reg         spi_accel_int1_i;
   reg         spi_accel_int2_i;

   // USB-RS232 Interface
   reg         uart_cts_i;
   wire        uart_rts_o;
   wire        uart_rxd_i;
   wire        uart_txd_o;

   // 7-segment display
   wire [7:0]  sseg_disp_seg_o;
   wire [7:0]  sseg_disp_an_o; // active low

   // LD16 RGB led
   wire        led_rgb1_red_o;
   wire        led_rgb1_green_o;
   wire        led_rgb1_blue_o;
   // LD17 RGB led
   wire        led_rgb2_red_o;
   wire        led_rgb2_green_o;
   wire        led_rgb2_blue_o;

   //LEDs
   wire [15:0] led_o;

   //VGA Signals
   wire        vga_hs_o;
   wire        vga_vs_o;
   wire [3:0]  vga_red_o;
   wire [3:0]  vga_green_o;
   wire [3:0]  vga_blue_o;

   // ADMP421 Omnidirectional Microphone Signals
   wire        mic_pdm_clk_o;
   reg         mic_pdm_data_i;
   wire        mic_pdm_lrsel_o;

   // Audio Out Signals
   wire        pwm_audio_o; // open-drain
   wire        pwm_sdaudio_o;

   // PS2 Signals
   wire         ps2_clk; // open-drain
   wire         ps2_data; // open-drain

   // Pmod Headers
   wire [7:0]  ja_io;
   wire [7:0]  jb_io;
   wire [7:0]  jc_io;
   wire [7:0]  jd_io;

   // Quad SPI Flash
   wire        spi_flash_cs_n_o;
   wire [3:0]  spi_flash_dq_io;
   //output        spi_flash_sck_o, // dedicated pin, access via STARTUPE2

   // SMSC Ethernet PHY
   reg [1:0]   eth_rxd_i;
   wire [1:0]  eth_txd_o;
   reg         eth_crsdv_i;
   reg         eth_intn_i;
   wire        eth_mdc_o;
   wire        eth_mdio_io;
   wire        eth_refclk_io;
   wire        eth_rst_n_o;
   wire        eth_txen_o;
   reg         eth_rxerr_i;

   // DDR2 interface signals
   wire [12:0] ddr2_addr;
   wire [2:0]  ddr2_ba;
   wire        ddr2_ras_n;
   wire        ddr2_cas_n;
   wire        ddr2_we_n;
   wire [0:0]  ddr2_ck_p;
   wire [0:0]  ddr2_ck_n;
   wire [0:0]  ddr2_cke;
   wire [0:0]  ddr2_cs_n;
   wire [1:0]  ddr2_dm;
   wire [0:0]  ddr2_odt;
   wire [15:0] ddr2_dq;
   wire [1:0]  ddr2_dqs_p;
   wire [1:0]  ddr2_dqs_n;

   // JTAG
   // wire              tdo_pad_o;
   // wire               tms_pad_i;
   // wire               tck_pad_i;
   // wire               tdi_pad_i;

   orpsoc_top #
     (
      .BOOTROM_FILE (BOOTROM_FILE)
      )
   dut
     (
      .clk_i                (clk),
      .rst_n_i              (rst_n),

      // Push-buttons
      .pbtn_i               (pbtn_i),

      // Switches
      .sw_i                 (sw_i),

      // ADT7420 Temperature Sensor TWI Signals
      .i2c_temp_scl_io      (i2c_temp_scl_io),
      .i2c_temp_sda_io      (i2c_temp_sda_io),
      .i2c_temp_int_i       (i2c_temp_int_i),
      .i2c_temp_ct_i        (i2c_temp_ct_i),

      // ADXL362 Accelerometer SPI Signals
      .spi_accel_sclk_o     (spi_accel_sclk_o),
      .spi_accel_mosi_o     (spi_accel_mosi_o),
      .spi_accel_miso_i     (spi_accel_miso_i),
      .spi_accel_cs_n_o     (spi_accel_cs_n_o),
      .spi_accel_int1_i     (spi_accel_int1_i),
      .spi_accel_int2_i     (spi_accel_int2_i),

      // USB-RS232 Interface
      .uart_cts_i           (uart_cts_i),
      .uart_rts_o           (uart_rts_o),
      .uart_rxd_i           (uart_rxd_i),
      .uart_txd_o           (uart_txd_o),

      // 7-segment display
      .sseg_disp_seg_o      (sseg_disp_seg_o),
      .sseg_disp_an_o       (sseg_disp_an_o),

      // LD16 RGB led
      .led_rgb1_red_o       (led_rgb1_red_o),
      .led_rgb1_green_o     (led_rgb1_green_o),
      .led_rgb1_blue_o      (led_rgb1_blue_o),
      // LD17 RGB led
      .led_rgb2_red_o       (led_rgb2_red_o),
      .led_rgb2_green_o     (led_rgb2_green_o),
      .led_rgb2_blue_o      (led_rgb2_blue_o),

      //LEDs
      .led_o                (led_o),

      //VGA Signals
      .vga_hs_o             (vga_hs_o),
      .vga_vs_o             (vga_vs_o),
      .vga_red_o            (vga_red_o),
      .vga_green_o          (vga_green_o),
      .vga_blue_o           (vga_blue_o),

      // ADMP421 Omnidirectional Microphone Signals
      .mic_pdm_clk_o        (mic_pdm_clk_o),
      .mic_pdm_data_i       (mic_pdm_data_i),
      .mic_pdm_lrsel_o      (mic_pdm_lrsel_o),

      // Audio Out Signals
      .pwm_audio_o          (pwm_audio_o),
      .pwm_sdaudio_o        (pwm_sdaudio_o),

      // PS2 Signals
      .ps2_clk              (ps2_clk),
      .ps2_data             (ps2_data),

      // Pmod Headers
      .ja_io                (ja_io),
      .jb_io                (jb_io),
      .jc_io                (jc_io),
      .jd_io                (jd_io),

      // Quad SPI Flash
      .spi_flash_cs_n_o     (spi_flash_cs_n_o),
      .spi_flash_dq_io      (spi_flash_dq_io),
      //output        spi_flash_sck_o, // dedicated pin, access via STARTUPE2

      // SMSC Ethernet PHY
      .eth_rxd_i            (eth_rxd_i),
      .eth_txd_o            (eth_txd_o),
      .eth_crsdv_i          (eth_crsdv_i),
      .eth_intn_i           (eth_intn_i),
      .eth_mdc_o            (eth_mdc_o),
      .eth_mdio_io          (eth_mdio_io),
      .eth_refclk_io        (eth_refclk_io),
      .eth_rst_n_o          (eth_rst_n_o),
      .eth_txen_o           (eth_txen_o),
      .eth_rxerr_i          (eth_rxerr_i),

      // DDR2 interface signals
      .ddr2_addr            (ddr2_addr),
      .ddr2_ba              (ddr2_ba),
      .ddr2_ras_n           (ddr2_ras_n),
      .ddr2_cas_n           (ddr2_cas_n),
      .ddr2_we_n            (ddr2_we_n),
      .ddr2_ck_p            (ddr2_ck_p),
      .ddr2_ck_n            (ddr2_ck_n),
      .ddr2_cke             (ddr2_cke),
      .ddr2_cs_n            (ddr2_cs_n),
      .ddr2_dm              (ddr2_dm),
      .ddr2_odt             (ddr2_odt),
      .ddr2_dq              (ddr2_dq),
      .ddr2_dqs_p           (ddr2_dqs_p),
      .ddr2_dqs_n           (ddr2_dqs_n)

      // JTAG
      //.tdo_pad_o            (tdo_pad_o),
      //.tms_pad_i            (tms_pad_i),
      //.tck_pad_i            (tck_pad_i),
      //.tdi_pad_i            (tdi_pad_i),
      );

`ifdef MOR1KX
   mor1kx_monitor #(.LOG_DIR(".")) i_monitor();
`else
   //or1200_monitor i_monitor();
`endif


   ////////////////////////////////////////////////////////////////////////
   // DDR2 DRAM model
   ////////////////////////////////////////////////////////////////////////

   ddr2 u_ddr2
     (
      .ck      (ddr2_ck_p),
      .ck_n    (ddr2_ck_n),
      .cke     (ddr2_cke),
      .cs_n    (ddr2_cs_n),
      .ras_n   (ddr2_ras_n),
      .cas_n   (ddr2_cas_n),
      .we_n    (ddr2_we_n),
      .dm_rdqs (ddr2_dm),
      .ba      (ddr2_ba),
      .addr    (ddr2_addr),
      .dq      (ddr2_dq),
      .dqs     (ddr2_dqs_p),
      .dqs_n   (ddr2_dqs_n),
      .rdqs_n  (),
      .odt     (ddr2_odt)
      );


   ////////////////////////////////////////////////////////////////////////
   // SPI Accelerometer
   ////////////////////////////////////////////////////////////////////////

   pullup spi_accel_pull_cs_n (spi_accel_cs_n_o);

   ////////////////////////////////////////////////////////////////////////
   // SPI Flash model
   ////////////////////////////////////////////////////////////////////////

   pullup spi_flash_pull_dq   [3:0] (spi_flash_dq_io);
   pullup spi_flash_pull_cs_n       (spi_flash_cs_n_o);


   s25fl128s #
     (
      .mem_file_name ("../src/nexys4ddr_top/bench/s25fl128s.mem"),
      .otp_file_name ("../src/nexys4ddr_top/bench/s25fl128sOTP.mem")
      )
   spi_flash0
     (
      .SI      (spi_flash_dq_io[0]),
      .SO      (spi_flash_dq_io[1]),
      .SCK     (glbl.CCLKO_GLBL),
      .CSNeg   (spi_flash_cs_n_o),
      .WPNeg   (spi_flash_dq_io[2]),
      .HOLDNeg (spi_flash_dq_io[3])
      );

   ////////////////////////////////////////////////////////////////////////
   // UART receiver
   ////////////////////////////////////////////////////////////////////////

   //FIXME: Get correct baud rate from parameter
   uart_decoder
     #(.uart_baudrate_period_ns(uart_baud_ns))
   uart_decoder0
     (
      .clk(clk),
      .uart_tx(uart_txd_o)
      );

   ////////////////////////////////////////////////////////////////////////
   // UART transmitter
   ////////////////////////////////////////////////////////////////////////

   //FIXME: Get correct baud rate from parameter
   uart_stim
     #(.uart_baudrate_period_ns(uart_baud_ns))
   uart_stim0
     (
      .clk(clk),
      .uart_rx(uart_rxd_i)
      );
   initial uart_cts_i = 1'b0;

   assign ja_io = 8'bz;
   assign jb_io = 8'bz;
   assign jc_io = 8'bz;
   assign jd_io = 8'bz;

   //initial sw_i <= 16'h1fff;
   initial sw_i <= 16'h5fff;
   initial pbtn_i <= 5'b00011;

   //wire [31:0] wb_m2s_dbg_adr;
   //wire [31:0] wb_m2s_dbg_dat;
   //wire        wb_m2s_dbg_sel;
   //wire        wb_m2s_dbg_we;
   //wire        wb_m2s_dbg_cyc;
   //wire        wb_m2s_dbg_stb;
   //wire [ 2:0] wb_m2s_dbg_cti;
   //wire [ 1:0] wb_m2s_dbg_bte;
   //always @(wb_m2s_dbg_adr) force dut.wb_m2s_dbg_adr = wb_m2s_dbg_adr;
   //always @(wb_m2s_dbg_dat) force dut.wb_m2s_dbg_dat = wb_m2s_dbg_dat;
   //always @(wb_m2s_dbg_sel) force dut.wb_m2s_dbg_sel = wb_m2s_dbg_sel;
   //always @(wb_m2s_dbg_we)  force dut.wb_m2s_dbg_we  = wb_m2s_dbg_we;
   //always @(wb_m2s_dbg_cyc) force dut.wb_m2s_dbg_cyc = wb_m2s_dbg_cyc;
   //always @(wb_m2s_dbg_stb) force dut.wb_m2s_dbg_stb = wb_m2s_dbg_stb;
   //always @(wb_m2s_dbg_cti) force dut.wb_m2s_dbg_cti = wb_m2s_dbg_cti;
   //always @(wb_m2s_dbg_bte) force dut.wb_m2s_dbg_bte = wb_m2s_dbg_bte;

   ////////////////////////////////////////////////////////////////////////
   // Write program to RAM via WB master
   ////////////////////////////////////////////////////////////////////////

`ifdef INCLUDE_WB_LOADER
   localparam WB_LOADER_MEM_SIZE = 2**14;
   reg [63:0] wb_loader_data_mem [0:WB_LOADER_MEM_SIZE-1];
   reg [31:0] wb_loader_addr;
   reg [31:0] wb_loader_data;
   reg [ 3:0] wb_loader_sel;
   reg        wb_loader_err;
   integer    i;
   initial
     begin
        $display("Preloading boot RAM from %s", WB_LOADER_MEM_FILE_NAME);
        wb_bfm_master_0.reset();
        $readmemh(WB_LOADER_MEM_FILE_NAME, wb_loader_data_mem);
        #100000;
        wb_loader_sel = 4'b1111;
        wb_bfm_master_0.write(32'h0, 32'h0, wb_loader_sel, wb_loader_err);
        wb_bfm_master_0.write(32'h4, 32'h0, wb_loader_sel, wb_loader_err);
        wb_bfm_master_0.write(32'h8, 32'h0, wb_loader_sel, wb_loader_err);
        wb_bfm_master_0.write(32'hc, 32'h0, wb_loader_sel, wb_loader_err);
        for (i = 0; i < WB_LOADER_MEM_SIZE && wb_loader_data_mem[i] !== {64{1'bx}}; i = i + 1)
          begin
             wb_loader_addr = wb_loader_data_mem[i][63:32];
             wb_loader_data = wb_loader_data_mem[i][31:0];
             $display("Write, address=%08x data=%08x, sel=%04b", wb_loader_addr, wb_loader_data, wb_loader_sel);
             wb_bfm_master_0.write(wb_loader_addr, wb_loader_data, wb_loader_sel, wb_loader_err);
             @(posedge dut.wb_clk);
             if (wb_loader_err)
               $display ("Bus error");
          end
        //force sw_i = 16'h3fff;
        force sw_i = 16'h7fff;
     end

   ////////////////////////////////////////////////////////////////////////
   // WB master
   ////////////////////////////////////////////////////////////////////////

   wb_bfm_master
     #(.MAX_BURST_LENGTH (4))
   wb_bfm_master_0
     (
      .wb_clk_i (dut.wb_clk),
      .wb_rst_i (dut.wb_rst),
      .wb_adr_o (dut.wb_m2s_dbg_rs232_adr),
      .wb_dat_o (dut.wb_m2s_dbg_rs232_dat),
      .wb_sel_o (dut.wb_m2s_dbg_rs232_sel),
      .wb_we_o  (dut.wb_m2s_dbg_rs232_we),
      .wb_cyc_o (dut.wb_m2s_dbg_rs232_cyc),
      .wb_stb_o (dut.wb_m2s_dbg_rs232_stb),
      .wb_cti_o (dut.wb_m2s_dbg_rs232_cti),
      .wb_bte_o (dut.wb_m2s_dbg_rs232_bte),
      .wb_dat_i (dut.wb_s2m_dbg_rs232_dat),
      .wb_ack_i (dut.wb_s2m_dbg_rs232_ack),
      .wb_err_i (1'b0), //dut.wb_s2m_dbg_err),
      .wb_rty_i (1'b0) //dut.wb_s2m_dbg_rty)
      );
`endif //  `ifdef INCLUDE_WB_LOADER

endmodule
