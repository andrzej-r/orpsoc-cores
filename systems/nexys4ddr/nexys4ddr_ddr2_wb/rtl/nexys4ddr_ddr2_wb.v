//
// Wishbone wrapper for Xilinx MIG DDR2 controller
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

`ifdef ICARUS_SIM
 `define BFM_MODEL
`endif

module nexys4ddr_ddr2_wb #
  (
   parameter awc =   4,
   parameter dwc =  16
   )
   (
    output                 wb_clk_o,
    output                 wb_rst_o,
    input                  async_rst_i,

    // Wishbone Interface
    input  [31:0]          wb_adr_i,
    input  [31:0]          wb_dat_i,
    input  [3:0]           wb_sel_i,
    input                  wb_we_i,
    input                  wb_cyc_i,
    input                  wb_stb_i,
    input  [2:0]           wb_cti_i,
    input  [1:0]           wb_bte_i,
    output [31:0]          wb_dat_o,
    output                 wb_ack_o,
    output                 wb_err_o,
    output                 wb_rty_o,

    // Control Wishbone Interface
    input  [awc-1:0]       wbc_adr_i,
    input  [dwc-1:0]       wbc_dat_i,
    input                  wbc_we_i,
    input                  wbc_cyc_i,
    input                  wbc_stb_i,
    output reg [dwc-1:0]   wbc_dat_o,
    output reg             wbc_ack_o,
    output                 wbc_err_o,
    output                 wbc_rty_o,

    // DDR2 I/F
    // core side
    input                  sys_clk_i,
    input                  clk_ref_i,
    output                 init_calib_complete_o,

    // Inouts
    inout [15:0]           ddr2_dq,
    inout [1:0]            ddr2_dqs_n,
    inout [1:0]            ddr2_dqs_p,

    // Outputs
    output [12:0]          ddr2_addr,
    output [2:0]           ddr2_ba,
    output                 ddr2_ras_n,
    output                 ddr2_cas_n,
    output                 ddr2_we_n,
    output                 ddr2_ck_p,
    output                 ddr2_ck_n,
    output                 ddr2_cke,
    output                 ddr2_cs_n,
    output [1:0]           ddr2_dm,
    output                 ddr2_odt
    );

   wire                  ui_clk;
   wire                  ui_clk_sync_rst;

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

   reg  [11:0]           device_temp_r;

   wire app_sr_req_pulse;
   wire app_ref_req_pulse;
   wire app_zq_req_pulse;

   assign                wb_clk_o = ui_clk;
   assign                wb_rst_o = ui_clk_sync_rst;

   // Wishbone to Xilinx UI bus adapter
   nexys4ddr_ddr2_wb_to_ui wb_to_ui_0
     (
      .async_rst_i         (async_rst_i),
      .clk_i               (wb_clk_o),
      .rst_i               (wb_rst_o),

      // Wishbone interface
      .wb_adr_i            (wb_adr_i),
      .wb_dat_i            (wb_dat_i),
      .wb_sel_i            (wb_sel_i),
      .wb_we_i             (wb_we_i),
      .wb_cyc_i            (wb_cyc_i),
      .wb_stb_i            (wb_stb_i),
      .wb_cti_i            (wb_cti_i),
      .wb_bte_i            (wb_bte_i),
      .wb_dat_o            (wb_dat_o),
      .wb_ack_o            (wb_ack_o),
      .wb_err_o            (wb_err_o),
      .wb_rty_o            (wb_rty_o),

      // DDR2 UI interface
      .app_addr_o          (app_addr),
      .app_cmd_o           (app_cmd),
      .app_en_o            (app_en),
      .app_wdf_data_o      (app_wdf_data),
      .app_wdf_end_o       (app_wdf_end),
      .app_wdf_mask_o      (app_wdf_mask),
      .app_wdf_wren_o      (app_wdf_wren),
      .app_rd_data_i       (app_rd_data),
      .app_rd_data_end_i   (app_rd_data_end),
      .app_rd_data_valid_i (app_rd_data_valid),
      .app_rdy_i           (app_rdy),
      .app_wdf_rdy_i       (app_wdf_rdy)
      );

`ifndef BFM_MODEL
   // Xilinx MIG DDR2 interface
   nexys4ddr_ddr2 #
     (
      .BANK_WIDTH               (3),
      .CK_WIDTH                 (1),
      .COL_WIDTH                (10),
      .CS_WIDTH                 (1),
      .nCS_PER_RANK             (1),
      .CKE_WIDTH                (1),
      .DATA_BUF_ADDR_WIDTH      (5),
      .DQ_CNT_WIDTH             (4),
      .DQ_PER_DM                (8),
      .DM_WIDTH                 (2),
      .DQ_WIDTH                 (16),
      .DQS_WIDTH                (2),
      .DQS_CNT_WIDTH            (1),
      .DRAM_WIDTH               (8),
      .ECC                      ("OFF"),
      .DATA_WIDTH               (16),
      .ECC_TEST                 ("OFF"),
      .PAYLOAD_WIDTH            (16),
      //.PAYLOAD_WIDTH            ((ECC_TEST == "OFF") ? DATA_WIDTH : DQ_WIDTH),
      .ECC_WIDTH                (8),
      .MC_ERR_ADDR_WIDTH        (31),
      .MEM_ADDR_ORDER           ("BANK_ROW_COLUMN"),
      .nBANK_MACHS              (4),
      .RANKS                    (1),
      .ODT_WIDTH                (1),
      .ROW_WIDTH                (13),
      .ADDR_WIDTH               (27),
      .USE_CS_PORT              (1),
      .USE_DM_PORT              (1),
      .USE_ODT_PORT             (1),
      .PHY_CONTROL_MASTER_BANK  (0),
      .MEM_DENSITY              ("1Gb"),
      .MEM_SPEEDGRADE           ("25E"),
      .MEM_DEVICE_WIDTH         (16),

      .AL                       ("0"),
      .nAL                      (0),
      .BURST_MODE               ("8"),
      .BURST_TYPE               ("SEQ"),
      .CL                       (4),
      .OUTPUT_DRV               ("HIGH"),
      .RTT_NOM                  ("50"),
      .ADDR_CMD_MODE            ("1T" ),
      .REG_CTRL                 ("OFF"),

      .CLKIN_PERIOD             (5000),
      .CLKFBOUT_MULT            (4),
      .DIVCLK_DIVIDE            (1),
      //.CLKIN_PERIOD             (3750),
      //.CLKFBOUT_MULT            (4),
      //.DIVCLK_DIVIDE            (1),
      .CLKOUT0_PHASE            (0.0),
      .CLKOUT0_DIVIDE           (2),
      .CLKOUT1_DIVIDE           (4),
      .CLKOUT2_DIVIDE           (64),
      .CLKOUT3_DIVIDE           (16),
      .tCKE                     (7500),
      .tFAW                     (45000),
`ifdef ICARUS_SIM
      .tPRDI                    (0), // disable periodic reads, cause problems with a dummy phy
`else
      .tPRDI                    (1_000_000),
`endif
      .tRAS                     (40000),
      .tRCD                     (15000),
      .tREFI                    (7800000),
      .tRFC                     (127500),
      .tRP                      (12500),
      .tRRD                     (10000),
      .tRTP                     (7500),
      .tWTR                     (7500),
      .tZQI                     (128_000_000),
      .tZQCS                    (64),
      //.SIM_BYPASS_INIT_CAL      ("OFF"),
      .SIM_BYPASS_INIT_CAL      ("FAST"),
`ifdef SIM
      .SIMULATION               ("TRUE"),
`else
      .SIMULATION               ("FALSE"),
`endif
      .BYTE_LANES_B0            (4'b1111),
      .BYTE_LANES_B1            (4'b0000),
      .BYTE_LANES_B2            (4'b0000),
      .BYTE_LANES_B3            (4'b0000),
      .BYTE_LANES_B4            (4'b0000),
      .DATA_CTL_B0              (4'b0101),
      .DATA_CTL_B1              (4'b0000),
      .DATA_CTL_B2              (4'b0000),
      .DATA_CTL_B3              (4'b0000),
      .DATA_CTL_B4              (4'b0000),
      .PHY_0_BITLANES           (48'hFFC_3F7_FFF_3FE),
      .PHY_1_BITLANES           (48'h000_000_000_000),
      .PHY_2_BITLANES           (48'h000_000_000_000),
      .CK_BYTE_MAP              (144'h00_00_00_00_00_00_00_00_00_00_00_00_00_00_00_00_00_03),
      .ADDR_MAP                 (192'h000_000_000_010_033_01A_019_032_03A_034_018_036_012_011_017_015),
      .BANK_MAP                 (36'h013_016_01B),
      .CAS_MAP                  (12'h039),
      .CKE_ODT_BYTE_MAP         (8'h00),
      .CKE_MAP                  (96'h000_000_000_000_000_000_000_038),
      .ODT_MAP                  (96'h000_000_000_000_000_000_000_035),
      .CS_MAP                   (120'h000_000_000_000_000_000_000_000_000_037),
      .PARITY_MAP               (12'h000),
      .RAS_MAP                  (12'h014),
      .WE_MAP                   (12'h03B),
      .DQS_BYTE_MAP             (144'h00_00_00_00_00_00_00_00_00_00_00_00_00_00_00_00_02_00),
      .DATA0_MAP                (96'h008_004_009_007_005_001_006_003),
      .DATA1_MAP                (96'h022_028_020_024_027_025_026_021),
      .DATA2_MAP                (96'h000_000_000_000_000_000_000_000),
      .DATA3_MAP                (96'h000_000_000_000_000_000_000_000),
      .DATA4_MAP                (96'h000_000_000_000_000_000_000_000),
      .DATA5_MAP                (96'h000_000_000_000_000_000_000_000),
      .DATA6_MAP                (96'h000_000_000_000_000_000_000_000),
      .DATA7_MAP                (96'h000_000_000_000_000_000_000_000),
      .DATA8_MAP                (96'h000_000_000_000_000_000_000_000),
      .DATA9_MAP                (96'h000_000_000_000_000_000_000_000),
      .DATA10_MAP               (96'h000_000_000_000_000_000_000_000),
      .DATA11_MAP               (96'h000_000_000_000_000_000_000_000),
      .DATA12_MAP               (96'h000_000_000_000_000_000_000_000),
      .DATA13_MAP               (96'h000_000_000_000_000_000_000_000),
      .DATA14_MAP               (96'h000_000_000_000_000_000_000_000),
      .DATA15_MAP               (96'h000_000_000_000_000_000_000_000),
      .DATA16_MAP               (96'h000_000_000_000_000_000_000_000),
      .DATA17_MAP               (96'h000_000_000_000_000_000_000_000),
      .MASK0_MAP                (108'h000_000_000_000_000_000_000_029_002),
      .MASK1_MAP                (108'h000_000_000_000_000_000_000_000_000),
      .SLOT_0_CONFIG            (8'b0000_0001),
      .SLOT_1_CONFIG            (8'b0000_0000),
      .IODELAY_HP_MODE          ("ON"),
      .IBUF_LPWR_MODE           ("OFF"),
      .DATA_IO_IDLE_PWRDWN      ("ON"),
      .BANK_TYPE                ("HR_IO"),
      .DATA_IO_PRIM_TYPE        ("HR_LP"),
      .CKE_ODT_AUX              ("FALSE"),
      .USER_REFRESH             ("OFF"),
      .WRLVL                    ("OFF"),
      .ORDERING                 ("STRICT"),
      .CALIB_ROW_ADD            (16'h0000),
      .CALIB_COL_ADD            (12'h000),
      .CALIB_BA_ADD             (3'h0),
      .TCQ                      (100),
      .IODELAY_GRP              ("IODELAY_MIG"),
      .SYSCLK_TYPE              ("NO_BUFFER"),
      //.REFCLK_TYPE              ("NO_BUFFER"),
      .REFCLK_TYPE              ("USE_SYSTEM_CLOCK"),
      .SYS_RST_PORT             ("FALSE"),
      .CMD_PIPE_PLUS1           ("ON"),
      .DRAM_TYPE                ("DDR2"),
      .CAL_WIDTH                ("HALF"),
      .STARVE_LIMIT             (2),
      .REFCLK_FREQ              (200.0),
      .DIFF_TERM_REFCLK         ("TRUE"),
      .tCK                      (3750),
      .nCK_PER_CLK              (4),
      .DIFF_TERM_SYSCLK         ("TRUE"),
      .DEBUG_PORT               ("OFF"),
      .TEMP_MON_CONTROL         ("EXTERNAL"),
      .RST_ACT_LOW              (1)
      )
   nexys4ddr_ddr2_0
     (
      // Inouts
      .ddr2_dq                  (ddr2_dq),
      .ddr2_dqs_n               (ddr2_dqs_n),
      .ddr2_dqs_p               (ddr2_dqs_p),

      // Outputs
      .ddr2_addr                (ddr2_addr),
      .ddr2_ba                  (ddr2_ba),
      .ddr2_ras_n               (ddr2_ras_n),
      .ddr2_cas_n               (ddr2_cas_n),
      .ddr2_we_n                (ddr2_we_n),
      .ddr2_ck_p                (ddr2_ck_p),
      .ddr2_ck_n                (ddr2_ck_n),
      .ddr2_cke                 (ddr2_cke),
      .ddr2_cs_n                (ddr2_cs_n),
      .ddr2_dm                  (ddr2_dm),
      .ddr2_odt                 (ddr2_odt),

      // Inputs
      // Single-ended system clock
      .sys_clk_i                (sys_clk_i),
      .sys_rst                  (~async_rst_i),
      // Single-ended iodelayctrl clk (reference clock)
      //.clk_ref_i                (clk_ref_i),
      // user interface signals
      .app_addr                 (app_addr),
      .app_cmd                  (app_cmd),
      .app_en                   (app_en),
      .app_wdf_data             (app_wdf_data),
      .app_wdf_end              (app_wdf_end),
      .app_wdf_mask             (app_wdf_mask),
      .app_wdf_wren             (app_wdf_wren),
      .app_rd_data              (app_rd_data),
      .app_rd_data_end          (app_rd_data_end),
      .app_rd_data_valid        (app_rd_data_valid),
      .app_rdy                  (app_rdy),
      .app_wdf_rdy              (app_wdf_rdy),
      .app_sr_req               (app_sr_req_pulse),
      .app_sr_active            (),
      .app_ref_req              (app_ref_req_pulse),
      .app_ref_ack              (),
      .app_zq_req               (app_zq_req_pulse),
      .app_zq_ack               (),
      .ui_clk                   (ui_clk),
      .ui_clk_sync_rst          (ui_clk_sync_rst),
      .init_calib_complete      (init_calib_complete_o),
      .device_temp_i            (device_temp_r)
      );
`else // !`ifndef BFM_MODEL
   ui_bfm_memory #
     (
      .dw          (128),
      .aw          (27),
      .memory_file (""),
      .mem_bytes   (32'h8000000),
      .verbose     (1)
      )
   ui_bfm_memory_0
     (
      .sys_clk_i           (sys_clk_i),
      .clk_ref_i           (sys_clk_i),
      //.clk_ref_i           (clk_ref_i),
      .app_addr            (app_addr),
      .app_cmd             (app_cmd),
      .app_en              (app_en),
      .app_wdf_data        (app_wdf_data),
      .app_wdf_end         (app_wdf_end),
      .app_wdf_mask        (app_wdf_mask),
      .app_wdf_wren        (app_wdf_wren),
      .app_rd_data         (app_rd_data),
      .app_rd_data_end     (app_rd_data_end),
      .app_rd_data_valid   (app_rd_data_valid),
      .app_rdy             (app_rdy),
      .app_wdf_rdy         (app_wdf_rdy),
      .app_sr_req          (app_sr_req_pulse),
      .app_sr_active       (),
      .app_ref_req         (app_ref_req_pulse),
      .app_ref_ack         (),
      .app_zq_req          (app_zq_req_pulse),
      .app_zq_ack          (),
      .ui_clk              (ui_clk),
      .ui_clk_sync_rst     (ui_clk_sync_rst),
      .init_calib_complete (init_calib_complete_o),
      .sys_rst             (~async_rst_i)
      );
`endif // !`ifndef BFM_MODEL

   // control interface

   // address decoder
   reg [2**awc-1:0] sel;
   integer          i;
   always @(*)
     begin
        sel = {2**awc{1'b0}};
        for (i = 0; i < 2**awc; i = i + 1)
          if (wbc_adr_i == i)
            sel[i] = 1'b1;
     end

   // device_temp_r register
   always @(posedge wb_clk_o or posedge async_rst_i)
     if (async_rst_i)   device_temp_r <= 16'h977; // approx 25 Cdeg
     else if (wb_rst_o) device_temp_r <= 16'h977;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[1])
       device_temp_r <= wb_dat_i[11:0];

   // app_ref_req_r register
   reg       app_ref_req_r;
   reg [1:0] app_ref_req_r2;
   always @(posedge wb_clk_o or posedge async_rst_i)
     if (async_rst_i)            app_ref_req_r <= 1'b0;
     else if (wb_rst_o)          app_ref_req_r <= 1'b0;
     else if (app_ref_req_r2[1]) app_ref_req_r <= 1'b0;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[2])
       app_ref_req_r <= wb_dat_i[0];

   // app_zq_req_r register
   reg       app_zq_req_r;
   reg [1:0] app_zq_req_r2;
   always @(posedge wb_clk_o or posedge async_rst_i)
     if (async_rst_i)           app_zq_req_r <= 1'b0;
     else if (wb_rst_o)         app_zq_req_r <= 1'b0;
     else if (app_zq_req_r2[1]) app_zq_req_r <= 1'b0;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[2])
       app_zq_req_r <= wb_dat_i[1];

   // app_sr_req_r register
   reg       app_sr_req_r;
   reg [1:0] app_sr_req_r2;
   always @(posedge wb_clk_o or posedge async_rst_i)
     if (async_rst_i)           app_sr_req_r <= 1'b0;
     else if (wb_rst_o)         app_sr_req_r <= 1'b0;
     else if (app_sr_req_r2[1]) app_sr_req_r <= 1'b0;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[2])
       app_sr_req_r <= wb_dat_i[2];


   // read back register values
   always @(posedge wb_clk_o)
     if (wb_rst_o)
       wbc_dat_o <= {dwc{1'b0}};
     else if (wbc_cyc_i)
       begin
          wbc_dat_o <= 0;
          if (sel[0]) wbc_dat_o[2:0]  <= {app_wdf_rdy, app_rdy, init_calib_complete_o};
          if (sel[1]) wbc_dat_o[11:0] <= device_temp_r;
       end

   // Ack generation
   always @(posedge wb_clk_o)
     if (wb_rst_o)
       wbc_ack_o <= 0;
     else if (wbc_ack_o)
       wbc_ack_o <= 0;
     else if (wbc_cyc_i & wbc_stb_i & ~wbc_ack_o)
       wbc_ack_o <= 1;

   assign wbc_err_o = 0;
   assign wbc_rty_o = 0;

   // app_ref_req pulse generator
   always @(posedge ui_clk or posedge async_rst_i)
     if (async_rst_i)          app_ref_req_r2 <= 2'b0;
     else if (ui_clk_sync_rst) app_ref_req_r2 <= 2'b0;
     else                      app_ref_req_r2 <= {app_ref_req_r2[0], app_ref_req_r};

   assign app_ref_req_pulse = (app_ref_req_r2 == 2'b01);

   // app_zq_req pulse generator
   always @(posedge ui_clk or posedge async_rst_i)
     if (async_rst_i)          app_zq_req_r2 <= 2'b0;
     else if (ui_clk_sync_rst) app_zq_req_r2 <= 2'b0;
     else                      app_zq_req_r2 <= {app_zq_req_r2[0], app_zq_req_r};

   assign app_zq_req_pulse = (app_zq_req_r2 == 2'b01);

   // app_sr_req pulse generator
   always @(posedge ui_clk or posedge async_rst_i)
     if (async_rst_i)          app_sr_req_r2 <= 2'b0;
     else if (ui_clk_sync_rst) app_sr_req_r2 <= 2'b0;
     else                      app_sr_req_r2 <= {app_sr_req_r2[0], app_sr_req_r};

   assign app_sr_req_pulse = (app_sr_req_r2 == 2'b01);

endmodule

