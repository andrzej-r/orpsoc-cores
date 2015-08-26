//
// Wishbone wrapper for Xilinx XADC
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


module nexys4ddr_xadc_wb #
  (
   parameter NEXYS4DDR_XADC_SIM_MONITOR_FILE = "../src/nexys4ddr_xadc/nexys4ddr_xadc/simulation/timing/nexys4ddr_xadc_sim.txt",
   parameter dw = 16,
   parameter aw =  8
   )
   (
    input                  wb_clk_i,
    input                  wb_rst_i,
    input                  async_rst_i,

    // Wishbone Interface
    input  [aw-1:0]        wb_adr_i,
    input  [dw-1:0]        wb_dat_i,
    input  [3:0]           wb_sel_i,
    input                  wb_we_i,
    input                  wb_cyc_i,
    input                  wb_stb_i,
    input  [2:0]           wb_cti_i,
    input  [1:0]           wb_bte_i,
    output reg [dw-1:0]    wb_dat_o,
    output reg             wb_ack_o,
    output reg             wb_err_o,
    output                 wb_rty_o,

    output [11:0]          device_temp_o,

    // xadc interrupts
    output                 irq_o
    );

   localparam nirq = 11;
   reg  [nirq-1:0] IRQ_flag_reg;

   // address decoder (
   localparam           aw_dec = 3;
   reg [2**aw_dec-1:0]  sel;
   integer              j;
   always @(*)
     begin
        sel = {2**aw_dec{1'b0}};
        for (j = 0; j < 2**aw_dec; j = j + 1)
          if (wb_adr_i[7] == 0 && wb_adr_i == j) // only decode lower 128 words
            sel[j] = 1'b1;
     end

   // enable register, currently unused (use DRD registers to power down XADC)
   reg enable_reg;
   always @(posedge wb_clk_i or posedge async_rst_i)
     if (async_rst_i)
       enable_reg <= 1'b0;
     else if (wb_rst_i)
       enable_reg <= 1'b0;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[0])
       enable_reg <= wb_dat_i[0];

   // mask IRQ register
   reg [nirq-1:0] IRQ_mask_reg;
   always @(posedge wb_clk_i or posedge async_rst_i)
     if (async_rst_i)
       IRQ_mask_reg <= ({nirq{1'b0}});
     else if (wb_rst_i)
       IRQ_mask_reg <= ({nirq{1'b0}});
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[1])
       IRQ_mask_reg <= wb_dat_i[nirq-1:0];


   // Wishbone to Xilinx DRD I/F
   wire        xadc_dclk   = wb_clk_i;
   wire        xadc_reset  = wb_rst_i;

   wire  [6:0] xadc_daddr  = wb_adr_i  [6:0];
   wire [15:0] xadc_di     = wb_dat_i [15:0];
   wire        xadc_dwe    = wb_we_i;

   wire        xadc_den    = wb_adr_i[7] & wb_cyc_i & wb_stb_i; // upper 128 words re-directed to DRD

   // read back register values
   wire  [15:0] xadc_do;
   always @(posedge wb_clk_i)
     if (wb_rst_i)
       wb_dat_o <= 32'b0;
     else if (wb_cyc_i)
       begin
          wb_dat_o <= 0;
          if (sel[0])      wb_dat_o[0]        <= enable_reg;
          if (sel[2])      wb_dat_o[nirq-1:0] <= IRQ_mask_reg;
          if (sel[3])      wb_dat_o[nirq-1:0] <= IRQ_flag_reg;
          if (wb_adr_i[7]) wb_dat_o[15:0]     <= xadc_do;
       end

   // Ack generation
   always @(posedge wb_clk_i)
     if (wb_rst_i)
       wb_ack_o <= 1'b0;
     else if (wb_ack_o)
       wb_ack_o <= 1'b0;
     else if (wb_cyc_i & wb_stb_i & ~wb_ack_o & wb_adr_i[7]) // upper 128 words
       wb_ack_o <= (wb_we_i | xadc_drdy); // if read, wait for xadc_drdy
     else if (wb_cyc_i & wb_stb_i & ~wb_ack_o)
       wb_ack_o <= 1'b1;

   // Err generation
   always @(posedge wb_clk_i)
     if (wb_rst_i)
       wb_err_o <= 1'b0;
     else if (wb_err_o)
       wb_err_o <= 1'b0;
     else if (wb_cyc_i & wb_stb_i & ~wb_err_o & (xadc_jtag_busy | xadc_jtag_locked))
       wb_err_o <= 1'b1;

   assign wb_rty_o = 1'b0;


   wire xadc_eoc;
   wire xadc_eos;
   wire xadc_jtag_busy;
   wire xadc_jtag_locked;
   wire xadc_jtag_modified;
   wire xadc_ot_alarm;
   wire xadc_vccaux_alarm;
   wire xadc_vccint_alarm;
   wire xadc_user_temp_alarm;
   wire xadc_vbram_alarm;
   wire xadc_alarm;

   nexys4ddr_xadc #
     (
      .NEXYS4DDR_XADC_SIM_MONITOR_FILE (NEXYS4DDR_XADC_SIM_MONITOR_FILE)
      )
   xadc_0
     (
      .DADDR_IN             (xadc_daddr),           // Address bus for the dynamic reconfiguration port
      .DCLK_IN              (xadc_dclk),            // Clock input for the dynamic reconfiguration port
      .DEN_IN               (xadc_den),             // Enable Signal for the dynamic reconfiguration port
      .DI_IN                (xadc_di),              // Input data bus for the dynamic reconfiguration port
      .DWE_IN               (xadc_dwe),             // Write Enable for the dynamic reconfiguration port
      .RESET_IN             (xadc_reset),           // Reset signal for the System Monitor control logic
      // Aux inputs, do not connect (implied)
      .VAUXP2               (),            // Auxiliary channel 2
      .VAUXN2               (),
      .VAUXP3               (),            // Auxiliary channel 3
      .VAUXN3               (),
      .VAUXP10              (),            // Auxiliary channel 10
      .VAUXN10              (),
      .VAUXP11              (),            // Auxiliary channel 11
      .VAUXN11              (),
      .BUSY_OUT             (xadc_busy),            // ADC Busy signal
      .CHANNEL_OUT          (),                     // Channel Selection Outputs
      .DO_OUT               (xadc_do),              // Output data bus for dynamic reconfiguration port
      .DRDY_OUT             (xadc_drdy),  //??          // Data ready signal for the dynamic reconfiguration port
      .EOC_OUT              (xadc_eoc),             // End of Conversion Signal
      .EOS_OUT              (xadc_eos),             // End of Sequence Signal
      .JTAGBUSY_OUT         (xadc_jtag_busy),       // JTAG DRP transaction is in progress signal
      .JTAGLOCKED_OUT       (xadc_jtag_locked),     // DRP port lock request has been made by JTAG
      .JTAGMODIFIED_OUT     (xadc_jtag_modified),   // Indicates JTAG Write to the DRP has occurred
      .OT_OUT               (xadc_ot_alarm),        // Over-Temperature alarm output
      .VCCAUX_ALARM_OUT     (xadc_vccaux_alarm),    // VCCAUX-sensor alarm output
      .VCCINT_ALARM_OUT     (xadc_vccint_alarm),    // VCCINT-sensor alarm output
      .USER_TEMP_ALARM_OUT  (xadc_user_temp_alarm), // Temperature-sensor alarm output
      .VBRAM_ALARM_OUT      (xadc_vbram_alarm),
      .ALARM_OUT            (xadc_alarm),           // OR'ed output of all the Alarms
      // Main input, do not connect (implied)
      .VP_IN                (),                     // Dedicated Analog Input Pair
      .VN_IN                ()
      );

   // IRQ flags
   // write '1' to clear it
   wire [nirq-1:0] xadc_alarms =
                   {xadc_alarm, xadc_vbram_alarm, xadc_user_temp_alarm, xadc_vccint_alarm,
                    xadc_vccaux_alarm, xadc_ot_alarm, xadc_jtag_modified, xadc_jtag_locked,
                    xadc_jtag_busy, xadc_eos, xadc_eoc};
   genvar i;
   generate
      for (i = 0; i < nirq; i = i + 1) begin: irq_gen
         always @(posedge wb_clk_i or posedge async_rst_i)
           if (async_rst_i)
             IRQ_flag_reg[i] <= 1'b0;
           else if (wb_rst_i)
             IRQ_flag_reg[i] <= 1'b0;
           else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[3] & wb_dat_i[i])
             IRQ_flag_reg[i] <= 1'b0;
           else if (xadc_alarms[i])
             IRQ_flag_reg[i] <= 1'b1;
      end
   endgenerate

   assign irq_o = |(IRQ_flag_reg & IRQ_mask_reg);

endmodule // nexys4ddr_xadc_wb

