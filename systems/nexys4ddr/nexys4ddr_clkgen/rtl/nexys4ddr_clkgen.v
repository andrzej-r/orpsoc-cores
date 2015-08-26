//
// Clock, reset generation unit for Nexys-4 DDR board
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

module nexys4ddr_clkgen
  (
   // Main clocks in, depending on board
   input  clk_i,
   // Asynchronous, active low reset in
   input  rst_n_i,
   // Input reset - through a buffer, asynchronous
   output async_rst_o,

   // Wishbone clock (66MHz) and reset
   output wb_clk_o,
   output wb_rst_o,

   // DDR2 DRAM clocks (266 & 200MHz) & reset
   output ddr2_if_sys_clk_o,
   output ddr2_if_ref_clk_o,
   output ddr2_if_rst_o,

   // an extra 100MHz clock
   output clk100_o
   );

   IBUFG ibufg_clk
     (
      .O (clk_buf),
      .I (clk_i)
      );

   IBUFG ibufg_rst_n
     (
      .O (rst_n_buf),
      .I (rst_n_i)
      );

   assign async_rst_o = ~rst_n_buf;

   // Clock synthesizer
   wire clkfb;
   wire clkfb_buf;
   wire ddr2_if_sys_clk;
   wire ddr2_if_ref_clk;
   wire clk100;
   wire wb_clk;
   wire locked;
   MMCM_ADV #
     (
      .BANDWIDTH            ("OPTIMIZED"),
      .CLKOUT4_CASCADE      ("FALSE"),
      .CLOCK_HOLD           ("FALSE"),
      .COMPENSATION         ("ZHOLD"),
      .STARTUP_WAIT         ("FALSE"),
      .DIVCLK_DIVIDE        (1),
      .CLKFBOUT_MULT_F      (8.000), // 800MHz
      .CLKFBOUT_PHASE       (0.000),
      .CLKFBOUT_USE_FINE_PS ("FALSE"),
      .CLKOUT0_DIVIDE_F     (3.000), // 266MHz
      .CLKOUT0_PHASE        (180.000),
      .CLKOUT0_DUTY_CYCLE   (0.500),
      .CLKOUT0_USE_FINE_PS  ("FALSE"),
      .CLKOUT1_DIVIDE       (4),     // 200MHz
      .CLKOUT1_PHASE        (0.000),
      .CLKOUT1_DUTY_CYCLE   (0.500),
      .CLKOUT1_USE_FINE_PS  ("FALSE"),
      .CLKOUT2_DIVIDE       (8),
      .CLKOUT2_PHASE        (0.000),
      .CLKOUT2_DUTY_CYCLE   (0.500),
      .CLKOUT2_USE_FINE_PS  ("FALSE"),
      //.CLKOUT3_DIVIDE       (16), //50MHz
      //.CLKOUT3_DIVIDE       (6),  //133MHz (timing violations)
      //.CLKOUT3_DIVIDE       (9),  //89MHz (timing violations in some runs)
      .CLKOUT3_DIVIDE       (12),  //66MHz
      .CLKOUT3_PHASE        (0.000),
      .CLKOUT3_DUTY_CYCLE   (0.500),
      .CLKOUT3_USE_FINE_PS  ("FALSE"),
      .CLKIN1_PERIOD        (10.000),
      .REF_JITTER1          (0.010))
   mmcm_adv_0
     (
      // Output clocks
      .CLKFBOUT             (clkfb),
      .CLKFBOUTB            (),
      .CLKOUT0              (ddr2_if_sys_clk),
      .CLKOUT0B             (),
      .CLKOUT1              (ddr2_if_ref_clk),
      .CLKOUT1B             (),
      .CLKOUT2              (clk100),
      .CLKOUT2B             (),
      .CLKOUT3              (wb_clk),
      .CLKOUT3B             (),
      .CLKOUT4              (),
      .CLKOUT5              (),
      .CLKOUT6              (),
      // Input clock control
      .CLKFBIN              (clkfb_buf),
      .CLKIN1               (clk_buf),
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
      .LOCKED               (locked),
      .CLKINSTOPPED         (),
      .CLKFBSTOPPED         (),
      .PWRDWN               (1'b0),
      .RST                  (async_rst_o)
      );

   // clock buffers
   BUFG bufg_clkfb
     (
      .O (clkfb_buf),
      .I (clkfb)
      );

   BUFG bufg_ddr2_if_sys_clk
     (
      .O   (ddr2_if_sys_clk_o),
      .I   (ddr2_if_sys_clk)
      );

   BUFG bufg_ddr2_if_ref_clk
     (
      .O   (ddr2_if_ref_clk_o),
      .I   (ddr2_if_ref_clk)
      );

   BUFG bufg_clk100
     (
      .O   (clk100_o),
      .I   (clk100)
      );

   BUFG bufg_wb_clk
     (
      .O   (wb_clk_o),
      .I   (wb_clk)
      );

   // Reset generator and buffers
   reg [15:0] rst_shift_r;
   always @(posedge wb_clk_o or posedge async_rst_o)
     if   (async_rst_o) rst_shift_r <= 16'hffff;
     else               rst_shift_r <= {rst_shift_r[14:0], ~locked};

   BUFG bufg_wb_rst
     (
      .O   (wb_rst_o),
      .I   (rst_shift_r[15])
      );

   BUFG bufg_ddr2_if_rst
     (
      .O   (ddr2_if_rst_o),
      .I   (rst_shift_r[15])
      );

endmodule
