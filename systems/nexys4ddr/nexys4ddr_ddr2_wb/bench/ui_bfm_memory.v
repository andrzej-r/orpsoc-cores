//
// A simple model of a Xilinx "UI" bus (Xilinx MIG DDR controller user interface).
// A full DDR controller model is rather heavy to simulate and it requires
// encrypted PHY components.
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

module ui_bfm_memory #
  (
   parameter dw = 32,
   parameter aw = 32,
   parameter memory_file = "",
   parameter mem_bytes = 32'h0000_8000, // 32KBytes to reduce simulation time
   parameter verbose = 1
   )
   (
    input               sys_clk_i,
    input               clk_ref_i,
    input [aw-1:0]      app_addr,
    input [2:0]         app_cmd,
    input               app_en,
    input [dw-1:0]      app_wdf_data,
    input               app_wdf_end,
    input [dw/8-1:0]    app_wdf_mask,
    input               app_wdf_wren,
    output reg [dw-1:0] app_rd_data,
    output reg          app_rd_data_end,
    output reg          app_rd_data_valid,
    output reg          app_rdy,
    output reg          app_wdf_rdy,
    input               app_sr_req,
    output              app_sr_active,
    input               app_ref_req,
    output              app_ref_ack,
    input               app_zq_req,
    output              app_zq_ack,
    output reg          ui_clk,
    output reg          ui_clk_sync_rst,
    output reg          init_calib_complete,
    input               sys_rst
    );

   reg [7:0] mem [0:mem_bytes-1];

   // generate clocks and reset
   initial ui_clk = 1'b0;

   integer ui_clk_cnt;
   initial ui_clk_cnt = 0;
   always @(posedge sys_clk_i)
     if (ui_clk_cnt < 1)
       ui_clk_cnt <= ui_clk_cnt + 1;
     else
       begin
          ui_clk_cnt <= 0;
          ui_clk <= ~ui_clk;
       end

   assign app_sr_active = 1'b0;
   assign app_ref_ack = 1'b0;
   assign app_zq_ack = 1'b0;

   integer i;
   integer seed;
   initial
     begin
        seed = 0;
        ui_clk_sync_rst <= 1'b1;
        init_calib_complete <= 1'b0;
        @(posedge sys_rst);
        @(posedge ui_clk);
        ui_clk_sync_rst <= 1'b0;
        for (i = 0; i < 100; i = i + 1)
          @(posedge ui_clk);
        init_calib_complete <= 1'b1;
     end // initial begin

   integer delay2;
   integer j;
   initial
     begin
        app_wdf_rdy <= 1'b0;
        delay2 = $dist_uniform(seed, 1, 2);
        while (1)
          begin
             for (j = 0; j < delay2; j = j + 1)
               @(posedge ui_clk);
             app_wdf_rdy <= ~app_wdf_rdy;
          end
     end // initial begin

   reg [dw-1:0]    wdf_data;
   reg [dw/8-1:0]  wdf_mask;
   always @(posedge ui_clk)
     if (app_wdf_rdy & app_wdf_wren)
       begin
          wdf_data <= app_wdf_data;
          wdf_mask <= app_wdf_mask;
       end

   wire app_strobe = app_rdy & app_en;
   reg [2:0] cmd;
   reg [aw-1:0] addr;
   always @(posedge ui_clk)
     if (app_strobe)
       begin
          addr    <= app_addr;
          cmd     <= app_cmd;
          app_rdy <= 1'b0;
       end
   reg app_strobe_r;
   always @(posedge ui_clk) app_strobe_r <= app_strobe;

   // Grossly simplified but good enough for wb_to_ui bridge
   // This should be modeled as a state machine with a command queue
   integer k;
   always @(posedge app_strobe_r)
     begin
        @(posedge ui_clk);
        @(posedge ui_clk);
        @(posedge ui_clk);
        app_rdy <= 1'b1;
        @(posedge ui_clk);
        if (cmd == 3'b000) // write
          begin
             if (verbose)
               $display("UI write, 0x%h : 0x%h, (0x%h)", addr, wdf_data, wdf_mask);
             for (k = 0; k < dw/8; k = k + 1)
               if (~wdf_mask[k])
                 begin
                    mem[addr+k] <= wdf_data[8*k +: 8];
                    //$display("write, 0x%h : 0x%h", addr+k, wdf_data[8*k +: 8]);
                 end
          end
        else if (cmd == 3'b001) // read
          begin
             for (k = 0; k < dw/8; k = k + 1)
               app_rd_data[8*k +: 8] = mem[addr+k];
             if (verbose)
               $display("UI read,  0x%h : 0x%h", addr, app_rd_data);
             app_rd_data_valid <= 1'b1;
             app_rd_data_end <= 1'b1;
             @(posedge ui_clk);
             app_rd_data_valid <= 1'b0;
             app_rd_data_end <= 1'b0;
             app_rd_data <= 128'bx;
          end
     end
endmodule // ui_bfm_memory
