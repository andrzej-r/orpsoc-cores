//
// Simple Wishbone to UI (Xilinx MIG DDR controller user interface) adapter
// No caching nor prefetching. Write bursts split into separate DRAM requests.
// Read bursts propagated to DRAM if aligned with 128b boundaries and
// split into bursts of 128b otherwise.
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

// Latch APP_WDF_DATA
// Makes timing closure easier but requires extra cycles for burst writes
`define SYNC_APP_WDF_DATA

module nexys4ddr_ddr2_wb_to_ui
  (
    input                                        clk_i,
    input                                        rst_i,
    input                                        async_rst_i,

    // Wishbone Interface
    input       [31:0]                           wb_adr_i,
    input       [31:0]                           wb_dat_i,
    input        [3:0]                           wb_sel_i,
    input                                        wb_we_i,
    input                                        wb_cyc_i,
    input                                        wb_stb_i,
    input        [2:0]                           wb_cti_i,
    input        [1:0]                           wb_bte_i,
    output reg  [31:0]                           wb_dat_o,
    output reg                                   wb_ack_o,
    output                                       wb_err_o,
    output                                       wb_rty_o,

    // DDR2
    // user interface signals
    output reg  [26:0]                           app_addr_o,
    output reg   [2:0]                           app_cmd_o,
    output reg                                   app_en_o,
    output     [127:0]                           app_wdf_data_o,
    output reg                                   app_wdf_end_o,
    output reg  [15:0]                           app_wdf_mask_o,
    output reg                                   app_wdf_wren_o,
    input      [127:0]                           app_rd_data_i,
    input                                        app_rd_data_end_i,
    input                                        app_rd_data_valid_i,
    input                                        app_rdy_i,
    input                                        app_wdf_rdy_i
    );

   function [31:0] swap_bytes;
      input [31:0] value;
      swap_bytes = {value[7:0], value[15:8], value[23:16], value[31:24]};
   endfunction

   function [3:0] swap_bits;
      input [3:0] value;
      swap_bits = {value[0], value[1], value[2], value[3]};
   endfunction

   // Wishbone bursting defines
   localparam [2:0]
     WB_CLASSIC     = 3'b000,
     WB_BURST_CONST = 3'b001,
     WB_BURST_INC   = 3'b010,
     WB_BURST_END   = 3'b111;
   localparam [1:0]
     BURST_LINEAR   = 2'b00,
     BURST_WRAP4    = 2'b01,
     BURST_WRAP8    = 2'b10,
     BURST_WRAP16   = 2'b11;

   // state register encoding
   localparam [3:0]
     STATE_IDLE               = 4'b0000,
     STATE_WRITE_CMD          = 4'b0100,
     STATE_WRITE_DATA         = 4'b0101,
     STATE_WRITE_ACK          = 4'b0110,
     STATE_WRITE_ACK_CMD      = 4'b0111,
     STATE_READ_CMD           = 4'b1000,
     STATE_READ_WAIT_RD_DATA  = 4'b1001,
     STATE_READ_SEND_RD_DATA  = 4'b1010;

   wire classic       = wb_cyc_i &  wb_stb_i & wb_cti_i == WB_CLASSIC;
   wire burst_end     = wb_cyc_i &  wb_cti_i == WB_BURST_END;
   wire burst         = wb_cyc_i & (wb_cti_i == WB_BURST_CONST || wb_cti_i == WB_BURST_INC);
   wire read          = wb_cyc_i & ~wb_we_i;
   wire write         = wb_cyc_i &  wb_we_i;

   reg [3:0] state_r;
   reg [3:0] state_new;

   // address of currently selected 32b word
   reg [26:0] address_burst_new;
   reg [26:0] address_burst_r;
   always @(*)
     if (state_r == STATE_IDLE)
       begin
          address_burst_new = wb_adr_i[26:0];
       end
     else if (state_new == STATE_WRITE_ACK_CMD || state_new == STATE_READ_SEND_RD_DATA)
       begin
          address_burst_new[1:0] = 2'b00;
          case (wb_bte_i)
            BURST_LINEAR: address_burst_new[26:2] = address_burst_r[26:2] + 1;
            BURST_WRAP4 : address_burst_new[ 3:2] = address_burst_r[ 3:2] + 1;
            BURST_WRAP8 : address_burst_new[ 4:2] = address_burst_r[ 4:2] + 1;
            BURST_WRAP16: address_burst_new[ 5:2] = address_burst_r[ 5:2] + 1;
          endcase // case (wb_bte_i)
       end // else: !if(state_r == STATE_IDLE)
     else
       begin
          address_burst_new = address_burst_r;
       end

   always @(posedge clk_i or posedge async_rst_i)
     if      (async_rst_i) address_burst_r <= 0;
     else if (rst_i)       address_burst_r <= 0;
     else                  address_burst_r <= address_burst_new;

   reg [26:0] address_burst_r2;
   always @(posedge clk_i or posedge async_rst_i)
     if      (async_rst_i) address_burst_r2 <= 0;
     else if (rst_i)       address_burst_r2 <= 0;
     else                  address_burst_r2 <= address_burst_r;

   // Schedule new request when read address moves beyond 128b boundary.
   wire miss = (address_burst_r[26:4] != address_burst_r2[26:4]);

   wire [6:0] app_addr_offs = {3'b000, address_burst_r[3:0]};

   // write data and mask
   reg [151:0] app_wdf_data_tmp;
   always @(*)
     begin
        app_wdf_data_tmp = 152'b0;
        app_wdf_data_tmp[(app_addr_offs<<3) +: 32] = swap_bytes(wb_dat_i);
     end

   reg [18:0] app_wdf_mask_tmp;
   reg [15:0] app_wdf_mask_r;
   always @(*)
     begin
        app_wdf_mask_tmp = {19{1'b1}};
        app_wdf_mask_tmp[app_addr_offs +: 4] = ~swap_bits(wb_sel_i);
     end

   // read data mux
   wire [6:0] rd_app_addr_offs = {3'b000, address_burst_r2[3:2], 2'b00};

   // latch received data (app_rd_data_i stable only during app_rd_data_valid_i
   reg [127:0] app_rd_data_r;
   always @(posedge clk_i)
     if (app_rd_data_valid_i)
       app_rd_data_r <= app_rd_data_i;

   wire [31:0] rd_data_selected = app_rd_data_r [(rd_app_addr_offs<<3) +: 32];

   // state machine transitions
   always @(*)
     begin
        state_new = state_r;
        case (state_r)
          STATE_IDLE:
            if (classic | burst_end | burst)
              if (write)
                state_new = STATE_WRITE_CMD;
              else
                state_new = STATE_READ_CMD;
          STATE_WRITE_CMD:
            if (app_rdy_i)
              state_new = STATE_WRITE_DATA;
          STATE_WRITE_DATA:
            if (app_wdf_rdy_i)
              if (classic | burst_end)
                state_new = STATE_WRITE_ACK;
              else
`ifdef SYNC_APP_WDF_DATA
                state_new = STATE_WRITE_ACK;
`else
                state_new = STATE_WRITE_ACK_CMD;
`endif
          STATE_WRITE_ACK:
            state_new = STATE_IDLE;
          STATE_WRITE_ACK_CMD:
            if (app_rdy_i)
              state_new = STATE_WRITE_DATA;
            else
              state_new = STATE_WRITE_CMD;
          STATE_READ_CMD:
            if (app_rdy_i)
              state_new = STATE_READ_WAIT_RD_DATA;
          STATE_READ_WAIT_RD_DATA:
            if (app_rd_data_valid_i)
              state_new = STATE_READ_SEND_RD_DATA;
          STATE_READ_SEND_RD_DATA:
            if (classic | burst_end)
              state_new = STATE_IDLE;
            else if (miss)
              state_new = STATE_READ_CMD;
          default:
            state_new = STATE_IDLE;
        endcase // case (state_r)
     end // always @ (*)

   // state machine register and outputs
   reg [127:0] app_wdf_data_r;
   always @(posedge clk_i or posedge async_rst_i)
     if (async_rst_i)
       begin
          app_en_o            <= 1'b0;
          app_cmd_o           <= 3'b000;
          app_addr_o          <= 27'h0;
          app_wdf_mask_o      <= 16'hffff;
          app_wdf_wren_o      <= 1'b0;
          app_wdf_end_o       <= 1'b1;
          app_wdf_data_r      <= 128'h0;
          wb_ack_o            <= 1'b0;
          wb_dat_o            <= 32'h0;
          state_r             <= STATE_IDLE;
       end
     else if (rst_i)
       begin
          app_en_o            <= 1'b0;
          app_cmd_o           <= 3'b000;
          app_addr_o          <= 27'h0;
          app_wdf_mask_o      <= 16'hffff;
          app_wdf_wren_o      <= 1'b0;
          app_wdf_end_o       <= 1'b1;
          app_wdf_data_r      <= 128'h0;
          wb_ack_o            <= 1'b0;
          wb_dat_o            <= 32'h0;
          state_r             <= STATE_IDLE;
       end
     else
       begin
          app_en_o            <= 1'b0;
          app_cmd_o           <= 3'b000;
          app_addr_o          <= 27'h0;
          app_wdf_mask_o      <= 16'hffff;
          app_wdf_wren_o      <= 1'b0;
          app_wdf_end_o       <= 1'b1;
          app_wdf_data_r      <= 128'h0;
          wb_ack_o            <= 1'b0;
          wb_dat_o            <= 32'h0;
          state_r             <= state_new;
          case (state_new)
            STATE_WRITE_CMD:
              begin
                 app_en_o       <= 1'b1;
                 app_cmd_o      <= 3'b000;
                 app_addr_o     <= {address_burst_new[26:4], 4'b0000};
             end
            STATE_WRITE_DATA:
              begin
                 app_wdf_wren_o <= 1'b1;
                 app_wdf_mask_o <= app_wdf_mask_tmp[15:0];
                 app_wdf_end_o  <= 1'b1;
                 app_wdf_data_r <= app_wdf_data_tmp[127:0];
              end
            STATE_WRITE_ACK:
              begin
                 wb_ack_o       <= 1'b1;
              end
            STATE_WRITE_ACK_CMD:
              begin
                 wb_ack_o       <= 1'b1;
                 app_en_o       <= 1'b1;
                 app_cmd_o      <= 3'b000;
                 app_addr_o     <= {address_burst_new[26:4], 4'b0000};
              end
            STATE_READ_CMD:
              begin
                 app_en_o       <= 1'b1;
                 app_cmd_o      <= 3'b001;
                 app_addr_o     <= {address_burst_new[26:4], 4'b0000};
              end
            STATE_READ_SEND_RD_DATA:
              begin
                 wb_ack_o       <= 1'b1;
                 wb_dat_o       <= swap_bytes(rd_data_selected);
              end
            default:
              begin
                 app_en_o       <= 1'b0;
                 app_cmd_o      <= 3'b000;
                 app_addr_o     <= 27'h0;
                 app_wdf_mask_o <= 16'hffff;
                 app_wdf_wren_o <= 1'b0;
                 app_wdf_end_o  <= 1'b1;
                 app_wdf_data_r <= 128'h0;
                 wb_ack_o       <= 1'b0;
                 wb_dat_o       <= 32'h0;
              end
          endcase // case (state_new)
       end // else: !if(rst_i)

   assign wb_err_o = wb_ack_o & 1'b0;
   assign wb_rty_o = wb_ack_o & 1'b0;

`ifdef SYNC_APP_WDF_DATA
   assign app_wdf_data_o = app_wdf_data_r;
`else
   assign app_wdf_data_o = app_wdf_data_tmp[127:0];
`endif
endmodule // wb_to_ui
