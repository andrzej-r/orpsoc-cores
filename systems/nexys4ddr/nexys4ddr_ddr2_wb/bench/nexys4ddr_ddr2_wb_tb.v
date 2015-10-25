/*
 * Copyright (c) 2015, Andrzej <ndrwrdck@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and non-source forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in non-source form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS WORK IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * WORK, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

`define USE_TRANSACTOR

`timescale 1ps/1ps

module nexys4ddr_ddr2_wb_tb;

   localparam MEMORY_SIZE_WORDS = 32'h0800_0000;
   localparam WB_PORTS = 1;
   localparam awc =  4;
   localparam dwc = 16;

`include "wb_bfm_params.v"

   vlog_tb_utils vlog_tb_utils0();
   glbl glbl();

   initial begin
      //$dumpfile("testlog.vcd");
      $dumpfile("testlog.lxt");
      $dumpvars(3);
   end

   reg clk = 1'b1;
   reg rst_n = 1'b0;

   initial #100000 rst_n <= 1'b1;
   always #5000 clk <= !clk;

   // DDR2 clock&reset generator
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
      .clk_i              (clk),
      .rst_n_i            (rst_n),
      .async_rst_o        (async_rst),
      .wb_clk_o           (), // generate wb_clk/rst in RAM_ctrl
      .wb_rst_o           (),
      .ddr2_if_sys_clk_o  (ddr2_if_sys_clk),
      .ddr2_if_ref_clk_o  (ddr2_if_ref_clk),
      .ddr2_if_rst_o      (ddr2_if_rst),
      .clk100_o           (clk100)
      );

   wire [WB_PORTS*32-1:0] wb_adr;
   wire [WB_PORTS-1:0]    wb_stb;
   wire [WB_PORTS-1:0] 	  wb_cyc;
   wire [WB_PORTS*3-1:0]  wb_cti;
   wire [WB_PORTS*2-1:0]  wb_bte;
   wire [WB_PORTS-1:0] 	  wb_we;
   wire [WB_PORTS*4-1:0]  wb_sel;
   wire [WB_PORTS*32-1:0] wb_dat;
   wire [WB_PORTS*32-1:0] wb_rdt;
   wire [WB_PORTS-1:0]    wb_ack;

   // control bus
   wire [awc-1:0]         wbc_adr;
   wire                   wbc_stb;
   wire                   wbc_cyc;
   wire                   wbc_we;
   wire [dwc-1:0]         wbc_dat;
   wire [dwc-1:0]         wbc_rdt;
   wire                   wbc_ack;

   wire [31:0]            slave_writes;
   wire [31:0]            slave_reads;
   wire [WB_PORTS-1:0]    done_int;

`ifdef USE_TRANSACTOR
   genvar 	 i;
   generate
      for(i=0;i<WB_PORTS;i=i+1) begin : masters
	 wb_bfm_transactor
	    #(.MEM_HIGH      (MEMORY_SIZE_WORDS*(i+1)-1),
	      .MEM_LOW       (MEMORY_SIZE_WORDS*i),
              .MAX_BURST_LEN (9),
	      .VERBOSE       (1))
	 wb_bfm_transactor0
	    (.wb_clk_i (wb_clk),
	     .wb_rst_i (wb_rst),
	     .wb_adr_o (wb_adr[i*32+:32]),
	     .wb_dat_o (wb_dat[i*32+:32]),
	     .wb_sel_o (wb_sel[i*4+:4]),
	     .wb_we_o  (wb_we[i] ),
	     .wb_cyc_o (wb_cyc[i]),
	     .wb_stb_o (wb_stb[i]),
	     .wb_cti_o (wb_cti[i*3+:3]),
	     .wb_bte_o (wb_bte[i*2+:2]),
	     .wb_dat_i (wb_rdt[i*32+:32]),
	     .wb_ack_i (wb_ack[i]),
	     .wb_err_i (1'b0),
	     .wb_rty_i (1'b0),
	     //Test Control
	     .done(done_int[i]));
      end // block: slaves
   endgenerate

   // Report results
   integer 	 idx;
   time start_time[WB_PORTS-1:0];
   time ack_delay[WB_PORTS-1:0];
   integer num_transactions[WB_PORTS-1:0];

   assign done = &done_int;

   always @(done) begin
      if(done === 1) begin
	 $display("Average wait times");
	 for(idx=0;idx<WB_PORTS;idx=idx+1)
	   $display("Master %0d : %f",idx, ack_delay[idx]/num_transactions[idx]);
	 $display("All tests passed!");
	 $finish;
      end
   end

   generate
      for(i=0;i<WB_PORTS;i=i+1) begin : wait_time
	 initial begin
	    ack_delay[i] = 0;
	    num_transactions[i] = 0;
	    while(1) begin
	       @(posedge wb_cyc[i]);
	       start_time[i] = $time;
	       @(posedge wb_ack[i]);
	       ack_delay[i] = ack_delay[i] + $time-start_time[i];
	       num_transactions[i] = num_transactions[i]+1;
	    end
	 end
      end
   endgenerate
`else // !`ifdef USE_TRANSACTOR
   wb_bfm_master
     #(.MAX_BURST_LENGTH (9))
   bfm
     (.wb_clk_i (wb_clk),
      .wb_rst_i (wb_rst),
      .wb_adr_o (wb_adr[31:0]),
      .wb_dat_o (wb_dat[31:0]),
      .wb_sel_o (wb_sel[3:0]),
      .wb_we_o  (wb_we[0]),
      .wb_cyc_o (wb_cyc[0]),
      .wb_stb_o (wb_stb[0]),
      .wb_cti_o (wb_cti[2:0]),
      .wb_bte_o (wb_bte[1:0]),
      .wb_dat_i (wb_rdt[31:0]),
      .wb_ack_i (wb_ack[0]),
      .wb_err_i (1'b0),
      .wb_rty_i (1'b0));
`endif

   // DDR2 interface signals
   wire [12:0] ddr2_addr;
   wire  [2:0] ddr2_ba;
   wire        ddr2_ras_n;
   wire        ddr2_cas_n;
   wire        ddr2_we_n;
   wire  [0:0] ddr2_ck_p;
   wire  [0:0] ddr2_ck_n;
   wire  [0:0] ddr2_cke;
   wire  [0:0] ddr2_cs_n;
   wire  [1:0] ddr2_dm;
   wire  [0:0] ddr2_odt;
   wire [15:0] ddr2_dq;
   wire  [1:0] ddr2_dqs_p;
   wire  [1:0] ddr2_dqs_n;
   wire        init_calib_complete;

   assign wbc_cyc = 1'b0;

   nexys4ddr_ddr2_wb #
     (
      .awc (awc),
      .dwc (dwc)
      )
   ddr2_wb_0
     (
      .wb_clk_o              (wb_clk),
      .wb_rst_o              (wb_rst),
      .async_rst_i           (async_rst),

      .wb_adr_i              (wb_adr),
      .wb_bte_i              (wb_bte),
      .wb_cti_i              (wb_cti),
      .wb_cyc_i              (wb_cyc),
      .wb_dat_i              (wb_dat),
      .wb_sel_i              (wb_sel),
      .wb_stb_i              (wb_stb),
      .wb_we_i               (wb_we),
      .wb_ack_o              (wb_ack),
      .wb_err_o              (wb_err),
      .wb_rty_o              (wb_rty),
      .wb_dat_o              (wb_rdt),

      .wbc_adr_i             (wbc_adr),
      .wbc_dat_i             (wbc_dat),
      .wbc_we_i              (wbc_we),
      .wbc_cyc_i             (wbc_cyc),
      .wbc_stb_i             (wbc_stb),
      .wbc_dat_o             (wbc_dat),
      .wbc_ack_o             (wbc_ack),
      .wbc_err_o             (wbc_err),
      .wbc_rty_o             (wbc_rty),

      .sys_clk_i             (ddr2_if_sys_clk),
      .clk_ref_i             (ddr2_if_ref_clk),
      .init_calib_complete_o (init_calib_complete),

      .ddr2_addr             (ddr2_addr),
      .ddr2_ba               (ddr2_ba),
      .ddr2_ras_n            (ddr2_ras_n),
      .ddr2_cas_n            (ddr2_cas_n),
      .ddr2_we_n             (ddr2_we_n),
      .ddr2_odt              (ddr2_odt),
      .ddr2_cke              (ddr2_cke),
      .ddr2_dm               (ddr2_dm),
      .ddr2_cs_n             (ddr2_cs_n),
      .ddr2_ck_p             (ddr2_ck_p),
      .ddr2_ck_n             (ddr2_ck_n),
      .ddr2_dq               (ddr2_dq),
      .ddr2_dqs_p            (ddr2_dqs_p),
      .ddr2_dqs_n            (ddr2_dqs_n)
      );



   //DDR2 Memory model
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

`ifndef USE_TRANSACTOR
   reg                err;
   reg [31:0]         address;
   reg [16*32-1:0]    write_data;
   reg [16*32-1:0]    read_data;

   task write_mem;
      input [31:0]   addr_i;
      input [31:0]   data_i;
      input [3:0]    mask_i;
      output 	     err_o;
      begin
         $display("Write, address=%0x data=%0x", addr_i, data_i, mask_i);
         bfm.write(addr_i, data_i, mask_i, err);
         @(posedge wb_clk);
      end
   endtask // write_mem

   integer k;
   initial begin
      bfm.reset;

`ifndef ICARUS_SIM
      @(posedge init_calib_complete);
`endif

      write_mem(32'h0000_0010, 32'h00010203, 4'hf, err);
      bfm.read(32'h0000_0000, read_data, 4'hf, err);
      bfm.read(32'h0000_0001, read_data, 4'hf, err);
      bfm.read(32'h0000_0002, read_data, 4'hf, err);
      bfm.read(32'h0000_0003, read_data, 4'hf, err);
      bfm.read(32'h0000_0000, read_data, 4'hf, err);
      bfm.read(32'h0000_0001, read_data, 4'hf, err);
      bfm.read(32'h0000_0002, read_data, 4'hf, err);
      bfm.read(32'h0000_0003, read_data, 4'hf, err);

      bfm.read_burst(32'h0000_0000, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0004, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0008, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_000c, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0010, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0014, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0018, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_001c, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0000, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0004, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0008, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_000c, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0000, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0001, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0002, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0003, read_data, 4'hf, 1, LINEAR_BURST, err);
      write_mem(32'h0000_001c, 32'h04050607, 4'hf, err);
      bfm.read_burst(32'h0000_0000, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0004, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0008, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_000c, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0010, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0014, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_0018, read_data, 4'hf, 1, LINEAR_BURST, err);
      bfm.read_burst(32'h0000_001c, read_data, 4'hf, 1, LINEAR_BURST, err);

      write_mem(32'h0001_0000, 32'h00010203, 4'hf, err);
      write_mem(32'h0001_0004, 32'h04050607, 4'hf, err);
      write_mem(32'h0001_0008, 32'h08090a0b, 4'hf, err);
      write_mem(32'h0001_000c, 32'h0c0d0e0f, 4'hf, err);
      write_mem(32'h0001_0010, 32'h10111213, 4'hf, err);
      write_mem(32'h0001_0014, 32'h14151617, 4'hf, err);
      write_mem(32'h0001_0018, 32'h18191a1b, 4'hf, err);
      write_mem(32'h0001_001c, 32'h1c1d1e1f, 4'hf, err);

      address = 32'h0001_0000;
      bfm.read_burst(address, read_data, 4'hf, 8, LINEAR_BURST, err);

      write_mem(32'h0002_0000, 32'h01020304, 4'b0001, err);
      write_mem(32'h0002_0004, 32'h01020304, 4'b0010, err);
      write_mem(32'h0002_0008, 32'h01020304, 4'b0100, err);
      write_mem(32'h0002_000c, 32'h01020304, 4'b1000, err);
      write_mem(32'h0002_0010, 32'h01020304, 4'b0011, err);
      write_mem(32'h0002_0014, 32'h01020304, 4'b0110, err);
      write_mem(32'h0002_0014, 32'h01020304, 4'b1100, err);

      address = 32'h0002_0000;
      write_data = 512'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f;
      bfm.write_burst(address, write_data, 4'hf, 16, LINEAR_BURST, err);
      $display("Write, address=%0x data=%0x", address, write_data);
      @(posedge wb_clk);
      bfm.read_burst(address, read_data, 4'hf, 16, LINEAR_BURST, err);
      $display("Read, address=%0x data=%0x", address, read_data);
      @(posedge wb_clk);

      address = 32'h0002_0000;
      bfm.write_burst(address, read_data, 4'hf, 8, LINEAR_BURST, err);
      $display("Read, address=%0x data=%0x", address, read_data);
      @(posedge wb_clk);

      for (k = 1; k <= 8; k = k + 1) begin
         address = 32'h0001_0000;
         bfm.read_burst(address, read_data, 4'hf, k, WRAP_4_BURST, err);
         $display("Read, address=%0x data=%0x", address, read_data);
         @(posedge wb_clk);
         address = 32'h0001_0010;
         bfm.read_burst(address, read_data, 4'hf, k, WRAP_4_BURST, err);
         $display("Read, address=%0x data=%0x", address, read_data);
         @(posedge wb_clk);
      end

      for (k = 1; k <= 8; k = k + 1) begin
         address = 32'h0001_0000;
         bfm.read_burst(address, read_data, 4'hf, k, WRAP_8_BURST, err);
         $display("Read, address=%0x data=%0x", address, read_data);
         @(posedge wb_clk);
      end

      for (k = 1; k <= 8; k = k + 1) begin
         address = 32'h0001_0000;
         bfm.read_burst(address, read_data, 4'hf, k, WRAP_16_BURST, err);
         $display("Read, address=%0x data=%0x", address, read_data);
         @(posedge wb_clk);
      end

      for (k = 1; k <= 8; k = k + 1) begin
         address = 32'h0001_0000;
         bfm.read_burst(address, read_data, 4'hf, k, LINEAR_BURST, err);
         $display("Read, address=%0x data=%0x", address, read_data);
         @(posedge wb_clk);
      end

      for (k = 1; k <= 8; k = k + 1) begin
         address = 32'h0001_0000;
         bfm.read_burst(address, read_data, 4'hf, k, CONSTANT_BURST, err);
         $display("Read, address=%0x data=%0x", address, read_data);
         @(posedge wb_clk);
      end

      address = 32'h0001_0000;
      write_data = 256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f;
      bfm.write_burst(address, write_data, 4'hf, 4, WRAP_4_BURST, err);
      $display("Write, address=%0x data=%0x", address, write_data);
      @(posedge wb_clk);
      bfm.read_burst(address, read_data, 4'hf, 4, WRAP_4_BURST, err);
      $display("Read, address=%0x data=%0x", address, read_data);
      @(posedge wb_clk);

      #3 $finish;
   end
`endif //  `ifndef USE_TRANSACTOR


endmodule

