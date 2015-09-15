module wb_data_resize
  #(parameter aw  = 32, //Address width
    parameter mdw = 32, //Master Data Width
    parameter sdw = 8) //Slave Data Width
   (//Wishbone Master interface
    input  [aw-1:0]  wbm_adr_i,
    input  [mdw-1:0] wbm_dat_i,
    input  [3:0]     wbm_sel_i,
    input 	     wbm_we_i,
    input 	     wbm_cyc_i,
    input 	     wbm_stb_i,
    input  [2:0]     wbm_cti_i,
    input  [1:0]     wbm_bte_i,
    output [mdw-1:0] wbm_dat_o,
    output 	     wbm_ack_o,
    output 	     wbm_err_o,
    output 	     wbm_rty_o, 
    // Wishbone Slave interface
    output [aw-1:0]  wbs_adr_o,
    output [sdw-1:0] wbs_dat_o,
    output 	     wbs_we_o,
    output 	     wbs_cyc_o,
    output 	     wbs_stb_o,
    output [2:0]     wbs_cti_o,
    output [1:0]     wbs_bte_o,
    input  [sdw-1:0] wbs_dat_i,
    input 	     wbs_ack_i,
    input 	     wbs_err_i,
    input 	     wbs_rty_i);

   assign wbs_adr_o[aw-1:2] = wbm_adr_i[aw-1:2];

   reg [1:0] wbs_adr_o2;
   always @(*)
     case (wbm_sel_i)
       4'b1000: wbs_adr_o2 = 2'd0; // 8b access
       4'b1100: wbs_adr_o2 = 2'd0; //16b access
       4'b1111: wbs_adr_o2 = 2'd0; //32b access
       4'b0100: wbs_adr_o2 = 2'd1; // 8b access
       4'b0010: wbs_adr_o2 = 2'd2; // 8b access
       4'b0011: wbs_adr_o2 = 2'd2; //16b access
       4'b0001: wbs_adr_o2 = 2'd3; // 8b access
       default: wbs_adr_o2 = 2'd0; // unaligned access
     endcase // case (wbm_sel_i)
   assign wbs_adr_o[1:0] = wbs_adr_o2;

   reg [31:0] wbs_dat_o32;
   reg [31:0] wbm_dat_i32;
   always @(*) begin
      wbs_dat_o32 = 32'b0;
      wbm_dat_i32 = 32'b0;
      wbm_dat_i32[mdw-1:0] = wbm_dat_i; // extended to 32b
      case (wbm_sel_i)
        4'b1000: wbs_dat_o32[ 7:0] = wbm_dat_i32[ 7:0]; // 8b access
        4'b1100: wbs_dat_o32[15:0] = wbm_dat_i32[15:0]; //16b access
        4'b1111: wbs_dat_o32[31:0] = wbm_dat_i32[31:0]; //32b access
        4'b0100: wbs_dat_o32[ 7:0] = wbm_dat_i32[ 7:0]; // 8b access
        4'b0010: wbs_dat_o32[ 7:0] = wbm_dat_i32[ 7:0]; // 8b access
        4'b0011: wbs_dat_o32[15:0] = wbm_dat_i32[15:0]; //16b access
        4'b0001: wbs_dat_o32[ 7:0] = wbm_dat_i32[ 7:0]; // 8b access
      endcase // case (wbm_sel_i)
   end
   assign wbs_dat_o = wbs_dat_o32[sdw-1:0];

   assign wbs_we_o  = wbm_we_i;

   assign wbs_cyc_o = wbm_cyc_i;
   assign wbs_stb_o = wbm_stb_i;

   assign wbs_cti_o = wbm_cti_i;
   assign wbs_bte_o = wbm_bte_i;

   reg [31:0] wbm_dat_o32;
   reg [31:0] wbs_dat_i32;
   always @(*) begin
      wbm_dat_o32 = 32'b0;
      wbs_dat_i32 = 32'b0;
      wbs_dat_i32[sdw-1:0] = wbs_dat_i; // extended to 32b
      case (wbm_sel_i)
        4'b1000: wbm_dat_o32[ 7:0] = wbs_dat_i32[ 7:0]; // 8b access
        4'b1100: wbm_dat_o32[15:0] = wbs_dat_i32[15:0]; //16b access
        4'b1111: wbm_dat_o32[31:0] = wbs_dat_i32[31:0]; //32b access
        4'b0100: wbm_dat_o32[ 7:0] = wbs_dat_i32[ 7:0]; // 8b access
        4'b0010: wbm_dat_o32[ 7:0] = wbs_dat_i32[ 7:0]; // 8b access
        4'b0011: wbm_dat_o32[15:0] = wbs_dat_i32[15:0]; //16b access
        4'b0001: wbm_dat_o32[ 7:0] = wbs_dat_i32[ 7:0]; // 8b access
      endcase // case (wbm_sel_i)
   end
   assign wbm_dat_o = wbm_dat_o32[mdw-1:0];

   assign wbm_ack_o = wbs_ack_i;
   assign wbm_err_o = wbs_err_i;
   assign wbm_rty_o = wbs_rty_i;
   
endmodule
