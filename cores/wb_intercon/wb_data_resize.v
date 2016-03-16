module wb_data_resize
  #(parameter aw  = 32, //Address width
    parameter mdw = 32, //Master Data Width
    parameter sdw = 8) //Slave Data Width
   (//Wishbone Master interface
    input      [aw-1:0]  wbm_adr_i,
    input      [mdw-1:0] wbm_dat_i,
    input      [3:0]     wbm_sel_i,
    input                wbm_we_i,
    input                wbm_cyc_i,
    input                wbm_stb_i,
    input      [2:0]     wbm_cti_i,
    input      [1:0]     wbm_bte_i,
    output reg [mdw-1:0] wbm_dat_o,
    output               wbm_ack_o,
    output reg           wbm_err_o,
    output               wbm_rty_o,
    // Wishbone Slave interface
    output reg [aw-1:0]  wbs_adr_o,
    output reg [sdw-1:0] wbs_dat_o,
    output reg           wbs_we_o,
    output reg           wbs_cyc_o,
    output reg           wbs_stb_o,
    output     [2:0]     wbs_cti_o,
    output     [1:0]     wbs_bte_o,
    input      [sdw-1:0] wbs_dat_i,
    input                wbs_ack_i,
    input                wbs_err_i,
    input                wbs_rty_i);

   generate
      if (sdw <= 8)
        begin : access_byte_gen
           always @(*)
             begin
                wbs_adr_o = wbm_adr_i;
                wbm_err_o = wbs_err_i;
                wbs_dat_o = {sdw{1'b0}};
                wbm_dat_o = {mdw{1'b0}};
                wbs_we_o  = wbm_we_i;
                wbs_cyc_o = wbm_cyc_i;
                wbs_stb_o = wbm_stb_i;
                case (wbm_sel_i)
                  4'b1000:
                    begin
                       wbs_adr_o[1:0]   = 2'd0;
                       wbs_dat_o        = wbm_dat_i[31:24];
                       wbm_dat_o[31:24] = wbs_dat_i;
                    end
                  4'b0100:
                    begin
                       wbs_adr_o[1:0]   = 2'd1;
                       wbs_dat_o        = wbm_dat_i[23:16];
                       wbm_dat_o[23:16] = wbs_dat_i;
                    end
                  4'b0010:
                    begin
                       wbs_adr_o[1:0]   = 2'd2;
                       wbs_dat_o        = wbm_dat_i[15:8];
                       wbm_dat_o[15:8]  = wbs_dat_i;
                    end
                  4'b0001:
                    begin
                       wbs_adr_o[1:0]   = 2'd3;
                       wbs_dat_o        = wbm_dat_i[7:0];
                       wbm_dat_o[7:0]   = wbs_dat_i;
                    end
                  default:
                    begin
                       wbm_err_o = 1'b1; // error on unaligned access
                       wbs_we_o  = 1'b0;
                       wbs_cyc_o = 1'b0;
                       wbs_stb_o = 1'b0;
                    end
                endcase // case (wbm_sel_i)
             end // always @ (*)
        end // block: access_byte_gen
      else if (sdw <= 16)
        begin : access_half_word_gen
           always @(*)
             begin
                wbs_adr_o = wbm_adr_i;
                wbm_err_o = wbs_err_i;
                wbs_dat_o = {sdw{1'b0}};
                wbm_dat_o = {mdw{1'b0}};
                wbs_we_o  = wbm_we_i;
                wbs_cyc_o = wbm_cyc_i;
                wbs_stb_o = wbm_stb_i;
                case (wbm_sel_i)
                  4'b1100:
                    begin
                       wbs_adr_o[1:0]   = 2'd0;
                       wbs_dat_o        = wbm_dat_i[31:16];
                       wbm_dat_o[31:16] = wbs_dat_i;
                    end
                  4'b0011:
                    begin
                       wbs_adr_o[1:0]   = 2'd2;
                       wbs_dat_o        = wbm_dat_i[15:0];
                       wbm_dat_o[15:0]  = wbs_dat_i;
                    end
                  default:
                    begin
                       wbm_err_o = 1'b1; // error on unaligned access
                       wbs_we_o  = 1'b0;
                       wbs_cyc_o = 1'b0;
                       wbs_stb_o = 1'b0;
                    end
                endcase // case (wbm_sel_i)
             end // always @ (*)
        end // block: access_half_word_gen
      else
        begin : access_word_gen
           always @(*)
             begin
                wbs_adr_o = wbm_adr_i;
                wbs_dat_o = wbm_dat_i;
                wbs_we_o  = wbm_we_i;
                wbs_cyc_o = wbm_cyc_i;
                wbs_stb_o = wbm_stb_i;
                wbm_dat_o = wbs_dat_i;
                wbm_err_o = wbs_err_i;
             end // always @ (*)
        end // block: access_word_gen
   endgenerate

   assign wbs_cti_o = wbm_cti_i;
   assign wbs_bte_o = wbm_bte_i;

   assign wbm_ack_o = wbs_ack_i;
   assign wbm_rty_o = wbs_rty_i;
endmodule
