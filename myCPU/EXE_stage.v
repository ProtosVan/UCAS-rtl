`include "mycpu.h"

module exe_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ms_allowin    ,
    output                         es_allowin    ,
    //from ds
    input                          ds_to_es_valid,
    input  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to ms
    output                         es_to_ms_valid,
    output [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    // data sram interface
    output        data_sram_en   ,
    output [ 3:0] data_sram_wen  ,
    output [31:0] data_sram_addr ,
    output [31:0] data_sram_wdata,

    output [37:0] stuck_es_to_ds_bus

);

wire [31:0] div_divisor_data;
wire        div_divisor_ready;
wire        div_divisor_valid;
wire [31:0] div_dividend_data;
wire        div_dividend_ready;
wire        div_dividend_valid;
wire [63:0] div_out_data;
wire        div_out_valid;

wire [31:0] divu_divisor_data;
wire        divu_divisor_ready;
wire        divu_divisor_valid;
wire [31:0] divu_dividend_data;
wire        divu_dividend_ready;
wire        divu_dividend_valid;
wire [63:0] divu_out_data;
wire        divu_out_valid;

reg [31:0] reg_hi;
reg [31:0] reg_lo;

reg         div_valid;
reg         divu_valid;

reg         es_valid;
wire        es_ready_go   ;

reg  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus_r;
wire [11:0] es_alu_op     ;
wire        es_load_op    ;
wire        es_src1_is_sa ;  
wire        es_src1_is_pc ;
wire        es_src2_is_imm;
wire        es_src2_ze_imm; 
wire        es_src2_is_8  ;
wire        es_gr_we      ;
wire        es_mem_we     ;
wire [ 4:0] es_dest       ;
wire [15:0] es_imm        ;
wire [31:0] es_rs_value   ;
wire [31:0] es_rt_value   ;
wire [31:0] es_pc         ;
wire        es_mult       ;
wire        es_multu      ;
wire        es_div        ;
wire        es_divu       ;

wire        es_mfhi       ;
wire        es_mflo       ;
wire        es_mthi       ;
wire        es_mtlo       ;

assign {es_mfhi        ,  //144
        es_mflo        ,  //143
        es_mthi        ,  //142
        es_mtlo        ,  //141
        es_divu        ,  //140
        es_div         ,  //139
        es_multu       ,  //138
        es_mult        ,  //137
        es_src2_ze_imm ,  //136:136
        es_alu_op      ,  //135:124
        es_load_op     ,  //123:123
        es_src1_is_sa  ,  //122:122
        es_src1_is_pc  ,  //121:121
        es_src2_is_imm ,  //120:120
        es_src2_is_8   ,  //119:119
        es_gr_we       ,  //118:118
        es_mem_we      ,  //117:117
        es_dest        ,  //116:112
        es_imm         ,  //111:96
        es_rs_value    ,  //95 :64
        es_rt_value    ,  //63 :32
        es_pc             //31 :0
       } = ds_to_es_bus_r;

wire [31:0] es_alu_src1   ;
wire [31:0] es_alu_src2   ;
wire [31:0] es_alu_result ;

wire        es_res_from_mem;
wire [31:0] es_to_ms_result;

wire [63:0] mult_result;
wire [63:0] multu_result;
wire [63:0] div_result;
wire [63:0] divu_result;

reg         div_not_in;
reg         divu_not_in;

assign multu_result = es_rs_value * es_rt_value;
assign mult_result = $signed(es_rs_value) * $signed(es_rt_value);
assign div_result = div_out_data;
assign divu_result = divu_out_data;

assign div_divisor_data = es_rt_value;
assign div_dividend_data = es_rs_value;
assign divu_divisor_data = es_rt_value;
assign divu_dividend_data = es_rs_value;

assign div_divisor_valid = div_valid;
assign div_dividend_valid= div_valid;
assign divu_divisor_valid = divu_valid;
assign divu_dividend_valid= divu_valid;

assign es_res_from_mem = es_load_op;
assign es_to_ms_bus = {es_res_from_mem,  //70:70
                       es_gr_we       ,  //69:69
                       es_dest        ,  //68:64
                       es_to_ms_result  ,  //63:32
                       es_pc             //31:0
                      };

assign es_ready_go    = !es_valid || (!(es_div && !div_out_valid) && !(es_divu && !divu_out_valid));
assign es_allowin     = !es_valid || es_ready_go && ms_allowin;
assign es_to_ms_valid =  es_valid && es_ready_go;
always @(posedge clk) begin
    if (reset) begin
        es_valid <= 1'b0;
    end
    else if (es_allowin) begin
        es_valid <= ds_to_es_valid;
    end

    if (ds_to_es_valid && es_allowin) begin
        ds_to_es_bus_r <= ds_to_es_bus;
    end
    
    if(es_mult) begin
        {reg_hi, reg_lo} <= mult_result;
    end
    else if(es_multu) begin
        {reg_hi, reg_lo} <= multu_result;
    end
    else if(es_div && div_out_valid) begin
        {reg_lo, reg_hi} <= div_result;
    end
    else if(es_divu && divu_out_valid) begin
        {reg_lo, reg_hi} <= divu_result;
    end
    
    if(es_mthi) begin
        reg_hi <= es_rs_value;
    end
    
    if(es_mtlo) begin
        reg_lo <= es_rs_value;
    end
    
    if(es_div && div_not_in)  begin
        div_valid <= 1'b1;
    end
    else if(div_divisor_ready && div_dividend_ready) begin
        div_valid <= 1'b0;
    end
    
    if(es_divu && divu_not_in) begin
        divu_valid <= 1'b1;
    end
    else if(divu_divisor_ready && divu_dividend_ready) begin
        divu_valid <= 1'b0;
    end
    
    if(reset || (es_div && div_out_valid)) begin
        div_not_in <= 1'b1;
    end
    else if(es_div && div_not_in) begin
        div_not_in <= 1'b0;
    end

    
    if(reset || (es_divu && divu_out_valid)) begin
        divu_not_in <= 1'b1;
    end
    else if(es_divu && divu_not_in) begin
        divu_not_in <= 1'b0;
    end
end

assign es_alu_src1 = es_src1_is_sa  ? {27'b0, es_imm[10:6]} : 
                     es_src1_is_pc  ? es_pc[31:0] :
                                      es_rs_value;
assign es_alu_src2 = es_src2_ze_imm ? {16'b0, es_imm[15:0]} :
                     es_src2_is_imm ? {{16{es_imm[15]}}, es_imm[15:0]} : 
                     es_src2_is_8   ? 32'd8 :
                                      es_rt_value;

alu u_alu(
    .alu_op     (es_alu_op    ),
    .alu_src1   (es_alu_src1  ),
    .alu_src2   (es_alu_src2  ),
    .alu_result (es_alu_result)
    );
    
div_mod my_div(
    .s_axis_divisor_tdata(div_divisor_data),
    .s_axis_divisor_tready(div_divisor_ready),
    .s_axis_divisor_tvalid(div_divisor_valid),
    .s_axis_dividend_tdata(div_dividend_data),
    .s_axis_dividend_tready(div_dividend_ready),
    .s_axis_dividend_tvalid(div_dividend_valid),
    .aclk(clk),
    .m_axis_dout_tdata(div_out_data),
    .m_axis_dout_tvalid(div_out_valid)
    );

divu_mod my_divu(
    .s_axis_divisor_tdata(divu_divisor_data),
    .s_axis_divisor_tready(divu_divisor_ready),
    .s_axis_divisor_tvalid(divu_divisor_valid),
    .s_axis_dividend_tdata(divu_dividend_data),
    .s_axis_dividend_tready(divu_dividend_ready),
    .s_axis_dividend_tvalid(divu_dividend_valid),
    .aclk(clk),
    .m_axis_dout_tdata(divu_out_data),
    .m_axis_dout_tvalid(divu_out_valid)
    );
    

assign data_sram_en    = 1'b1;
assign data_sram_wen   = es_mem_we&&es_valid ? 4'hf : 4'h0;
assign data_sram_addr  = es_alu_result;
assign data_sram_wdata = es_rt_value;

assign es_to_ms_result = es_mfhi ? reg_hi :
                         es_mflo ? reg_lo :
                         es_alu_result;

assign stuck_es_to_ds_bus = (!es_valid || !es_gr_we) ? 38'b0 : {es_load_op, es_dest, es_alu_result};
endmodule
