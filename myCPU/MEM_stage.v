`include "mycpu.h"

module mem_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ws_allowin    ,
    output                         ms_allowin    ,
    //from es
    input                          es_to_ms_valid,
    input  [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    //to ws
    output                         ms_to_ws_valid,
    output [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus  ,
    //from data-sram
    input  [31                 :0] data_sram_rdata,

    //stuck
    output [36 :0] stuck_ms_to_ds_bus
);

reg         ms_valid;
wire        ms_ready_go;

reg [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus_r;
wire        ms_res_from_mem;
wire        ms_gr_we;
wire [ 4:0] ms_dest;
wire [31:0] ms_alu_result;
wire [31:0] ms_pc;
wire [2 :0] ms_load_inst;
wire [1 :0] ms_load_addr;
wire [31:0] ms_load_adjs;

assign {ms_load_adjs   , 
        ms_load_addr   ,  //75:74
        ms_load_inst   ,  //73:71
        ms_res_from_mem,  //70:70
        ms_gr_we       ,  //69:69
        ms_dest        ,  //68:64
        ms_alu_result  ,  //63:32
        ms_pc             //31:0
       } = es_to_ms_bus_r;

wire [31:0] mem_result;
wire [31:0] ms_final_result;

wire        ms_lw;
wire        ms_lb;
wire        ms_lh;
wire        ms_lbu;
wire        ms_lhu;
wire        ms_lwl;
wire        ms_lwr;

assign ms_lw = ms_res_from_mem & (ms_load_inst == 3'b000);
assign ms_lb = ms_res_from_mem & (ms_load_inst == 3'b001);
assign ms_lh = ms_res_from_mem & (ms_load_inst == 3'b010);
assign ms_lbu= ms_res_from_mem & (ms_load_inst == 3'b011);
assign ms_lhu= ms_res_from_mem & (ms_load_inst == 3'b100);
assign ms_lwl= ms_res_from_mem & (ms_load_inst == 3'b101);
assign ms_lwr= ms_res_from_mem & (ms_load_inst == 3'b110);

assign ms_to_ws_bus = {ms_gr_we       ,  //69:69
                       ms_dest        ,  //68:64
                       ms_final_result,  //63:32
                       ms_pc             //31:0
                      };

assign ms_ready_go    = 1'b1;
assign ms_allowin     = !ms_valid || ms_ready_go && ws_allowin;
assign ms_to_ws_valid = ms_valid && ms_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ms_valid <= 1'b0;
    end
    else if (ms_allowin) begin
        ms_valid <= es_to_ms_valid;
    end

    if (es_to_ms_valid && ms_allowin) begin
        es_to_ms_bus_r <= es_to_ms_bus;
    end
end

assign mem_result = ms_lw ? data_sram_rdata :
                    ms_lb && (ms_load_addr == 2'b00) ? {{25{data_sram_rdata[7 ]}}, data_sram_rdata[6 :0 ]} :
                    ms_lb && (ms_load_addr == 2'b01) ? {{25{data_sram_rdata[15]}}, data_sram_rdata[14:8 ]} :
                    ms_lb && (ms_load_addr == 2'b10) ? {{25{data_sram_rdata[23]}}, data_sram_rdata[22:16]} :
                    ms_lb && (ms_load_addr == 2'b11) ? {{25{data_sram_rdata[31]}}, data_sram_rdata[30:24]} :
                    ms_lh && (ms_load_addr == 2'b00) ? {{17{data_sram_rdata[15]}}, data_sram_rdata[14:0 ]} :
                    ms_lh && (ms_load_addr == 2'b10) ? {{17{data_sram_rdata[31]}}, data_sram_rdata[30:16]} :
                    ms_lbu&& (ms_load_addr == 2'b00) ? {24'b0                    , data_sram_rdata[7 :0 ]} :
                    ms_lbu&& (ms_load_addr == 2'b01) ? {24'b0                    , data_sram_rdata[15:8 ]} :
                    ms_lbu&& (ms_load_addr == 2'b10) ? {24'b0                    , data_sram_rdata[23:16]} :
                    ms_lbu&& (ms_load_addr == 2'b11) ? {24'b0                    , data_sram_rdata[31:24]} :
                    ms_lhu&& (ms_load_addr == 2'b00) ? {16'b0                    , data_sram_rdata[15:0 ]} :
                    ms_lhu&& (ms_load_addr == 2'b10) ? {16'b0                    , data_sram_rdata[31:16]} :
                    ms_lwl&& (ms_load_addr == 2'b00) ? {data_sram_rdata[7 :0 ], ms_load_adjs[23:0 ]} :
                    ms_lwl&& (ms_load_addr == 2'b01) ? {data_sram_rdata[15:0 ], ms_load_adjs[15:0 ]} :
                    ms_lwl&& (ms_load_addr == 2'b10) ? {data_sram_rdata[23:0 ], ms_load_adjs[7 :0 ]} :
                    ms_lwl&& (ms_load_addr == 2'b11) ? {data_sram_rdata[31:0 ]} :
                    ms_lwr&& (ms_load_addr == 2'b00) ? {data_sram_rdata[31:0 ]} :
                    ms_lwr&& (ms_load_addr == 2'b01) ? {ms_load_adjs[31:24], data_sram_rdata[31:8 ]} :
                    ms_lwr&& (ms_load_addr == 2'b10) ? {ms_load_adjs[31:16], data_sram_rdata[31:16]} :
                    ms_lwr&& (ms_load_addr == 2'b11) ? {ms_load_adjs[31:8 ], data_sram_rdata[31:24]} :
                    data_sram_rdata;

assign ms_final_result = ms_res_from_mem ? mem_result
                                         : ms_alu_result;

assign stuck_ms_to_ds_bus = (!ms_valid || !ms_gr_we) ? 37'b0 : {ms_dest, ms_final_result};

endmodule
