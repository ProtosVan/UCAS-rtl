`include "mycpu.h"

module id_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          es_allowin    ,
    output                         ds_allowin    ,
    //from fs
    input                          fs_to_ds_valid,
    input  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus  ,
    //to es
    output                         ds_to_es_valid,
    output [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to fs
    output [`BR_BUS_WD       -1:0] br_bus        ,
    //to rf: for write back
    input  [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus ,

    //handle stuck
    input [37:0] stuck_es_to_ds_bus,
    input [36:0] stuck_ms_to_ds_bus,
    input [36:0] stuck_ws_to_ds_bus
);

wire damn_it_stuck;

wire [ 4:0] stuck_es_addr;
wire [ 4:0] stuck_ms_addr;
wire [ 4:0] stuck_ws_addr;

wire [31:0] stuck_es_data;
wire [31:0] stuck_ms_data;
wire [31:0] stuck_ws_data;

wire [ 2:0] rs_upup;
wire [ 2:0] rt_upup;

assign {stuck_es_addr,// [36:32]
        stuck_es_data // [31: 0]
 } = stuck_es_to_ds_bus;

assign {stuck_ms_addr,// [36:32]
        stuck_ms_data // [31: 0]
} = stuck_ms_to_ds_bus;

assign {stuck_ws_addr,// [36:32]
        stuck_ws_data // [31: 0]
} = stuck_ws_to_ds_bus;

reg         ds_valid   ;
wire        ds_ready_go;


wire        ds_rs_rt;
wire        ds_rs;
wire        ds_rt;


wire [31                 :0] fs_pc;
reg  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus_r;
assign fs_pc = fs_to_ds_bus[31:0];

wire [31:0] ds_inst;
wire [31:0] ds_pc  ;
assign {ds_inst,
        ds_pc  } = fs_to_ds_bus_r;


wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;
assign {rf_we   ,  //37:37
        rf_waddr,  //36:32
        rf_wdata   //31:0
       } = ws_to_rf_bus;

wire        br_taken;
wire [31:0] br_target;

wire [11:0] alu_op;
wire        load_op;
wire [2 :0] load_inst;
wire        src1_is_sa;
wire        src1_is_pc;
wire        src2_is_imm;
wire        src2_ze_imm;
wire        src2_is_8;
wire        res_from_mem;
wire        gr_we;
wire        mem_we;
wire [ 4:0] dest;
wire [15:0] imm;
wire [31:0] rs_value;
wire [31:0] rt_value;

wire [ 5:0] op;
wire [ 4:0] rs;
wire [ 4:0] rt;
wire [ 4:0] rd;
wire [ 4:0] sa;
wire [ 5:0] func;
wire [25:0] jidx;
wire [63:0] op_d;
wire [31:0] rs_d;
wire [31:0] rt_d;
wire [31:0] rd_d;
wire [31:0] sa_d;
wire [63:0] func_d;

wire        inst_add;
wire        inst_addu;
wire        inst_sub;
wire        inst_subu;
wire        inst_slt;
wire        inst_sltu;
wire        inst_and;
wire        inst_andi;
wire        inst_or;
wire        inst_ori;
wire        inst_xor;
wire        inst_xori;
wire        inst_nor;
wire        inst_sll;
wire        inst_sllv;
wire        inst_srl;
wire        inst_srlv;
wire        inst_sra;
wire        inst_srav;
wire        inst_addi;
wire        inst_addiu;
wire        inst_slti;
wire        inst_sltiu;
wire        inst_lui;
wire        inst_lb;
wire        inst_lh;
wire        inst_lbu;
wire        inst_lhu;
wire        inst_lwl;
wire        inst_lwr;
wire        inst_lw;
wire        inst_sw;
wire        inst_sb;
wire        inst_sh;
wire        inst_swl;
wire        inst_swr;
wire        inst_beq;
wire        inst_bne;
wire        inst_j;
wire        inst_jal;
wire        inst_jalr;
wire        inst_jr;
wire        inst_mult;
wire        inst_multu;
wire        inst_div;
wire        inst_divu;
wire        inst_mfhi;
wire        inst_mflo;
wire        inst_mthi;
wire        inst_mtlo;
wire        inst_bgez;
wire        inst_bgezal;
wire        inst_bgtz;
wire        inst_blez;
wire        inst_bltz;
wire        inst_bltzal;

wire        dst_is_r31;  
wire        dst_is_rt;   

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;

wire [31:0] load_adjs;
wire [2 :0] writ_inst;

wire        rs_eq_rt;
wire        rs_gr_ze;
wire        rs_sm_ze;

assign br_bus       = {br_taken,br_target};

assign ds_to_es_bus = {writ_inst   ,  //182:180
                       load_adjs   ,  //179:148
                       load_inst   ,  //147:145
                       inst_mfhi   ,  //144
                       inst_mflo   ,  //143
                       inst_mthi   ,  //142
                       inst_mtlo   ,  //141
                       inst_divu   ,  //140
                       inst_div    ,  //139
                       inst_multu  ,  //138
                       inst_mult   ,  //137
                       src2_ze_imm ,  //136:136
                       alu_op      ,  //135:124
                       load_op     ,  //123:123
                       src1_is_sa  ,  //122:122
                       src1_is_pc  ,  //121:121
                       src2_is_imm ,  //120:120
                       src2_is_8   ,  //119:119
                       gr_we       ,  //118:118
                       mem_we      ,  //117:117
                       dest        ,  //116:112
                       imm         ,  //111:96
                       rs_value    ,  //95 :64
                       rt_value    ,  //63 :32
                       ds_pc          //31 :0
                      };

assign ds_rs_rt     =   inst_addu|| inst_sub || inst_subu||inst_slt||
                        inst_sltu||inst_and||inst_or||
                        inst_xor||inst_nor || inst_bne || inst_sw || inst_beq ||
                        inst_add || inst_sllv || inst_srlv || inst_srav ||
                        inst_mult || inst_multu || inst_div || inst_divu ||
                        inst_lwl  || inst_lwr || inst_sb || inst_sh ||
                        inst_swl || inst_swr;
assign ds_rs        =   inst_jr || inst_addi || inst_addiu || inst_lw ||ds_rs_rt ||
                        inst_slti || inst_sltiu ||
                        inst_andi || inst_ori || inst_xori ||
                        inst_mtlo || inst_mthi ||
                        inst_bgez || inst_bgtz || inst_blez || inst_bltz ||
                        inst_bgezal || inst_bltzal ||
                        inst_jalr || inst_lb ||inst_lh || inst_lbu || inst_lhu;
assign ds_rt        =   inst_sll || inst_srl || inst_sra  || ds_rs_rt;                        




assign ds_ready_go  =   !ds_valid || !damn_it_stuck;

assign damn_it_stuck =  stuck_es_to_ds_bus[37]
                        && (rs != 5'b0)
                        && (rs == stuck_es_addr)
                        ||stuck_es_to_ds_bus[37]
                        && (rt != 5'b0)
                        && (rt == stuck_es_addr);
                          /*|| rs == stuck_ms_addr
                          || rs == stuck_ws_addr)
                     || (ds_rt
                      && (rt != 5'b0)
                      && (rt == stuck_es_addr
                           || rt == stuck_ms_addr
                           || rt == stuck_ws_addr))*/

assign rs_upup[2] = (ds_rs) 
                      && (rs != 5'b0)
                      && (rs == stuck_es_addr);
assign rs_upup[1] = (ds_rs) 
                      && (rs != 5'b0)
                      && (rs == stuck_ms_addr);
assign rs_upup[0] = (ds_rs) 
                      && (rs != 5'b0)
                      && (rs == stuck_ws_addr);
                      
assign rt_upup[2] = (ds_rt) 
                      && (rt != 5'b0)
                      && (rt == stuck_es_addr);
assign rt_upup[1] = (ds_rt) 
                      && (rt != 5'b0)
                      && (rt == stuck_ms_addr);
assign rt_upup[0] = (ds_rt) 
                      && (rt != 5'b0)
                      && (rt == stuck_ws_addr);


assign ds_allowin     = !ds_valid || ds_ready_go && es_allowin;
assign ds_to_es_valid = ds_valid && ds_ready_go;

always @(posedge clk) begin

    if (reset) begin
        ds_valid <= 1'b0;
    end
    else if (ds_allowin) begin
        ds_valid <= fs_to_ds_valid;
    end
    
    if (fs_to_ds_valid && ds_allowin) begin
        fs_to_ds_bus_r <= fs_to_ds_bus;
    end
end

assign op   = ds_inst[31:26];
assign rs   = ds_inst[25:21];
assign rt   = ds_inst[20:16];
assign rd   = ds_inst[15:11];
assign sa   = ds_inst[10: 6];
assign func = ds_inst[ 5: 0];
assign imm  = ds_inst[15: 0];
assign jidx = ds_inst[25: 0];

decoder_6_64 u_dec0(.in(op  ), .out(op_d  ));
decoder_6_64 u_dec1(.in(func), .out(func_d));
decoder_5_32 u_dec2(.in(rs  ), .out(rs_d  ));
decoder_5_32 u_dec3(.in(rt  ), .out(rt_d  ));
decoder_5_32 u_dec4(.in(rd  ), .out(rd_d  ));
decoder_5_32 u_dec5(.in(sa  ), .out(sa_d  ));

assign inst_sll    = op_d[6'h00] & func_d[6'h00] & rs_d[5'h00];
assign inst_srl    = op_d[6'h00] & func_d[6'h02] & rs_d[5'h00];
assign inst_sra    = op_d[6'h00] & func_d[6'h03] & rs_d[5'h00];
assign inst_sllv   = op_d[6'h00] & func_d[6'h04] & sa_d[5'h00];
assign inst_srlv   = op_d[6'h00] & func_d[6'h06] & sa_d[5'h00];
assign inst_srav   = op_d[6'h00] & func_d[6'h07] & sa_d[5'h00];
assign inst_jr     = op_d[6'h00] & func_d[6'h08] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];
assign inst_jalr   = op_d[6'h00] & func_d[6'h09] & rt_d[5'h00] & sa_d[5'h00];
assign inst_mfhi   = op_d[6'h00] & func_d[6'h10] & rt_d[5'h00] & rs_d[5'h00] & sa_d[5'h00];
assign inst_mthi   = op_d[6'h00] & func_d[6'h11] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];
assign inst_mflo   = op_d[6'h00] & func_d[6'h12] & rt_d[5'h00] & rs_d[5'h00] & sa_d[5'h00];
assign inst_mtlo   = op_d[6'h00] & func_d[6'h13] & rt_d[5'h00] & rd_d[5'h00] & sa_d[5'h00];
assign inst_mult   = op_d[6'h00] & func_d[6'h18] & sa_d[5'h00] & rd_d[5'h00];
assign inst_multu  = op_d[6'h00] & func_d[6'h19] & sa_d[5'h00] & rd_d[5'h00];
assign inst_div    = op_d[6'h00] & func_d[6'h1a] & sa_d[5'h00] & rd_d[5'h00];
assign inst_divu   = op_d[6'h00] & func_d[6'h1b] & sa_d[5'h00] & rd_d[5'h00];
assign inst_add    = op_d[6'h00] & func_d[6'h20] & sa_d[5'h00];
assign inst_addu   = op_d[6'h00] & func_d[6'h21] & sa_d[5'h00];
assign inst_sub    = op_d[6'h00] & func_d[6'h22] & sa_d[5'h00];
assign inst_subu   = op_d[6'h00] & func_d[6'h23] & sa_d[5'h00];
assign inst_and    = op_d[6'h00] & func_d[6'h24] & sa_d[5'h00];
assign inst_or     = op_d[6'h00] & func_d[6'h25] & sa_d[5'h00];
assign inst_xor    = op_d[6'h00] & func_d[6'h26] & sa_d[5'h00];
assign inst_nor    = op_d[6'h00] & func_d[6'h27] & sa_d[5'h00];
assign inst_slt    = op_d[6'h00] & func_d[6'h2a] & sa_d[5'h00];
assign inst_sltu   = op_d[6'h00] & func_d[6'h2b] & sa_d[5'h00];
assign inst_bltz   = op_d[6'h01] & rt_d[5'h00];
assign inst_bgez   = op_d[6'h01] & rt_d[5'h01];
assign inst_bltzal = op_d[6'h01] & rt_d[5'h10];
assign inst_bgezal = op_d[6'h01] & rt_d[5'h11];
assign inst_lui    = op_d[6'h0f] & rs_d[5'h00];
assign inst_j      = op_d[6'h02];
assign inst_jal    = op_d[6'h03];
assign inst_beq    = op_d[6'h04];
assign inst_bne    = op_d[6'h05];
assign inst_blez   = op_d[6'h06] & rt_d[5'h00];
assign inst_bgtz   = op_d[6'h07] & rt_d[5'h00];
assign inst_addi   = op_d[6'h08];
assign inst_addiu  = op_d[6'h09];
assign inst_slti   = op_d[6'h0a];
assign inst_sltiu  = op_d[6'h0b];
assign inst_andi   = op_d[6'h0c];
assign inst_ori    = op_d[6'h0d];
assign inst_xori   = op_d[6'h0e];
assign inst_lb     = op_d[6'h20];
assign inst_lh     = op_d[6'h21];
assign inst_lwl    = op_d[6'h22];
assign inst_lw     = op_d[6'h23];
assign inst_lbu    = op_d[6'h24];
assign inst_lhu    = op_d[6'h25];
assign inst_lwr    = op_d[6'h26];
assign inst_sb     = op_d[6'h28];
assign inst_sh     = op_d[6'h29];
assign inst_swl    = op_d[6'h2a];
assign inst_sw     = op_d[6'h2b];
assign inst_swr    = op_d[6'h2e];



assign alu_op[ 0] = inst_add | inst_addu | inst_addi | inst_addiu | inst_lw | inst_sw | inst_jal |
                    inst_bltzal | inst_bgezal | inst_jalr |
                    inst_lb | inst_lh | inst_lbu | inst_lhu |
                    inst_lwl | inst_lwr | inst_sb | inst_sh |
                    inst_swl | inst_swr;
assign alu_op[ 1] = inst_subu| inst_sub;
assign alu_op[ 2] = inst_slt | inst_slti;
assign alu_op[ 3] = inst_sltu| inst_sltiu;
assign alu_op[ 4] = inst_and | inst_andi;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or  | inst_ori;
assign alu_op[ 7] = inst_xor | inst_xori;
assign alu_op[ 8] = inst_sll | inst_sllv;
assign alu_op[ 9] = inst_srl | inst_srlv;
assign alu_op[10] = inst_sra | inst_srav;
assign alu_op[11] = inst_lui;
assign load_op    = inst_lw | inst_lb | inst_lh | inst_lbu |inst_lhu |
                    inst_lwl| inst_lwr;
assign writ_inst  = inst_sw ? 3'b001 :
                    inst_sb ? 3'b010 :
                    inst_sh ? 3'b011 :
                    inst_swl? 3'b100 :
                    inst_swr? 3'b101 :
                    3'b000;
assign load_inst  = inst_lw ? 3'b000 : 
                    inst_lb ? 3'b001 :
                    inst_lh ? 3'b010 :
                    inst_lbu? 3'b011 :
                    inst_lhu? 3'b100 :
                    inst_lwl? 3'b101 :
                    inst_lwr? 3'b110 :
                    3'b0;
assign src1_is_sa   = inst_sll   | inst_srl | inst_sra;
assign src1_is_pc   = inst_jal | inst_bltzal | inst_bgezal | inst_jalr;
assign src2_is_imm  = inst_addi | inst_addiu | inst_lui | inst_lw | inst_sw | inst_slti | inst_sltiu |
                      inst_lb | inst_lh | inst_lbu | inst_lhu |
                      inst_lwl| inst_lwr | inst_sb | inst_sh |
                      inst_swl | inst_swr;
assign src2_ze_imm  = inst_andi | inst_ori | inst_xori;
assign src2_is_8    = inst_jal | inst_bltzal | inst_bgezal | inst_jalr;
assign res_from_mem = inst_lw  | inst_lb | inst_lh | inst_lbu | inst_lhu |
                      inst_lwl | inst_lwr;
assign dst_is_r31   = inst_jal | inst_bltzal | inst_bgezal;
assign dst_is_rt    = inst_addi | inst_addiu | inst_lui | inst_lw | inst_slti | inst_sltiu |
                      inst_andi | inst_ori | inst_xori | inst_lb | inst_lh |
                      inst_lbu  | inst_lhu | inst_lwl | inst_lwr;
assign gr_we        = ~inst_sw & ~inst_beq & ~inst_bne & ~inst_jr &
                      ~inst_mult & ~inst_multu & ~inst_div & ~inst_divu &
                      ~inst_mtlo & ~inst_mthi &
                      ~inst_bltz & ~inst_blez & ~inst_bgtz & ~inst_bgez &
                      ~inst_j & ~inst_sb &~inst_sh &
                      ~inst_swl&~inst_swr;
assign mem_we       = inst_sw & inst_sb &inst_sh &
                      inst_swl & inst_swr;

assign dest         = dst_is_r31 ? 5'd31 :
                      dst_is_rt  ? rt    : 
                                   rd;

assign rf_raddr1 = rs;
assign rf_raddr2 = rt;

regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

assign rs_value = rs_upup[2] ? stuck_es_data :
                  rs_upup[1] ? stuck_ms_data :
                  rs_upup[0] ? stuck_ws_data : rf_rdata1;
assign rt_value = rt_upup[2] ? stuck_es_data :
                  rt_upup[1] ? stuck_ms_data :
                  rt_upup[0] ? stuck_ws_data : rf_rdata2;

assign load_adjs = rt_value;
assign rs_eq_rt = (rs_value == rt_value);
assign rs_gr_ze = ($signed(rs_value) >  0);
assign rs_sm_ze = ($signed(rs_value) <  0);

assign br_taken = (   inst_beq  &&  rs_eq_rt
                   || inst_bne  && !rs_eq_rt
                   || inst_bgez && !rs_sm_ze
                   || inst_bgtz &&  rs_gr_ze
                   || inst_blez && !rs_gr_ze
                   || inst_bltz &&  rs_sm_ze
                   || inst_bltzal &&  rs_sm_ze
                   || inst_bgezal && !rs_sm_ze
                   || inst_j
                   || inst_jal
                   || inst_jr
                   || inst_jalr
                  ) && ds_valid;
assign br_target = (inst_beq  || inst_bne  ||
                    inst_bltz || inst_blez ||
                    inst_bgtz || inst_bgez ||
                    inst_bgezal|| inst_bltzal) ? (fs_pc + {{14{imm[15]}}, imm[15:0], 2'b0}) :
                   (inst_jr || inst_jalr)      ? rs_value :
                  /*inst_jal  || inst_j*/  {fs_pc[31:28], jidx[25:0], 2'b0};

endmodule
