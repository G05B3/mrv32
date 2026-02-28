//==============================================================================
// Module: mrv32_decode v1.0
//------------------------------------------------------------------------------
// Description:
//   Instruction decode stage for RV32I subset.
//
// Responsibilities:
//   - Decode opcode/funct3/funct7
//   - Generate ALU control signals
//   - Select immediate format
//   - Generate LSU control signals
//   - Identify branch and jump instructions
//
// Output:
//   - Control signals for EX, MEM, WB
//   - Immediate value
//   - Register file addresses
//
// Notes:
//   Designed to be compatible with future fully pipelined operation.
//
// Author: Martim Bento
// Date  : 28/02/2026
//==============================================================================

module mrv32_decode(
    input  logic [31:0] instr,
    output logic [4:0]  rs1_addr,
    output logic [4:0]  rs2_addr,
    output logic [4:0]  rd_addr,

    output logic [3:0]  aluop,
    output logic        alusrc,      // 1 => use immediate for op2
    output logic        mem_ren,
    output logic        mem_wen,
    output logic [3:0]  mem_wstrb,
    output logic        load_unsigned,
    output logic        reg_wen,

    output logic        is_lui,
    output logic        is_auipc,
    output logic        is_jalr,
    output logic [1:0]  br_sel,

    output logic [31:0] imm,
    output logic        unsupported_instr
);
  import mrv32_pkg::*;

  logic [6:0] opcode, funct7;
  logic [2:0] funct3;
  logic [2:0] imm_sel;
  logic [1:0] funct3_upper;
  logic funct3_lower;

    assign opcode   = instr[6:0];
    assign rd_addr  = instr[11:7];
    assign funct3   = instr[14:12];
    assign rs1_addr = instr[19:15];
    assign rs2_addr = instr[24:20];
    assign funct7   = instr[31:25];
    assign funct3_upper = funct3[2:1];
    assign funct3_lower = funct3[0];

  always_comb begin

    // Defaults
    aluop            = ALU_ADD;
    alusrc           = 1'b0;
    imm_sel          = IMM_I;
    mem_ren          = 1'b0;
    mem_wen          = 1'b0;
    mem_wstrb        = 4'b0000;
    load_unsigned    = 1'b0;
    reg_wen          = 1'b0;
    is_lui           = 1'b0;
    is_auipc         = 1'b0;
    is_jalr          = 1'b0;
    br_sel           = BR_NONE;
    unsupported_instr = 1'b0;

    case (opcode)
        OPCODE_LUI: begin // LUI
            is_lui  = 1'b1;
            reg_wen = (rd_addr != 5'd0);
            imm_sel = IMM_U;
        end

        OPCODE_AUIPC: begin // AUIPC
            is_auipc = 1'b1;
            reg_wen  = (rd_addr != 5'd0);
            imm_sel  = IMM_U;
        end

        OPCODE_ITYPE: begin // I-TYPE (IMM OPs)
            alusrc = 1'b1;
            reg_wen = (rd_addr != 5'd0);
            imm_sel = IMM_I;
            case (funct3)
            3'b000: aluop = ALU_ADD; // ADDI
            3'b001: aluop = ALU_SLL; // SLLI
            3'b010: aluop = ALU_SLT; // SLTI
            3'b011: aluop = ALU_SLTU; // SLTIU
            3'b100: aluop = ALU_XOR; // XORI
            3'b101: begin
                if (funct7 == 7'b0000000) aluop = ALU_SRL; // SRLI
                else if (funct7 == 7'b0100000) aluop = ALU_SRA; // SRAI
                else // Unsupported Instruction
                begin
                    reg_wen = 1'b0;
                    unsupported_instr = 1'b1;               
                end
            end
            3'b110: aluop = ALU_OR; // ORI
            3'b111: aluop = ALU_AND; // ANDI
            default: begin
                reg_wen = 1'b0;
                unsupported_instr = 1'b1;
            end
            endcase
        end

        OPCODE_RTYPE: begin // R-TYPE
            reg_wen = (rd_addr != 5'd0);
            case (funct3)
            3'b000: begin
                if (funct7 == 7'b0000000) aluop = ALU_ADD;
                else if (funct7 == 7'b0100000) aluop = ALU_SUB;
                else begin
                reg_wen = 1'b0;
                unsupported_instr = 1'b1;
                end
            end
            3'b001: aluop = ALU_SLL; // SLL
            3'b010: aluop = ALU_SLT; // SLT
            3'b011: aluop = ALU_SLTU; // SLTU
            3'b100: aluop = ALU_XOR; // XOR
            3'b101: begin
                if (funct7 == 7'b0000000) aluop = ALU_SRL; // SRL
                else if (funct7 == 7'b0100000) aluop = ALU_SRA; // SRA
                else // Unsupported Instruction
                begin
                    reg_wen = 1'b0;
                    unsupported_instr = 1'b1;               
                end
            end
            3'b110: aluop = ALU_OR; // OR
            3'b111: aluop = ALU_AND; // AND
            default: begin
                reg_wen = 1'b0;
                unsupported_instr = 1'b1;
            end
            endcase
        end

        OPCODE_LOAD: begin /// I-TYPE (LOAD)
           alusrc = 1'b1;
           mem_ren = 1'b1;
           reg_wen = (rd_addr != 5'd0);
           imm_sel = IMM_I;
           aluop = ALU_ADD;
           case (funct3)
           3'b000: mem_wstrb = WSTRB_B; // LB
           3'b001: mem_wstrb = WSTRB_H; // LH
           3'b010: mem_wstrb = WSTRB_W; // LW
           3'b100: begin mem_wstrb = WSTRB_B; load_unsigned = 1'b1; end // LBU
           3'b101: begin mem_wstrb = WSTRB_H; load_unsigned = 1'b1; end// LHU
           default: begin
            mem_ren = 1'b0;
            unsupported_instr = 1'b1;
           end
           endcase 
        end

        OPCODE_STYPE: begin // S-TYPE (STORE)
            alusrc = 1'b1;
            mem_wen = 1'b1;
            imm_sel = IMM_S;
            aluop = ALU_ADD;
            case (funct3)
            3'b000: mem_wstrb = WSTRB_B; // SB
            3'b001: mem_wstrb = WSTRB_H; // SH
            3'b010: mem_wstrb = WSTRB_W; // SW
            default: begin
                mem_wen = 1'b0;
                unsupported_instr = 1'b1;
            end
            endcase
        end

        OPCODE_BTYPE: begin // B-TYPE (BRANCHES)
            alusrc = 1'b0;
            imm_sel = IMM_B;
            br_sel = (funct3_lower) ? BR_NQLT : BR_EQLT; // BNE/BEQ, BLT/BGE, BLTU/BGEU
            case(funct3_upper)
            2'b00: aluop = ALU_SUB;  // BEQ  (000) or BNE  (001)
            2'b10: aluop = ALU_SLT;  // BLT  (100) or BGE  (101)
            2'b11: aluop = ALU_SLTU; // BLTU (110) or BGEU (111)
            default: begin
                unsupported_instr = 1'b0;
            end
            endcase
        end

        OPCODE_JAL: begin // JAL
            br_sel = BR_ALWAYS;
            imm_sel = IMM_J;
            reg_wen = (rd_addr != 5'd0);
        end

        OPCODE_JALR: begin // JALR
            is_jalr = 1'b1;
            br_sel = BR_ALWAYS;
            imm_sel = IMM_I;
            reg_wen = (rd_addr != 5'd0);
            alusrc = 1'b1; // op with IMM
            aluop = ALU_ADD; // target addr = rs1 + imm
        end

        default: begin
            unsupported_instr = 1'b1;
        end
    endcase
  end

  // Immediate generator
  mrv32_imm_gen immgen (
    .instr(instr),
    .imm_sel(imm_sel),
    .imm(imm)
  );

endmodule