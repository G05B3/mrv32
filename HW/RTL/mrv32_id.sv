/*
 * MRV32 Instruction Decoder (RV32I Subset)
 *
 * Decodes a 32-bit RISC-V instruction and produces:
 *   - Register specifiers: rs1, rs2, rd
 *   - ALU control: aluop + alusrc (selects imm vs rs2 as operand 2)
 *   - Memory control: mem_ren, mem_wen, mem_wstrb (byte enables)
 *   - Writeback control: reg_wen
 *   - Instruction class flags: is_lui, is_jal
 *   - A sign-extended immediate value (imm) via the imm_gen module
 *   - unsupported_instr flag for illegal/unsupported encodings
 *
 * Currently supported instructions (bring-up subset):
 *   - LUI
 *   - OP-IMM: ADDI
 *   - OP: ADD, SUB
 *   - STORE: SW
 *   - JAL
 *
 * Notes:
 * - This module is purely combinational (no internal state).
 * - x0 handling (write suppression) is performed by reg_wen gating (rd != 0);
 *   the register file must still enforce x0=0 for safety.
 * - Immediate selection (imm_sel) is internal; imm_gen performs format
 *   extraction and sign-extension.
 * - For unsupported instructions, unsupported_instr is asserted and all other
 *   control outputs remain in their safe default (inactive) states.
 */

module instr_decode(
    input  logic [31:0] instr,
    output logic [4:0]  rs1_addr,
    output logic [4:0]  rs2_addr,
    output logic [4:0]  rd_addr,

    output logic [3:0]  aluop,
    output logic        alusrc,      // 1 => use immediate for op2
    output logic        mem_ren,
    output logic        mem_wen,
    output logic [3:0]  mem_wstrb,
    output logic        reg_wen,
    output logic [2:0]  f3,

    output logic        is_lui,
    output logic        is_jal,

    output logic [31:0] imm,
    output logic        unsupported_instr
);
  import mrv32_pkg::*;

  logic [6:0] opcode, funct7;
  logic [2:0] funct3;
  logic [2:0] imm_sel;

    assign opcode   = instr[6:0];
    assign rd_addr  = instr[11:7];
    assign funct3   = instr[14:12];
    assign rs1_addr = instr[19:15];
    assign rs2_addr = instr[24:20];
    assign funct7   = instr[31:25];

    assign f3       = instr[14:12];

  always_comb begin

    // Defaults
    aluop            = ALU_ADD;
    alusrc           = 1'b0;
    imm_sel          = IMM_I;
    mem_ren          = 1'b0;
    mem_wen          = 1'b0;
    mem_wstrb        = 4'b0000;
    reg_wen          = 1'b0;
    is_lui           = 1'b0;
    is_jal           = 1'b0;
    unsupported_instr = 1'b0;

    case (opcode)
        OPCODE_LUI: begin // LUI
            is_lui  = 1'b1;
            reg_wen = (rd_addr != 5'd0);
            imm_sel = IMM_U;
        end

        OPCODE_ITYPE: begin // I-TYPE (IMM OPs)
            alusrc = 1'b1;
            reg_wen = (rd_addr != 5'd0);
            imm_sel = IMM_I;
            case (funct3)
            3'b000: aluop = ALU_ADD; // ADDI
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
            default: begin
                reg_wen = 1'b0;
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
            3'b010: mem_wstrb = WSTRB_W; // SW
            default: begin
                mem_wen = 1'b0;
                unsupported_instr = 1'b1;
            end
            endcase
        end

        OPCODE_JAL: begin // JAL
            is_jal  = 1'b1;
            imm_sel = IMM_J;
            reg_wen = (rd_addr != 5'd0);
        end

        default: begin
            unsupported_instr = 1'b1;
        end
    endcase
  end

  // Immediate generator
  imm_gen immgen (
    .instr(instr),
    .imm_sel(imm_sel),
    .imm(imm)
  );

endmodule