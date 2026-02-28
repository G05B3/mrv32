//==============================================================================
// Package: mrv32_pkg v1.0
//------------------------------------------------------------------------------
// Description:
//   Global definitions for the MRV32 core.
//
// Contents:
//   - Opcode encodings
//   - ALU operation encodings
//   - Immediate selectors
//   - LSU width encodings
//   - Architectural constants (XLEN, register count, etc.)
//
// Notes:
//   Centralizes ISA-related definitions to ensure consistency
//   across all modules.
//
// Author: Martim Bento
// Date  : 28/02/2026
//==============================================================================

package mrv32_pkg;

  // ----------------------------
  // RV32I opcodes (instr[6:0])
  // ----------------------------
  localparam logic [6:0] OPCODE_LUI    = 7'b0110111;
  localparam logic [6:0] OPCODE_AUIPC  = 7'b0010111;
  localparam logic [6:0] OPCODE_JAL    = 7'b1101111;
  localparam logic [6:0] OPCODE_JALR   = 7'b1100111;
  localparam logic [6:0] OPCODE_BTYPE  = 7'b1100011;
  localparam logic [6:0] OPCODE_LOAD   = 7'b0000011;
  localparam logic [6:0] OPCODE_STYPE  = 7'b0100011;
  localparam logic [6:0] OPCODE_ITYPE  = 7'b0010011;
  localparam logic [6:0] OPCODE_RTYPE  = 7'b0110011;
  localparam logic [6:0] OPCODE_SYSTEM = 7'b1110011; // ecall/ebreak/csrs (future)

  // ----------------------------
  // ALU op encoding
  // ----------------------------
  localparam logic [3:0]
      ALU_ADD  = 4'd0,
      ALU_SUB  = 4'd1,
      ALU_AND  = 4'd2,
      ALU_OR   = 4'd3,
      ALU_XOR  = 4'd4,
      ALU_SLL  = 4'd5,
      ALU_SRL  = 4'd6,
      ALU_SRA  = 4'd7,
      ALU_SLT  = 4'd8,
      ALU_SLTU = 4'd9;

  // ----------------------------
  // Immediate selector
  // ----------------------------
  localparam logic [2:0]
      IMM_I = 3'd0,
      IMM_S = 3'd1,
      IMM_B = 3'd2,
      IMM_U = 3'd3,
      IMM_J = 3'd4;

  localparam logic [1:0]
      BR_NONE   = 2'b00, // not a branch (default)
      BR_ALWAYS = 2'b01, // unconditional jump (JAL/JALR)
      BR_EQLT   = 2'b10, // branch (BEQ, BLT, BLTU)
      BR_NQLT   = 2'b11; // branch (BNE, BGE, BGEU)

  // ----------------------------
  // Write strobe helpers (byte enables)
  // ----------------------------
  localparam logic [3:0] WSTRB_NONE = 4'b0000;
  localparam logic [3:0] WSTRB_B    = 4'b0001; // store byte (lane select handled elsewhere)
  localparam logic [3:0] WSTRB_H    = 4'b0011; // store halfword (lane select handled elsewhere)
  localparam logic [3:0] WSTRB_W    = 4'b1111; // store word

  localparam MEM_BYTES  = 1024 * 1024;
  localparam ADDR_WIDTH = $clog2(MEM_BYTES);

  // ----------------------------
  // Common widths / constants
  // ----------------------------
  localparam int unsigned XLEN    = 32;
  localparam int unsigned REGADDR = 5;

endpackage : mrv32_pkg
