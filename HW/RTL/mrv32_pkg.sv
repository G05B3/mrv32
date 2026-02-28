/*
 * MRV32 Core Package
 *
 * Shared constants, encodings, and architectural parameters for the MRV32
 * RV32I-compatible CPU core.
 *
 * This package defines:
 *   - RV32I instruction opcode encodings (instr[6:0])
 *   - ALU operation encodings used by decode and execute stages
 *   - Immediate format selectors for the immediate generator
 *   - Write strobe (byte enable) constants for memory stores
 *   - Common architectural constants (XLEN, register address width)
 *
 * The purpose of this package is to centralize all cross-module encodings
 * and architectural constants to ensure consistency between RTL blocks
 * such as:
 *   - instruction decode
 *   - ALU / execute stage
 *   - immediate generator
 *   - load/store unit
 *
 * All RTL modules that rely on these definitions should import this package:
 *
 *   import mrv32_pkg::*;
 *
 * This package contains no logic and is fully synthesizable.
 */

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
