/*
 * MRV32 Arithmetic Logic Unit (ALU)
 *
 * Combinational ALU used by the MRV32 RV32I core.
 *
 * Inputs:
 *   - op1, op2 : 32-bit operands
 *   - aluop    : operation selector (encoding defined in mrv32_pkg)
 *
 * Output:
 *   - result   : 32-bit operation result
 *
 * Notes:
 * - Purely combinational (no internal state).
 * - ALU operation encodings (ALU_ADD, ALU_SUB, etc.) are defined in mrv32_pkg
 *   and must be kept consistent with the decoder and execute stage.
 * - Unsupported operations return zero by default.
 */

module alu (
    input  logic [31:0] op1,
    input  logic [31:0] op2,
    input  logic [3:0]  aluop,
    output logic [31:0] result
);
  import mrv32_pkg::*;

  always_comb begin
    case (aluop)
      ALU_ADD: result = op1 + op2;
      ALU_SUB: result = op1 - op2;
      ALU_AND: result = op1 & op2;
      ALU_OR:  result = op1 | op2;
      ALU_XOR: result = op1 ^ op2;
      default: result = 32'd0; // Default case for unsupported operations
    endcase
  end

endmodule