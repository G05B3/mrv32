//==============================================================================
// Module: mrv32_alu v1.0
//------------------------------------------------------------------------------
// Description:
//   RV32I arithmetic logic unit.
//
// Supported Operations:
//   - ADD / SUB
//   - AND / OR / XOR
//   - SLT / SLTU
//   - Shift left/right (logical and arithmetic)
//
// Notes:
//   Operation selected via alu_op control from decode stage.
//   ALU result is used for arithmetic, branch comparison, and address
//   generation.
//
// Author: Martim Bento
// Date  : 28/02/2026
//==============================================================================

module mrv32_alu (
    input  logic [31:0] op1,
    input  logic [31:0] op2,
    input  logic [3:0]  aluop,
    output logic [31:0] result
);
  import mrv32_pkg::*;

  wire [4:0] shamt = op2[4:0];  // extract shift amount outside always block

  always_comb begin
    case (aluop)
      ALU_ADD: result = op1 + op2;
      ALU_SUB: result = op1 - op2;
      ALU_AND: result = op1 & op2;
      ALU_OR:  result = op1 | op2;
      ALU_XOR: result = op1 ^ op2;
      ALU_SLL:  result = op1 << shamt;
      ALU_SRL:  result = op1 >> shamt;
      ALU_SRA:  result = 32'($signed(op1) >>> shamt);
      ALU_SLT:  result = {31'd0, $signed(op1) < $signed(op2)};
      ALU_SLTU: result = {31'd0, op1 < op2};
      default: result = 32'd0; // Default case for unsupported operations
    endcase
  end

endmodule