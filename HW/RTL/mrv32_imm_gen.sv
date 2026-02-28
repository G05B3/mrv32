//==============================================================================
// Module: mrv32_imm_gen v1.0
//------------------------------------------------------------------------------
// Description:
//   Immediate value generator for RV32I instructions.
//
// Supported Formats:
//   - I-type
//   - S-type
//   - B-type
//   - U-type
//   - J-type
//
// Notes:
//   Immediate format selected via imm_sel control from decode stage.
//   Outputs properly sign-extended 32-bit immediate.
//
// Author: Martim Bento
// Date  : 28/02/2026
//==============================================================================

module mrv32_imm_gen (
    input  logic [31:0] instr,
    input  logic [2:0]  imm_sel,
    output logic [31:0] imm
);
  import mrv32_pkg::*;

  // Precompute all immediates (pure wiring)
  wire [31:0] imm_i = {{20{instr[31]}}, instr[31:20]};
  wire [31:0] imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  wire [31:0] imm_b = {{19{instr[31]}}, instr[31], instr[7],
                       instr[30:25], instr[11:8], 1'b0};
  wire [31:0] imm_u = {instr[31:12], 12'b0};
  wire [31:0] imm_j = {{11{instr[31]}}, instr[31], instr[19:12],
                       instr[20], instr[30:21], 1'b0};

  // Select immediate
  assign imm =
      (imm_sel == IMM_I) ? imm_i :
      (imm_sel == IMM_S) ? imm_s :
      (imm_sel == IMM_B) ? imm_b :
      (imm_sel == IMM_U) ? imm_u :
      (imm_sel == IMM_J) ? imm_j :
                           32'd0;
endmodule