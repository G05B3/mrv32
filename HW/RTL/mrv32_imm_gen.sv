/*
 * MRV32 Immediate Generator (RV32I)
 *
 * Generates a sign-extended immediate value from a 32-bit RISC-V instruction,
 * based on the immediate format selected by the decoder.
 *
 * Supported immediate formats:
 *   - I-type : instr[31:20]
 *   - S-type : {instr[31:25], instr[11:7]}
 *   - B-type : {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}
 *   - U-type : instr[31:12] << 12
 *   - J-type : {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}
 *
 * All immediates are sign-extended to 32 bits as defined by the RV32I ISA.
 *
 * Notes:
 * - This module is purely combinational.
 * - No instruction decoding is performed here; the decoder is responsible
 *   for selecting the correct immediate format via imm_sel.
 * - The module is reusable across single-cycle, multi-cycle, and pipelined
 *   implementations of the MRV32 core.
 */

module imm_gen (
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