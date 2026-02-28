//==============================================================================
// Module: mrv32_bru v1.0
//------------------------------------------------------------------------------
// Description:
//   Branch resolution unit.
//
// Function:
//   - Evaluates branch conditions using ALU result flags
//   - Generates take_branch signal
//
// Supported Branches:
//   - BEQ, BNE
//   - BLT, BGE
//   - BLTU, BGEU
//
//   - JAL, JALR (Unconditional Branches)
//
// Notes:
//   Branch resolution currently occurs in MEM stage.
//
// Author: Martim Bento
// Date  : 28/02/2026
//==============================================================================

module mrv32_bru (

    input logic [1:0] br_sel,
    input logic [31:0] alu_result,
    output logic take_branch

);

    logic is_Zero;
    assign is_Zero = alu_result == 32'd0;

    always_comb begin
        case (br_sel)
            2'b00: take_branch = 1'b0; // not a branch
            2'b01: take_branch = 1'b1; // unconditional jump (JAL/JALR)
            2'b10: take_branch = is_Zero ? 1'b1 : 1'b0; // branch (BEQ, BLT, BLTU)
            2'b11: take_branch = is_Zero ? 1'b0 : 1'b1; // branch (BNE, BGE, BGEU)
            default: take_branch = 1'b0;
        endcase
    end

endmodule