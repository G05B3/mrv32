//==============================================================================
// Module: mrv32_fetch v1.1
//------------------------------------------------------------------------------
// Description:
//   Instruction Fetch stage for pipelined execution.
//
// Behavior:
//   - Continuously fetches instructions, advancing PC each cycle.
//   - PC advances by 4 each cycle unless take_branch is asserted,
//     in which case PC jumps to branch_target.
//   - Outputs NOP (0x00000013) and instr_valid=0 while waiting for
//     memory to return data or after a branch redirect.
//   - instr_valid=1 only when a_rvalid returns a real instruction.
//
// Notes:
//   Branch flush (squashing in-flight instructions) is not yet implemented.
//   Pipeline stages behind the branch will need to be invalidated separately.
//
// Interfaces:
//   - Memory request/response handshake (always fetching)
//   - take_branch / branch_target from MEM stage for PC redirect
//
// Author: Martim Bento
// Date  : 08/03/2026
//==============================================================================

import mrv32_pkg::*;

module mrv32_fetch (
    input  logic                  clk,
    input  logic                  rst_n,

    // Memory port A (instruction side)
    output logic                  a_valid,
    output logic [ADDR_WIDTH-1:0] a_addr,
    output logic [31:0]           a_wdata,
    output logic [3:0]            a_wstrb,
    input  logic [31:0]           a_rdata,
    input  logic                  a_rvalid,

    // Branch redirect from MEM stage
    input  logic                  take_branch,
    input  logic [31:0]           branch_target,

    // Stall from hazard unit
    input  logic                  stall,

    // Output to IF/ID stage register
    output logic [31:0]           instr,
    output logic [31:0]           pc,
    output logic                  instr_valid
);

  localparam NOP = 32'h00000013; // ADDI x0, x0, 0

  logic [31:0] pc_fetch; // PC of the instruction currently being fetched

  // PC register — advances every cycle unless stalled
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pc_fetch <= 32'd0;
    else if (!stall) begin
      if (take_branch)
        pc_fetch <= branch_target;
      else
        pc_fetch <= pc_fetch + 32'd4;
    end
  end

  // Always request a fetch
  assign a_valid = 1'b1;
  assign a_addr  = pc_fetch[ADDR_WIDTH-1:0];
  assign a_wdata = 32'd0;
  assign a_wstrb = 4'b0000;

  // Output to IF/ID register
  // pc_fetch is registered so it trails the fetch address by one cycle,
  // matching the cycle when a_rvalid returns
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      instr       <= NOP;
      pc          <= 32'd0;
      instr_valid <= 1'b0;
    end else if (!stall) begin
      instr       <= a_rvalid ? a_rdata : NOP;
      pc          <= pc_fetch;
      instr_valid <= a_rvalid;
    end
  end

endmodule