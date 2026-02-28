//==============================================================================
// Module: mrv32_fetch v1.0
//------------------------------------------------------------------------------
// Description:
//   Instruction Fetch stage with simple request/accept handshake.
//
// Behavior:
//   - Issues instruction fetch requests to instruction memory.
//   - Holds fetched instruction until accepted by WB stage.
//   - PC advances only when instr_accept is asserted.
//   - Guarantees only one instruction is active in the system.
//
// Notes:
//   This stage intentionally prevents instruction overlap to simplify
//   bring-up and eliminate pipeline hazards.
//
// Interfaces:
//   - Memory request/response handshake
//   - instr_valid / instr_accept handshake with WB
//
// Author: Martim Bento
// Date  : 28/02/2026
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

    // Control from core
    input  logic [31:0]           pc_next,    // next PC value (byte address)

    // Output to core (latched instruction)
    output logic [31:0]           instr,
    output logic [31:0]           pc,
    output logic                  instr_valid, // 1 when instr holds a valid fetched instruction
    input  logic                  instr_accept // core pulses when it consumed instr (update PC)
);

  typedef enum logic [1:0] { IF_IDLE, IF_REQ, IF_WAIT, IF_HAVE } if_state_t;
  if_state_t state, state_n;

  logic [31:0] pc_n;
  logic [31:0] instr_n;
  logic        instr_valid_n;

  // Memory read is always (wstrb==0)
  assign a_wdata = 32'd0;
  assign a_wstrb = 4'b0000;

  // Address is current PC truncated to memory address width (byte addressing)
  assign a_addr  = pc[ADDR_WIDTH-1:0];

  // State / registers
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state       <= IF_IDLE;
      pc          <= 32'd0;
      instr       <= 32'd0;
      instr_valid <= 1'b0;
    end else begin
      state       <= state_n;
      pc          <= pc_n;
      instr       <= instr_n;
      instr_valid <= instr_valid_n;
    end
  end

  // Next-state / outputs
  always_comb begin
    // defaults
    state_n       = state;
    pc_n          = pc;
    instr_n       = instr;
    instr_valid_n = instr_valid;

    a_valid       = 1'b0;

    // PC update from core (commit)
    if (instr_accept) begin
      pc_n = pc_next;
    end

    case (state)
      IF_IDLE: begin
        // Start by requesting first fetch immediately after reset release
        state_n = IF_REQ;
      end

      IF_REQ: begin
        // Enqueue a fetch request (one-cycle pulse)
        a_valid = 1'b1;
        state_n = IF_WAIT;
      end

      IF_WAIT: begin
        // Wait for memory to return instruction
        if (a_rvalid) begin
          instr_n       = a_rdata;
          instr_valid_n = 1'b1;
          state_n       = IF_HAVE;
        end
      end

      IF_HAVE: begin
        // Hold instruction stable until core accepts it
        if (instr_accept) begin
          state_n       = IF_REQ; // request next instruction (PC is updated by core via pc_set)
        end
        instr_valid_n = 1'b0;
        instr_n = 0;
      end

      default: begin
        state_n = IF_IDLE;
      end
    endcase
  end

endmodule