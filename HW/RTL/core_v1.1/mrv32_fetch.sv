import mrv32_pkg::*;

//==============================================================================
// Module: mrv32_fetch v1.1
//------------------------------------------------------------------------------
// Description:
//   Instruction fetch stage for RV32I subset.
//
// Responsibilities:
//   - Fetch instructions from memory
//   - Manage instruction queue
//   - Handle branch and jump resolution
//
// Output:
//   - Instruction and PC to decode stage
//
// Version v1.1 notes:
// Design notes:
//  - Issuance of new memory requests (a_valid) is NEVER throttled by an
//    unresolved branch/jump; fetch keeps reading ahead into the queue.
//  - Only DELIVERY (handing an instruction to ID) freezes while a branch or
//    jump sits unresolved in ID or EX. The core computes this as
//    `branch_pending` and feeds it in on `stall`, together with any
//    load_stall / global_stall conditions.
//  - On take_branch, the ENTIRE in-flight queue's `wantq` is unconditionally
//    cleared (marking every outstanding/queued entry as "don't deliver").
//    Flushed entries just drain as harmless NOP bubbles as head catches up.
//    This replaces an earlier, much more fragile "dedup matching already
//    in-flight entries" design — once delivery itself never runs ahead of
//    an unresolved branch, no instruction past the branch is ever delivered
//    before the branch resolves, so there is nothing to de-duplicate.
//
// Author: Martim Bento
// Date  : 06/10/2026
//==============================================================================

module mrv32_fetch #(
    parameter integer MAX_OUTSTANDING = 8
) (
    input  logic                  clk,
    input  logic                  rst_n,
    output logic                  a_valid,
    output logic [ADDR_WIDTH-1:0] a_addr,
    output logic [31:0]           a_wdata,
    output logic [3:0]            a_wstrb,
    input  logic [31:0]           a_rdata,
    input  logic                  a_rvalid,
    input  logic [31:0]           branch_target,
    input  logic                  take_branch,
    input  logic                  stall,     // freezes DELIVERY only (global_stall | load_stall | branch_pending)
    output logic [31:0]           instr,
    output logic [31:0]           pc,
    output logic                  instr_valid
);
    localparam integer QW = $clog2(MAX_OUTSTANDING);

    logic [31:0]  pc_fetch;
    logic [QW:0]  tail, recv, head;
    logic [QW:0]  depth;

    assign depth   = tail - head;
    assign a_valid = rst_n && (depth < MAX_OUTSTANDING);
    assign a_addr  = pc_fetch[ADDR_WIDTH-1:0];
    assign a_wdata = 32'd0;
    assign a_wstrb = 4'd0;

    logic [31:0] pcq   [0:MAX_OUTSTANDING-1];
    logic [31:0] dataq [0:MAX_OUTSTANDING-1];
    logic        wantq [0:MAX_OUTSTANDING-1];
    logic        readyq[0:MAX_OUTSTANDING-1];

    logic can_deliver;
    assign can_deliver = !stall && (head != tail) && readyq[head[QW-1:0]];

    assign instr       = wantq[head[QW-1:0]] ? dataq[head[QW-1:0]] : 32'h00000013; // NOP
    assign pc          = pcq[head[QW-1:0]];
    assign instr_valid = can_deliver && wantq[head[QW-1:0]];

    integer fi;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tail <= '0;
            recv <= '0;
            head <= '0;
            pc_fetch <= 32'd0;
            for (fi = 0; fi < MAX_OUTSTANDING; fi = fi + 1) begin
                wantq[fi]  <= 1'b0;
                readyq[fi] <= 1'b0;
            end
        end else begin
            if (take_branch) begin
                for (fi = 0; fi < MAX_OUTSTANDING; fi = fi + 1) wantq[fi] <= 1'b0;
            end

            if (a_valid) begin
                pcq[tail[QW-1:0]]   <= pc_fetch;
                wantq[tail[QW-1:0]] <= !take_branch;
                tail <= tail + 1'b1;
            end

            if (a_rvalid && ((recv != tail) || a_valid)) begin
                dataq[recv[QW-1:0]]  <= a_rdata;
                readyq[recv[QW-1:0]] <= 1'b1;
                recv <= recv + 1'b1;
            end

            if (can_deliver) begin
                readyq[head[QW-1:0]] <= 1'b0;
                head <= head + 1'b1;
            end

            if (take_branch)  pc_fetch <= branch_target;
            else if (a_valid) pc_fetch <= pc_fetch + 32'd4;
        end
    end
endmodule