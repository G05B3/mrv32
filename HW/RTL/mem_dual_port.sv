// =============================================================================
// dual_port_byte_mem (instant - zero latency)
// =============================================================================
// Byte-addressed, dual-port memory model for RTL simulation.
//
// Intended use:
//   - CPU bring-up and hazard validation (NOT meant for synthesis).
//   - Zero-latency combinational reads on both ports.
//   - Drop-in replacement for the pipelined dual_port_byte_mem.
//   - RD_LATENCY and WRITE_FIRST parameters kept for compatibility but ignored.
//
// Addressing / Endianness:
//   - Byte addressed: address selects a byte in `mem[]`.
//   - 32-bit reads return 4 consecutive bytes in little-endian order.
//
// Notes:
//   - a_rvalid and b_rvalid are always 1.
//   - Writes are still synchronous (posedge clk).
//   - Use $readmemh to load programs: $readmemh("prog.hex", dut.mem);
//   - The read path is NOT a plain `assign`/`always_comb` over mem[] indexed by
//     a variable, because several simulators don't reliably resensitize such a
//     read when only the array contents change and the index stays fixed
//     (exactly what happens on sw followed by lw to the same held address).
//     `always_comb`/`always @*` is also prohibitively slow to elaborate here
//     since sensitivity analysis has to consider the whole array. Instead, a
//     small `wr_gen` counter increments every cycle in the same clocked
//     process as the writes, and the read process is purely level-sensitive
//     to `wr_gen` (plus the addresses) -- this guarantees a re-evaluation
//     every cycle, after that cycle's writes have committed, using nothing
//     but small explicit signals. `wr_gen` is explicitly initialized (X + 1
//     stays X forever in some simulators otherwise, which silently defeats
//     this whole mechanism).
// =============================================================================

module dual_port_byte_mem #(
  parameter integer MEM_BYTES   = 64 * 1024,
  parameter integer ADDR_WIDTH  = $clog2(MEM_BYTES),
  parameter integer RD_LATENCY  = 1,  // ignored, kept for compatibility
  parameter integer WRITE_FIRST = 0   // ignored, kept for compatibility
) (
  input  logic                  clk,

  // Port A
  input  logic                  a_valid,
  input  logic [ADDR_WIDTH-1:0] a_addr,
  input  logic [31:0]           a_wdata,
  input  logic [3:0]            a_wstrb,
  output logic [31:0]           a_rdata,
  output logic                  a_rvalid,

  // Port B
  input  logic                  b_valid,
  input  logic [ADDR_WIDTH-1:0] b_addr,
  input  logic [31:0]           b_wdata,
  input  logic [3:0]            b_wstrb,
  output logic [31:0]           b_rdata,
  output logic                  b_rvalid
);

  byte mem [0:MEM_BYTES-1];

  assign a_rvalid = 1'b1;
  assign b_rvalid = 1'b1;

  // Ticks every cycle unconditionally, purely to give the read process below
  // something small and explicit to be sensitive to (see header note).
  logic [3:0] wr_gen;
  initial wr_gen = 4'd0;

  // Synchronous writes
  always_ff @(posedge clk) begin
    wr_gen <= wr_gen + 4'd1;
    if (a_valid) begin
      if (a_wstrb[0] && (a_addr+0 < MEM_BYTES)) mem[a_addr+0] <= a_wdata[7:0];
      if (a_wstrb[1] && (a_addr+1 < MEM_BYTES)) mem[a_addr+1] <= a_wdata[15:8];
      if (a_wstrb[2] && (a_addr+2 < MEM_BYTES)) mem[a_addr+2] <= a_wdata[23:16];
      if (a_wstrb[3] && (a_addr+3 < MEM_BYTES)) mem[a_addr+3] <= a_wdata[31:24];
    end
    if (b_valid) begin
      if (b_wstrb[0] && (b_addr+0 < MEM_BYTES)) mem[b_addr+0] <= b_wdata[7:0];
      if (b_wstrb[1] && (b_addr+1 < MEM_BYTES)) mem[b_addr+1] <= b_wdata[15:8];
      if (b_wstrb[2] && (b_addr+2 < MEM_BYTES)) mem[b_addr+2] <= b_wdata[23:16];
      if (b_wstrb[3] && (b_addr+3 < MEM_BYTES)) mem[b_addr+3] <= b_wdata[31:24];
    end
  end

  // Port A/B — "combinational" read, always valid. Purely level-sensitive.
  always @(a_addr or b_addr or wr_gen) begin
    a_rdata = (a_addr+3 < MEM_BYTES) ? {mem[a_addr+3], mem[a_addr+2], mem[a_addr+1], mem[a_addr+0]} : 32'h0;
    b_rdata = (b_addr+3 < MEM_BYTES) ? {mem[b_addr+3], mem[b_addr+2], mem[b_addr+1], mem[b_addr+0]} : 32'h0;
  end

endmodule