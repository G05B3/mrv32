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

  function automatic [7:0] rd8(input integer unsigned addr);
    if (addr < MEM_BYTES) rd8 = mem[addr];
    else                  rd8 = 8'h00;
  endfunction

  // Port A — combinational read, always valid
  assign a_rdata  = {rd8(a_addr+3), rd8(a_addr+2), rd8(a_addr+1), rd8(a_addr+0)};
  assign a_rvalid = 1'b1;

  // Port B — combinational read, always valid
  assign b_rdata  = {rd8(b_addr+3), rd8(b_addr+2), rd8(b_addr+1), rd8(b_addr+0)};
  assign b_rvalid = 1'b1;

  // Synchronous writes
  always_ff @(posedge clk) begin
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

endmodule