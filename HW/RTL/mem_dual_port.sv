// =============================================================================
// dual_port_byte_mem
// =============================================================================
// Byte-addressed, dual-port memory model for RTL simulation.
//
// Intended use:
//   - CPU bring-up and cycle-level simulation (NOT meant for synthesis).
//   - Works well as a stand-in for an I$ (Port A) and D$ / LSU (Port B).
//   - Supports loading program images via $readmemh into the public `mem[]` array.
//
// Addressing / Endianness:
//   - Byte addressed: address selects a byte in `mem[]`.
//   - 32-bit reads return 4 consecutive bytes in little-endian order:
//       rdata[7:0]   = mem[addr+0]
//       rdata[15:8]  = mem[addr+1]
//       rdata[23:16] = mem[addr+2]
//       rdata[31:24] = mem[addr+3]
//   - This matches RV32 little-endian instruction/data layout.
//
// Ports / Protocol (both ports identical):
//   Inputs:
//     *valid : when 1, a request is accepted on the rising edge of clk.
//     *addr  : byte address for the request.
//     *wdata : write data (used when *wstrb != 0).
//     *wstrb : write strobe per byte lane (little-endian lanes):
//              wstrb[0] -> writes mem[addr+0] with wdata[7:0]
//              wstrb[1] -> writes mem[addr+1] with wdata[15:8]
//              wstrb[2] -> writes mem[addr+2] with wdata[23:16]
//              wstrb[3] -> writes mem[addr+3] with wdata[31:24]
//              wstrb == 0 implies a read request.
//   Outputs:
//     *rvalid: asserted when *rdata corresponds to a previous accepted request.
//     *rdata : 32-bit read data corresponding to the address captured with the
//              request that produced this response.
//
// Latency:
//   - RD_LATENCY parameter controls the internal request pipeline depth.
//   - This model pipelines requests and produces responses in order.
//   - Effective request->response latency depends on implementation details;
//     users should rely on *rvalid rather than assuming a fixed cycle count.
//
// Ordering / Concurrency:
//   - Each port is independent; both ports can accept one request per cycle.
//   - Responses for a given port are returned in the same order requests were
//     accepted on that port.
//   - No backpressure/ready signal is modeled (always "accepts" when *valid=1).
//
// Simultaneous read/write to same address:
//   - WRITE_FIRST parameter (best-effort model):
//       0: read-first (default) behavior in ambiguous same-cycle cases
//       1: write-first behavior for the special case RD_LATENCY==1 and same-port
//          read+write overlap (limited corner-case support; see code).
//
// Bounds behavior:
//   - Byte reads out of range return 0.
//   - Writes out of range are ignored (guarded by address checks).
//
// Notes:
//   - Because `mem` is declared as a public byte array, a testbench can do:
//       $readmemh("program.hex", dut.mem);
//   - If you later add caches or a bus, keep using *rvalid to align responses.
// =============================================================================
module dual_port_byte_mem #(
  parameter integer MEM_BYTES    = 64 * 1024,
  parameter integer ADDR_WIDTH   = $clog2(MEM_BYTES),

  // Read latency in cycles (>=1 recommended for CPU simulation)
  parameter integer RD_LATENCY   = 1,

  // If 1: model "write-first" when read+write same address same cycle on same port
  // If 0: model "read-first" (read old data). This mainly matters in corner cases.
  parameter integer WRITE_FIRST  = 0
) (
  input  logic                  clk,

  // Port A
  input  logic                  a_valid,
  input  logic [ADDR_WIDTH-1:0]  a_addr,
  input  logic [31:0]           a_wdata,
  input  logic [3:0]            a_wstrb,
  output logic [31:0]           a_rdata,
  output logic                  a_rvalid,   // NEW: indicates rdata corresponds to a past request

  // Port B
  input  logic                  b_valid,
  input  logic [ADDR_WIDTH-1:0]  b_addr,
  input  logic [31:0]           b_wdata,
  input  logic [3:0]            b_wstrb,
  output logic [31:0]           b_rdata,
  output logic                  b_rvalid
);

  // Byte-addressed storage
  byte mem [0:MEM_BYTES-1];

  function automatic [7:0] rd8(input integer unsigned addr);
    if (addr < MEM_BYTES) rd8 = mem[addr];
    else                  rd8 = 8'h00;
  endfunction

  // -------------------------
  // Port A pipelines
  // -------------------------
  logic [ADDR_WIDTH-1:0] a_addr_pipe [0:RD_LATENCY-1];
  logic                  a_v_pipe    [0:RD_LATENCY-1];

  // -------------------------
  // Port B pipelines
  // -------------------------
  logic [ADDR_WIDTH-1:0] b_addr_pipe [0:RD_LATENCY-1];
  logic                  b_v_pipe    [0:RD_LATENCY-1];

  integer i;

  // Optional init (prevents Xs on outputs)
  initial begin
    a_rdata  = 32'h0;
    b_rdata  = 32'h0;
    a_rvalid = 1'b0;
    b_rvalid = 1'b0;

    // Init pipelines to 0
    for (i = 0; i < RD_LATENCY; i = i + 1) begin
      a_addr_pipe[i] = '0; a_v_pipe[i] = 1'b0;
      b_addr_pipe[i] = '0; b_v_pipe[i] = 1'b0;
    end
  end

  // -------------------------
  // Writes (posedge)
  // -------------------------
  always_ff @(posedge clk) begin
    if (a_valid) begin
      if (a_wstrb[0] && (a_addr + 0 < MEM_BYTES)) mem[a_addr + 0] <= a_wdata[7:0];
      if (a_wstrb[1] && (a_addr + 1 < MEM_BYTES)) mem[a_addr + 1] <= a_wdata[15:8];
      if (a_wstrb[2] && (a_addr + 2 < MEM_BYTES)) mem[a_addr + 2] <= a_wdata[23:16];
      if (a_wstrb[3] && (a_addr + 3 < MEM_BYTES)) mem[a_addr + 3] <= a_wdata[31:24];
    end

    if (b_valid) begin
      if (b_wstrb[0] && (b_addr + 0 < MEM_BYTES)) mem[b_addr + 0] <= b_wdata[7:0];
      if (b_wstrb[1] && (b_addr + 1 < MEM_BYTES)) mem[b_addr + 1] <= b_wdata[15:8];
      if (b_wstrb[2] && (b_addr + 2 < MEM_BYTES)) mem[b_addr + 2] <= b_wdata[23:16];
      if (b_wstrb[3] && (b_addr + 3 < MEM_BYTES)) mem[b_addr + 3] <= b_wdata[31:24];
    end
  end

  // -------------------------
  // Read request pipelines (posedge)
  // -------------------------
  always_ff @(posedge clk) begin
    // stage 0 captures incoming request
    a_addr_pipe[0] <= a_addr;
    a_v_pipe[0]    <= a_valid;

    b_addr_pipe[0] <= b_addr;
    b_v_pipe[0]    <= b_valid;

    // shift down the pipeline
    for (i = 1; i < RD_LATENCY; i = i + 1) begin
      a_addr_pipe[i] <= a_addr_pipe[i-1];
      a_v_pipe[i]    <= a_v_pipe[i-1];

      b_addr_pipe[i] <= b_addr_pipe[i-1];
      b_v_pipe[i]    <= b_v_pipe[i-1];
    end
  end

  // -------------------------
  // Read response generation (posedge)
  // Data corresponds to addr_pipe[RD_LATENCY-1]
  // -------------------------
  always_ff @(posedge clk) begin : RESP
    logic [ADDR_WIDTH-1:0] aa;
    logic [ADDR_WIDTH-1:0] bb;
    logic [31:0] a_word;
    logic [31:0] b_word;

    aa = a_addr_pipe[RD_LATENCY-1];
    bb = b_addr_pipe[RD_LATENCY-1];

    // Base read data (little-endian assembly)
    a_word = { rd8(aa + 3), rd8(aa + 2), rd8(aa + 1), rd8(aa + 0) };
    b_word = { rd8(bb + 3), rd8(bb + 2), rd8(bb + 1), rd8(bb + 0) };

    // Optional same-port write-first modeling (only for same cycle as request stage 0)
    // This is a corner-case model; you can ignore if you don't care.
    if (WRITE_FIRST) begin
      // If the *request being returned now* was issued RD_LATENCY cycles ago,
      // we do NOT track the historical write strobes for that request here.
      // So WRITE_FIRST is only meaningful when RD_LATENCY==1.
      if (RD_LATENCY == 1) begin
        if (a_valid && (a_wstrb != 0) && (a_addr == aa)) begin
          if (a_wstrb[0]) a_word[7:0]   = a_wdata[7:0];
          if (a_wstrb[1]) a_word[15:8]  = a_wdata[15:8];
          if (a_wstrb[2]) a_word[23:16] = a_wdata[23:16];
          if (a_wstrb[3]) a_word[31:24] = a_wdata[31:24];
        end
        if (b_valid && (b_wstrb != 0) && (b_addr == bb)) begin
          if (b_wstrb[0]) b_word[7:0]   = b_wdata[7:0];
          if (b_wstrb[1]) b_word[15:8]  = b_wdata[15:8];
          if (b_wstrb[2]) b_word[23:16] = b_wdata[23:16];
          if (b_wstrb[3]) b_word[31:24] = b_wdata[31:24];
        end
      end
    end

    a_rdata  <= a_word;
    b_rdata  <= b_word;
    a_rvalid <= a_v_pipe[RD_LATENCY-1];
    b_rvalid <= b_v_pipe[RD_LATENCY-1];
  end

endmodule