module core_tb;

  import mrv32_pkg::*;

  // -------------------------
  // Parameters
  // -------------------------
  localparam integer RD_LATENCY = 2;
  localparam integer MAX_CYCLES = 500;

  // -------------------------
  // Clock / Reset
  // -------------------------
  logic clk;
  logic rst_n;

  initial clk = 0;
  always #5 clk = ~clk;

  initial begin
    rst_n = 0;
    repeat (5) @(posedge clk);
    rst_n = 1;
  end

  integer cycle;
  always @(posedge clk)
    if (!rst_n) cycle <= 0;
    else        cycle <= cycle + 1;

  // -------------------------
  // Retirement Trace Control
  // -------------------------
  bit trace_retire;

  initial begin
    trace_retire = 0; // Change this depending on whether to trace or not
    if (trace_retire) begin
      $display("Retirement trace ENABLED");
    end
    else begin
      $display("Retirement trace DISABLED");
    end
  end

  // -------------------------
  // Retirement Trace
  // -------------------------
  always @(posedge clk) begin
    if (rst_n && trace_retire && core.instr_valid_wb) begin
      $display("[CYCLE %0d] RETIRE | PC=0x%08h | NEXT_PC=0x%08h",
               cycle,
               core.pc_wb,
               core.pc_next);
    end
    if (rst_n && trace_retire && core.instr_valid_mem) begin
        $display("[CYCLE %0d] MEM | stall=%b, lsu_done=%b, waddr=%h, ld=%h",cycle,core.stall, core.lsu_done, core.b_addr, core.lsu.load_data);
    end
  end

  // -------------------------
  // Core <-> Memory Wires
  // -------------------------

  // Port A (Instruction)
  logic                  a_valid;
  logic [ADDR_WIDTH-1:0] a_addr;
  logic [31:0]           a_wdata;
  logic [3:0]            a_wstrb;
  logic [31:0]           a_rdata;
  logic                  a_rvalid;

  // Port B (Data)
  logic                  b_valid;
  logic [ADDR_WIDTH-1:0] b_addr;
  logic [31:0]           b_wdata;
  logic [3:0]            b_wstrb;
  logic [31:0]           b_rdata;
  logic                  b_rvalid;

  logic unsupported_instr;

  // -------------------------
  // Instantiate Core
  // -------------------------
  mrv32_core core (
    .clk(clk),
    .rst_n(rst_n),

    .a_valid(a_valid),
    .a_addr(a_addr),
    .a_wdata(a_wdata),
    .a_wstrb(a_wstrb),
    .a_rdata(a_rdata),
    .a_rvalid(a_rvalid),

    .b_valid(b_valid),
    .b_addr(b_addr),
    .b_wdata(b_wdata),
    .b_wstrb(b_wstrb),
    .b_rdata(b_rdata),
    .b_rvalid(b_rvalid),

    .unsupported_instr(unsupported_instr)
  );

  // -------------------------
  // Instantiate Memory
  // -------------------------
  dual_port_byte_mem #(
    .MEM_BYTES(MEM_BYTES),
    .ADDR_WIDTH(ADDR_WIDTH),
    .RD_LATENCY(RD_LATENCY)
  ) mem (
    .clk(clk),

    .a_valid(a_valid),
    .a_addr(a_addr),
    .a_wdata(a_wdata),
    .a_wstrb(a_wstrb),
    .a_rdata(a_rdata),
    .a_rvalid(a_rvalid),

    .b_valid(b_valid),
    .b_addr(b_addr),
    .b_wdata(b_wdata),
    .b_wstrb(b_wstrb),
    .b_rdata(b_rdata),
    .b_rvalid(b_rvalid)
  );

  // -------------------------
  // Program Loading
  // -------------------------
  initial begin
    string prog;
    prog = "program.hex";
    void'($value$plusargs("PROG=%s", prog));

    $display("Loading program: %s", prog);
    $readmemh(prog, mem.mem);
  end

  // -------------------------
  // Simulation Control
  // -------------------------
  initial begin
    wait(rst_n);

    while (cycle < MAX_CYCLES && !unsupported_instr)
      @(posedge clk);

    $display("\n=======================================");
    $display("Simulation finished at cycle %0d", cycle);
    $display("=======================================\n");

    dump_registers();
    dump_memory(32'h0000ffc, 64);

    $finish;
  end

  // -------------------------
  // Dump Register File
  // -------------------------
  task dump_registers;
    integer i;
    reg [31:0] val;
  begin
    $display("------ REGISTER FILE ------");

    for (i = 0; i < 32; i++) begin
      if (i == 0)
        val = 32'd0;
      else
        val = core.mrv_rf.registers[i-1];

      $display("x%0d = 0x%08x", i, val);
    end

    $display("---------------------------\n");
  end
  endtask

  // -------------------------
  // Dump Memory (word aligned)
  // -------------------------
  task dump_memory;
    input [31:0] start_addr;
    input [31:0] length;
    integer a;
    reg [31:0] w;
    reg [7:0] b0, b1, b2, b3;
  begin
    $display("------ MEMORY DUMP ------");

    for (a = start_addr; a < start_addr + length; a = a + 4) begin
      w = {
        mem.mem[a+3],
        mem.mem[a+2],
        mem.mem[a+1],
        mem.mem[a+0]
      };

      b0 = w[7:0];
      b1 = w[15:8];
      b2 = w[23:16];
      b3 = w[31:24];

      $display("0x%08x : %02x %02x %02x %02x  (0x%08x)",
               a, b0, b1, b2, b3, w);
    end

    $display("--------------------------\n");
  end
  endtask

endmodule