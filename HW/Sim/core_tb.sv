module core_tb;

  import mrv32_pkg::*;

  // -------------------------
  // Parameters
  // -------------------------
  localparam integer RD_LATENCY = 2;

  // -------------------------
  // Runtime arguments
  // -------------------------
  integer      MAX_INSTRS;
  logic [31:0] dump_start;
  integer      dump_len;
  bit          trace_retire;

  initial begin
    MAX_INSTRS   = 10000;
    dump_start   = 32'h00000000;
    dump_len     = 64;
    trace_retire = 0;

    begin
      integer tmp;
      string  stmp;

      if ($value$plusargs("MAX_INSTRS=%d", tmp)) MAX_INSTRS   = tmp;
      if ($value$plusargs("DUMP_LEN=%d",   tmp)) dump_len     = tmp;
      if ($value$plusargs("TRACE=%d",      tmp)) trace_retire = tmp[0];
      if ($value$plusargs("DUMP_START=%s", stmp)) dump_start  = stmp.atohex();
    end

    $display("MAX_INSTRS  = %0d",    MAX_INSTRS);
    $display("DUMP_START  = 0x%08x", dump_start);
    $display("DUMP_LEN    = %0d",    dump_len);
    $display("TRACE       = %0b",    trace_retire);
  end

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

  int retired_instrs = 0;

  // -------------------------
  // Retirement Trace
  // -------------------------
  always @(posedge clk) begin
    if (rst_n && trace_retire && core.instr_valid_wb) begin
      $display("[CYCLE %0d] RETIRE | PC=0x%08h | NEXT_PC=0x%08h",
               cycle, core.pc_wb, core.pc_next);
    end
    if (rst_n && trace_retire && core.instr_valid_mem) begin
      $display("[CYCLE %0d] MEM | stall=%b, lsu_done=%b, waddr=%h, ld=%h",
               cycle, core.stall, core.lsu_done, core.b_addr, core.lsu.load_data);
    end
    if (rst_n && trace_retire) begin
      $display("[cycle %0d] aluop=%d, rs1_addr=%h, rs1_val=%h, rs2_addr=%h, rs2_val=%h, branch taken=%b, br_sel=%h, br_sel_mem=%h, z=%h",
               cycle, core.aluop_ex, core.rs1_addr, core.rs1_data,
               core.rs2_addr, core.rs2_data, core.take_branch,
               core.br_sel_ex, core.br_sel_mem, core.mrv_bru.is_Zero);
    end
    if (rst_n && core.instr_accept)
      retired_instrs = retired_instrs + 1;
  end

  // -------------------------
  // Core <-> Memory Wires
  // -------------------------
  logic                  a_valid;
  logic [ADDR_WIDTH-1:0] a_addr;
  logic [31:0]           a_wdata;
  logic [3:0]            a_wstrb;
  logic [31:0]           a_rdata;
  logic                  a_rvalid;

  logic                  b_valid;
  logic [ADDR_WIDTH-1:0] b_addr;
  logic [31:0]           b_wdata;
  logic [3:0]            b_wstrb;
  logic [31:0]           b_rdata;
  logic                  b_rvalid;

  logic                  illegal_instr;

  // -------------------------
  // Peripheral Wires
  // -------------------------
  logic        periph_valid;
  logic [31:0] periph_addr;
  logic [31:0] periph_wdata;
  logic        tohost_fired;

  // -------------------------
  // Instantiate Core
  // -------------------------
  mrv32_core core (
    .clk(clk),
    .rst_n(rst_n),

    .a_valid(a_valid),  .a_addr(a_addr),
    .a_wdata(a_wdata),  .a_wstrb(a_wstrb),
    .a_rdata(a_rdata),  .a_rvalid(a_rvalid),

    .b_valid(b_valid),  .b_addr(b_addr),
    .b_wdata(b_wdata),  .b_wstrb(b_wstrb),
    .b_rdata(b_rdata),  .b_rvalid(b_rvalid),

    .periph_valid(periph_valid),
    .periph_addr (periph_addr),
    .periph_wdata(periph_wdata),

    .illegal_instr(illegal_instr)
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

    .a_valid(a_valid),  .a_addr(a_addr),
    .a_wdata(a_wdata),  .a_wstrb(a_wstrb),
    .a_rdata(a_rdata),  .a_rvalid(a_rvalid),

    .b_valid(b_valid),  .b_addr(b_addr),
    .b_wdata(b_wdata),  .b_wstrb(b_wstrb),
    .b_rdata(b_rdata),  .b_rvalid(b_rvalid)
  );

  // -------------------------
  // Instantiate Sim Peripherals
  // -------------------------
  sim_peripherals periph (
    .clk         (clk),
    .periph_valid(periph_valid),
    .periph_addr (periph_addr),
    .periph_wdata(periph_wdata),
    .tohost_fired(tohost_fired)
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

    while (retired_instrs < MAX_INSTRS && !illegal_instr && !tohost_fired)
      @(posedge clk);

    $display("\n=======================================");
    if (tohost_fired) begin
      $display("Simulation ended: TOHOST signal");
    end
    else if (illegal_instr)
      $display("Simulation ended: illegal instruction");
    else
      $display("Simulation ended: MAX_INSTRS reached (%0d)", MAX_INSTRS);
    $display("Cycles:   %0d", cycle);
    $display("Retired:  %0d", retired_instrs);
    $display("CPI:      %.2f", real'(cycle) / real'(retired_instrs));
    $display("=======================================\n");

    dump_registers();
    dump_memory(dump_start, dump_len);

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
      if (i == 0) val = 32'd0;
      else        val = core.mrv_rf.registers[i-1];
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
    input integer length;
    integer a;
    reg [31:0] w;
    reg [7:0] b0, b1, b2, b3;
  begin
    $display("------ MEMORY DUMP (0x%08x, %0d bytes) ------", start_addr, length);
    for (a = start_addr; a < start_addr + length; a = a + 4) begin
      w = { mem.mem[a+3], mem.mem[a+2], mem.mem[a+1], mem.mem[a+0] };
      b0 = w[7:0]; b1 = w[15:8]; b2 = w[23:16]; b3 = w[31:24];
      $display("0x%08x : %02x %02x %02x %02x  (0x%08x)", a, b0, b1, b2, b3, w);
    end
    $display("--------------------------------------------\n");
  end
  endtask

endmodule