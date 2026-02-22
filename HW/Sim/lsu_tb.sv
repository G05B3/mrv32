`timescale 1ns/1ps

module tb_mrv32_lsu;

  import mrv32_pkg::*;

  // ------------------------
  // Clock / Reset
  // ------------------------
  logic clk;
  logic rst_n;

  initial clk = 0;
  always #5 clk = ~clk; // 100 MHz

  task automatic apply_reset();
    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);
  endtask

  // ------------------------
  // LSU interface
  // ------------------------
  logic        mem_valid;
  logic        mem_ren;
  logic        mem_wen;
  logic [3:0]  mem_wstrb;
  logic [2:0]  load_funct3;
  logic [31:0] eff_addr;
  logic [31:0] store_data;

  logic        lsu_done;
  logic [31:0] load_data;

  // Memory port B wires
  logic                  b_valid;
  logic [ADDR_WIDTH-1:0] b_addr;
  logic [31:0]           b_wdata;
  logic [3:0]            b_wstrb;
  logic [31:0]           b_rdata;
  logic                  b_rvalid;

  // ------------------------
  // Unused Port A wires (tie off)
  // ------------------------
  logic                  a_valid;
  logic [ADDR_WIDTH-1:0] a_addr;
  logic [31:0]           a_wdata;
  logic [3:0]            a_wstrb;
  logic [31:0]           a_rdata;
  logic                  a_rvalid;

  assign a_valid = 1'b0;
  assign a_addr  = '0;
  assign a_wdata = '0;
  assign a_wstrb = '0;

  // ------------------------
  // DUTs
  // ------------------------
  dual_port_byte_mem #(
    .MEM_BYTES   (MEM_BYTES),
    .ADDR_WIDTH  (ADDR_WIDTH),
    .RD_LATENCY  (1),
    .WRITE_FIRST (0)
  ) mem (
    .clk     (clk),

    .a_valid (a_valid),
    .a_addr  (a_addr),
    .a_wdata (a_wdata),
    .a_wstrb (a_wstrb),
    .a_rdata (a_rdata),
    .a_rvalid(a_rvalid),

    .b_valid (b_valid),
    .b_addr  (b_addr),
    .b_wdata (b_wdata),
    .b_wstrb (b_wstrb),
    .b_rdata (b_rdata),
    .b_rvalid(b_rvalid)
  );

  mrv32_lsu dut (
    .clk         (clk),
    .rst_n       (rst_n),

    .mem_valid   (mem_valid),
    .mem_ren     (mem_ren),
    .mem_wen     (mem_wen),
    .mem_wstrb   (mem_wstrb),
    .load_funct3 (load_funct3),
    .eff_addr    (eff_addr),
    .store_data  (store_data),

    .lsu_done    (lsu_done),
    .load_data   (load_data),

    .b_valid     (b_valid),
    .b_addr      (b_addr),
    .b_wdata     (b_wdata),
    .b_wstrb     (b_wstrb),
    .b_rdata     (b_rdata),
    .b_rvalid    (b_rvalid)
  );

  // ------------------------
  // Helpers
  // ------------------------
  task automatic idle_bus();
    mem_valid   = 0;
    mem_ren     = 0;
    mem_wen     = 0;
    mem_wstrb   = WSTRB_NONE;
    load_funct3 = 3'b000;
    eff_addr    = 32'd0;
    store_data  = 32'd0;
  endtask

  // Hold request stable until lsu_done observed on a posedge
  task automatic do_store(input logic [31:0] addr,
                          input logic [3:0]  wstrb_kind,   // WSTRB_B/H/W from pkg
                          input logic [31:0] data);
    // Drive
    eff_addr    = addr;
    store_data  = data;
    mem_wstrb   = wstrb_kind;
    mem_wen     = 1;
    mem_ren     = 0;
    mem_valid   = 1;

    // Wait completion
    do @(posedge clk); while (!lsu_done);

    // Drop request
    @(posedge clk);
    mem_valid = 0;
    mem_wen   = 0;
    mem_wstrb = WSTRB_NONE;
  endtask

    task automatic do_load(input  logic [31:0] addr,
                        input  logic [2:0]  f3,
                        output logic [31:0] data_out);
    eff_addr    = addr;
    load_funct3 = f3;
    mem_wen     = 0;
    mem_ren     = 1;
    mem_wstrb   = WSTRB_NONE;
    mem_valid   = 1;

    // Wait for completion
    do @(posedge clk); while (!lsu_done);

    // Let NBA updates to load_data settle, then sample
    #1 
    data_out = load_data;

    // Drop request
    @(posedge clk);
    mem_valid = 0;
    mem_ren   = 0;
    endtask

  task automatic expect_eq(input string name, input logic [31:0] got, input logic [31:0] exp);
    if (got !== exp) begin
      $display("FAIL: %s got=0x%08x exp=0x%08x @ t=%0t", name, got, exp, $time);
      $fatal(1);
    end else begin
      $display("PASS: %s got=0x%08x", name, got);
    end
  endtask

  // Direct byte peek (optional extra check of memory layout)
  function automatic [7:0] peek8(input int unsigned addr);
    peek8 = mem.mem[addr];
  endfunction


    logic [31:0] r;

    // Pick some safe in-range addresses
    logic [31:0] base;

  // ------------------------
  // Test sequence
  // ------------------------
  initial begin
    idle_bus();
    apply_reset();

    base = 32'h0000_0100;

    // ------------------------------------------------------------
    // 1) SW then LW
    // ------------------------------------------------------------
    do_store(base + 32'd0, WSTRB_W, 32'hA1B2_C3D4);

    // Memory little-endian check
    if (peek8(base+0) !== 8'hD4) begin $display("Endian byte0 mismatch"); $finish; end
    if (peek8(base+1) !== 8'hC3) begin $display("Endian byte1 mismatch"); $finish; end
    if (peek8(base+2) !== 8'hB2) begin $display("Endian byte2 mismatch"); $finish; end
    if (peek8(base+3) !== 8'hA1) begin $display("Endian byte3 mismatch"); $finish; end

    do_load(base + 32'd0, 3'b010 /*LW*/, r);
    expect_eq("LW after SW", r, 32'hA1B2_C3D4);

    // ------------------------------------------------------------
    // 2) SB then LB/LBU (sign/zero extend)
    // ------------------------------------------------------------
    // write 0x80 at base+1 (negative if signed byte)
    do_store(base + 32'd1, WSTRB_B, 32'h0000_0080);

    do_load(base + 32'd1, 3'b000 /*LB*/, r);
    expect_eq("LB sign-extend 0x80", r, 32'hFFFF_FF80);

    do_load(base + 32'd1, 3'b100 /*LBU*/, r);
    expect_eq("LBU zero-extend 0x80", r, 32'h0000_0080);

    // ------------------------------------------------------------
    // 3) SH then LH/LHU (sign/zero extend)
    // ------------------------------------------------------------
    // write halfword 0x8001 at base+2 (aligned for halfword)
    do_store(base + 32'd2, WSTRB_H, 32'h0000_8001);

    do_load(base + 32'd2, 3'b001 /*LH*/, r);
    expect_eq("LH sign-extend 0x8001", r, 32'hFFFF_8001);

    do_load(base + 32'd2, 3'b101 /*LHU*/, r);
    expect_eq("LHU zero-extend 0x8001", r, 32'h0000_8001);

    // ------------------------------------------------------------
    // 4) Misaligned LW should be ignored => returns 0
    // ------------------------------------------------------------
    do_load(base + 32'd2, 3'b010 /*LW misaligned*/, r);
    expect_eq("Misaligned LW returns 0", r, 32'h0000_0000);

    // ------------------------------------------------------------
    // 5) Unmapped/out-of-range access ignored => returns 0
    // ------------------------------------------------------------
    // eff_addr >= MEM_BYTES => ram_hit=0 in LSU
    do_load(MEM_BYTES + 32'd16, 3'b010 /*LW*/, r);
    expect_eq("Unmapped LW returns 0", r, 32'h0000_0000);

    // Unmapped store should not affect valid memory (spot check)
    do_store(MEM_BYTES + 32'd32, WSTRB_W, 32'hDEAD_BEEF);
    do_load(base + 32'd0, 3'b010 /*LW*/, r);
    // base+0 word was modified by SB/SH, so just ensure it’s not DEAD_BEEF
    if (r === 32'hDEAD_BEEF) begin
        $display("Unmapped store corrupted RAM!"); $finish;
    end

    $display("ALL TESTS PASSED");
    $finish;
  end

endmodule