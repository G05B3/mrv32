module mem_tb;

  import mrv32_pkg::*;

  // Match your memory parameter
  localparam integer RD_LATENCY = 2;   // change freely (>=1)

  // How many random reads to issue as a stream
  localparam integer N_REQ = 7;

  // Clock
  reg clk;

  // Cycle counter (for nice prints)
  integer cycle;

  // Port A
  reg                   a_valid;
  reg  [ADDR_WIDTH-1:0]  a_addr;
  reg  [31:0]            a_wdata;
  reg  [3:0]             a_wstrb;
  wire [31:0]            a_rdata;
  wire                   a_rvalid;

  // Port B (unused)
  reg                   b_valid;
  reg  [ADDR_WIDTH-1:0]  b_addr;
  reg  [31:0]            b_wdata;
  reg  [3:0]             b_wstrb;
  wire [31:0]            b_rdata;
  wire                   b_rvalid;

  // Instantiate memory
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

  // Clock generation
  initial clk = 0;
  always #5 clk = ~clk;

  // Cycle counter
  initial cycle = 0;
  always @(posedge clk) cycle <= cycle + 1;

  // -------------------------
  // Dump memory range via port A (single request at a time, latency-aware)
  // -------------------------
  task read_word_a_lat;
    input  [31:0] addr;
    output [31:0] data;
    output integer resp_cycle;
    begin
      @(negedge clk);
      a_valid = 1'b1;
      a_wstrb = 4'b0000;
      a_addr  = addr[ADDR_WIDTH-1:0];

      // pulse for 1 cycle
      @(negedge clk);
      a_valid = 1'b0;

      // wait for response
      while (a_rvalid !== 1'b1) @(posedge clk);

      data = a_rdata;
      resp_cycle = cycle;
    end
  endtask

  task dump_range;
    input [31:0] start_addr;
    input [31:0] length;
    integer a;
    reg [31:0] w;
    reg [7:0] b0, b1, b2, b3;
    integer rc;
    begin
      $display("--------------------------------------------------");
      $display("Dump via Port A: start=0x%08x len=%0d bytes, RD_LATENCY=%0d",
               start_addr, length, RD_LATENCY);
      $display("--------------------------------------------------");

      for (a = start_addr; a < start_addr + length; a = a + 4) begin
        read_word_a_lat(a, w, rc);

        b0 = w[7:0];
        b1 = w[15:8];
        b2 = w[23:16];
        b3 = w[31:24];

        $display("[cycle %0d] 0x%08x : %02x %02x %02x %02x   (word=0x%08x)",
                 rc, a, b0, b1, b2, b3, w);
      end
    end
  endtask

  // -------------------------
  // NEW: issue a stream of reads (1 per cycle) and print responses
  // Keeps a small FIFO of requested addresses to match responses in-order.
  // -------------------------
  task random_read_stream;
  input integer nreq;

  integer issued;
  integer received;
  integer head, tail;
  integer outstanding;

  reg [31:0] req_fifo [0:1023]; // plenty for TB
  reg [31:0] addr;
  reg [31:0] w;
  reg [7:0]  b0, b1, b2, b3;
  reg [31:0] cnt;

  begin
    $display("--------------------------------------------------");
    $display("Random read stream on Port A: nreq=%0d (aim: 1 req/cycle), RD_LATENCY=%0d",
             nreq, RD_LATENCY);
    $display("--------------------------------------------------");

    // init
    issued      = 0;
    received    = 0;
    head        = 0;
    tail        = 0;
    outstanding = 0;

    a_wstrb = 4'b0000;
    a_wdata = 32'h0;
    a_valid = 1'b0;
    a_addr  = '0;
    cnt = 8;

    // Run until we've received all responses
    // Each loop iteration corresponds to ONE full clock cycle.
    while (received < nreq) begin

      // ---- Drive next request (at negedge) ----
      @(negedge clk);

      if (issued < nreq) begin
        // Pick random *word-aligned* address
        //addr = {$random} % (MEM_BYTES - 4);
        addr = cnt;
        addr = addr & 32'hFFFF_FFFC;
        cnt = cnt + 4;

        // Drive request
        a_valid = 1'b1;
        a_addr  = addr[ADDR_WIDTH-1:0];

        // Enqueue address for in-order matching
        req_fifo[tail] = addr;
        tail = tail + 1;

        issued      = issued + 1;
        outstanding = outstanding + 1;

        $display("[cycle %0d] issued req %0d: addr=0x%08x", cycle, issued-1, addr);
      end else begin
        // No more requests to issue
        a_valid = 1'b0;
      end

      // ---- Sample response (at posedge) ----
      @(posedge clk);

      if (a_rvalid === 1'b1) begin
        if (outstanding <= 0) begin
          $display("[cycle %0d] WARNING: got response but outstanding==0", cycle);
        end else begin
          addr = req_fifo[head];
          head = head + 1;

          w  = a_rdata;
          b0 = w[7:0];
          b1 = w[15:8];
          b2 = w[23:16];
          b3 = w[31:24];

          $display("[cycle %0d] resp %0d for addr=0x%08x : %02x %02x %02x %02x (word=0x%08x)",
                   cycle, received, addr, b0, b1, b2, b3, w);

          received    = received + 1;
          outstanding = outstanding - 1;
        end
      end
    end

    // deassert cleanly
    @(negedge clk);
    a_valid = 1'b0;

    $display("Random stream done.");
  end
endtask

  // -------------------------
  // Test sequence
  // -------------------------
  initial begin
    string prog;
    reg [31:0] start_addr;
    reg [31:0] len;
    integer ok;

    // Defaults
    prog       = "program.hex";
    start_addr = 32'h00000000;
    len        = 128;

    ok = $value$plusargs("PROG=%s",  prog);
    ok = $value$plusargs("START=%h", start_addr);
    ok = $value$plusargs("LEN=%d",   len);

    // Init ports
    a_valid = 0; a_addr = 0; a_wdata = 0; a_wstrb = 0;
    b_valid = 0; b_addr = 0; b_wdata = 0; b_wstrb = 0;

    // Load program
    $display("Loading hex file: %s", prog);
    $readmemh(prog, mem.mem);

    // Let things settle a couple cycles
    repeat (2) @(posedge clk);

    // Dump memory
    dump_range(start_addr, len);

    // NEW: Issue 7 random read requests as a stream, 1 per cycle
    random_read_stream(N_REQ);

    $display("ADDR WIDTH: %d", ADDR_WIDTH);
    $display("Simulation done.");
    $finish;
  end

endmodule