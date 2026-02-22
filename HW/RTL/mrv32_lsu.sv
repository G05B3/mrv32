/*
 * MRV32 Load/Store Unit (LSU) - Bring-up Skeleton
 *
 * - Consumes 32-bit effective address from ALU (no address calculation here).
 * - Talks to dual_port_byte_mem Port B using valid/rvalid.
 * - RAM-only for now: addresses outside [0 .. MEM_BYTES-1] are ignored.
 * - Misaligned accesses are ignored for now (later: trap).
 *
 * Stores:
 *   - Uses mem_wstrb encoding from mrv32_pkg (WSTRB_B/H/W).
 *   - Lane shifting is implemented so SB/SH can be enabled later without refactor.
 *
 * Loads:
 *   - Request/response skeleton is implemented (waits for b_rvalid).
 *   - Extraction/extend is implemented based on load_funct3.
 *   - If ignored/unmapped/misaligned: returns 0.
 */

module mrv32_lsu (
    input  logic                  clk,
    input  logic                  rst_n,

    // Core request (MEM stage)
    input  logic                  mem_valid,      // Valid if either read or write; in the wrapper define this logic
    input  logic                  mem_ren,
    input  logic                  mem_wen,
    input  logic [3:0]            mem_wstrb,      // WSTRB_B/H/W
    input  logic [2:0]            load_funct3,    // 000 LB,001 LH,010 LW,100 LBU,101 LHU
    input  logic [31:0]           eff_addr,       // ALU result
    input  logic [31:0]           store_data,     // rs2

    // Core response
    output logic                  lsu_done,       // 1-cycle pulse
    output logic [31:0]           load_data,      // valid when a load completes (else 0)

    // Memory Port B
    output logic                  b_valid,
    output logic [ADDR_WIDTH-1:0] b_addr,
    output logic [31:0]           b_wdata,
    output logic [3:0]            b_wstrb,
    input  logic [31:0]           b_rdata,
    input  logic                  b_rvalid
);
  import mrv32_pkg::*;

  // RAM-only map, base = 0
  logic ram_hit;
  always @* begin
    ram_hit = (eff_addr < MEM_BYTES);
  end

  assign b_addr = eff_addr[ADDR_WIDTH-1:0];

  // ----------------------------
  // Store formatting (SB/SH/SW)
  // ----------------------------
  logic [31:0] st_wdata;
  logic [3:0]  st_wstrb;
  logic        st_misaligned;

  always @* begin
    st_wdata      = 32'd0;
    st_wstrb      = WSTRB_NONE;
    st_misaligned = 1'b1;

    case (mem_wstrb)
      WSTRB_B: begin
        st_misaligned = 1'b0;
        case (eff_addr[1:0])
          2'd0: begin st_wdata = {24'd0, store_data[7:0]};               st_wstrb = 4'b0001; end
          2'd1: begin st_wdata = {16'd0, store_data[7:0], 8'd0};         st_wstrb = 4'b0010; end
          2'd2: begin st_wdata = { 8'd0, store_data[7:0], 16'd0};        st_wstrb = 4'b0100; end
          2'd3: begin st_wdata = {       store_data[7:0], 24'd0};        st_wstrb = 4'b1000; end
        endcase
      end

      WSTRB_H: begin
        st_misaligned = eff_addr[0];
        if (!eff_addr[0]) begin
          case (eff_addr[1])
            1'b0: begin st_wdata = {16'd0, store_data[15:0]};            st_wstrb = 4'b0011; end
            1'b1: begin st_wdata = {      store_data[15:0], 16'd0};      st_wstrb = 4'b1100; end
          endcase
        end
      end

      WSTRB_W: begin
        st_misaligned = (eff_addr[1:0] != 2'b00);
        st_wdata      = store_data;
        st_wstrb      = WSTRB_W;
      end

      default: begin
        // illegal / none => keep defaults (treated as ignored)
      end
    endcase
  end

  // ----------------------------
  // Load alignment check
  // ----------------------------
  logic ld_misaligned;

  always @* begin
    ld_misaligned = 1'b1;
    case (load_funct3)
      3'b000, 3'b100: ld_misaligned = 1'b0;                 // LB/LBU
      3'b001, 3'b101: ld_misaligned = eff_addr[0];          // LH/LHU
      3'b010:         ld_misaligned = (eff_addr[1:0]!=2'b00);// LW
      default:        ld_misaligned = 1'b1;
    endcase
  end

  // ----------------------------
  // FSM
  // ----------------------------
  typedef enum logic [1:0] {IDLE, ISSUE, WAIT_RD} state_t;
  state_t state, state_n;

  // Latched metadata for loads (used when response returns)
  logic [31:0] addr_q;
  logic [2:0]  f3_q;

  // Extracted pieces from returned word
  logic [31:0] rshift_b;
  logic [31:0] rshift_h;
  logic [7:0]  ld_byte;
  logic [15:0] ld_half;

  // Outputs default
  always @* begin
    b_valid  = 1'b0;
    b_wdata  = 32'd0;
    b_wstrb  = WSTRB_NONE;
    lsu_done = 1'b0;

    state_n  = state;

    case (state)
      IDLE: begin
        if (mem_valid) state_n = ISSUE;
      end

      ISSUE: begin
        // Unmapped => ignore and complete
        if (!ram_hit) begin
          lsu_done = 1'b1;
          state_n  = IDLE;

        end else if (mem_wen) begin
          // Store => enqueue if aligned/legal, then complete immediately
          if (!st_misaligned && (st_wstrb != WSTRB_NONE)) begin
            b_valid = 1'b1;
            b_wdata = st_wdata;
            b_wstrb = st_wstrb;
          end
          lsu_done = 1'b1;
          state_n  = IDLE;
          //$display("STORE: addr=%h data=%h wen=%b, valid=%b", b_addr, b_wdata, mem_wen, b_valid);

        end else if (mem_ren) begin
          // Load => if misaligned ignore, else enqueue and wait
          if (ld_misaligned) begin
            lsu_done = 1'b1;
            state_n  = IDLE;
          end else begin
            b_valid = 1'b1;       // read request
            b_wstrb = WSTRB_NONE; // read
            state_n = WAIT_RD;
          end

        end else begin
          // no-op
          lsu_done = 1'b1;
          state_n  = IDLE;
        end
      end

      WAIT_RD: begin
        if (b_rvalid) begin
          lsu_done = 1'b1;
          state_n  = IDLE;
        end
      end
    endcase
  end

  // State + load_data registers
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= IDLE;
      addr_q    <= 32'd0;
      f3_q      <= 3'd0;
      load_data <= 32'd0;
    end else begin
      state <= state_n;

      // Latch metadata when issuing a load that will wait
      if (state == ISSUE && mem_valid && mem_ren && ram_hit && !ld_misaligned) begin
        addr_q <= eff_addr;
        f3_q   <= load_funct3;
      end

      // For ignored loads, return 0
      if (state == ISSUE && mem_valid && mem_ren && (!ram_hit || ld_misaligned)) begin
        load_data <= 32'd0;
      end

      // On response, extract/extend
      if (state == WAIT_RD && b_rvalid) begin
        // byte extract
        rshift_b = (b_rdata >> (8*addr_q[1:0]));
        ld_byte  = rshift_b[7:0];

        // half extract (aligned to 0 or 2)
        rshift_h = (b_rdata >> (8*{addr_q[1],1'b0}));
        ld_half  = rshift_h[15:0];

        case (f3_q)
          3'b000: load_data <= {{24{ld_byte[7]}}, ld_byte};   // LB
          3'b100: load_data <= {24'd0, ld_byte};              // LBU
          3'b001: load_data <= {{16{ld_half[15]}}, ld_half};  // LH
          3'b101: load_data <= {16'd0, ld_half};              // LHU
          3'b010: load_data <= b_rdata;                       // LW
          default: load_data <= 32'd0;
        endcase
      end
    end
  end

endmodule