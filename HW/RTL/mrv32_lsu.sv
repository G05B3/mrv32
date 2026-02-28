//==============================================================================
// Module: mrv32_lsu v1.0
//------------------------------------------------------------------------------
// Description:
//   Load-Store Unit for RV32I.
//
// Features:
//   - Supports LB/LH/LW/LBU/LHU
//   - Supports SB/SH/SW
//   - Byte-enable generation
//   - Sign/zero extension
//   - Alignment checking
//   - Blocking memory interface
//
// Operation:
//   - Multi-cycle FSM (IDLE → ISSUE → WAIT_RD)
//   - Asserts lsu_done when memory operation completes
//
// Notes:
//   This unit stalls the entire core during memory transactions.
//
// Author: Martim Bento
// Date  : 28/02/2026
//==============================================================================

module mrv32_lsu (
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic                  mem_valid,
    input  logic                  mem_ren,
    input  logic                  mem_wen,
    input  logic [3:0]            mem_wstrb,
    input  logic                  load_unsigned, // 1=zero-extend, 0=sign-extend
    input  logic [31:0]           eff_addr,
    input  logic [31:0]           store_data,

    output logic                  lsu_done,
    output logic [31:0]           load_data,

    output logic                  b_valid,
    output logic [ADDR_WIDTH-1:0] b_addr,
    output logic [31:0]           b_wdata,
    output logic [3:0]            b_wstrb,
    input  logic [31:0]           b_rdata,
    input  logic                  b_rvalid
);
  import mrv32_pkg::*;

  logic ram_hit;
  logic [31:0] load_data_q;
  always @* ram_hit = (eff_addr < MEM_BYTES);

  assign b_addr = {eff_addr[ADDR_WIDTH-1:2], 2'b00};  // word-align memory access

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
          2'd0: begin st_wdata = {24'd0, store_data[7:0]};        st_wstrb = 4'b0001; end
          2'd1: begin st_wdata = {16'd0, store_data[7:0],  8'd0}; st_wstrb = 4'b0010; end
          2'd2: begin st_wdata = { 8'd0, store_data[7:0], 16'd0}; st_wstrb = 4'b0100; end
          2'd3: begin st_wdata = {store_data[7:0], 24'd0};        st_wstrb = 4'b1000; end
        endcase
      end
      WSTRB_H: begin
        st_misaligned = eff_addr[0];
        if (!eff_addr[0]) begin
          case (eff_addr[1])
            1'b0: begin st_wdata = {16'd0, store_data[15:0]}; st_wstrb = 4'b0011; end
            1'b1: begin st_wdata = {store_data[15:0], 16'd0}; st_wstrb = 4'b1100; end
          endcase
        end
      end
      WSTRB_W: begin
        st_misaligned = (eff_addr[1:0] != 2'b00);
        st_wdata      = store_data;
        st_wstrb      = WSTRB_W;
      end
      default: ;
    endcase
  end

  // ----------------------------
  // Load alignment check
  // Uses mem_wstrb to determine access size
  // ----------------------------
  logic ld_misaligned;

  always @* begin
    case (mem_wstrb)
      WSTRB_B:   ld_misaligned = 1'b0;
      WSTRB_H:   ld_misaligned = eff_addr[0];
      WSTRB_W:   ld_misaligned = (eff_addr[1:0] != 2'b00);
      default:   ld_misaligned = 1'b1;
    endcase
  end

  // ----------------------------
  // FSM: IDLE -> ISSUE -> WAIT_RD
  // ----------------------------
  typedef enum logic [1:0] {IDLE, ISSUE, WAIT_RD} state_t;
  state_t state, state_n;

  logic [31:0] addr_q;
  logic [3:0]  wstrb_q;      // latched size for extraction
  logic        unsigned_q;   // latched sign control

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
        if (!ram_hit) begin
          lsu_done = 1'b1;
          state_n  = IDLE;
        end else if (mem_wen) begin
          if (!st_misaligned && (st_wstrb != WSTRB_NONE)) begin
            b_valid = 1'b1;
            b_wdata = st_wdata;
            b_wstrb = st_wstrb;
          end
          lsu_done = 1'b1;
          state_n  = IDLE;
        end else if (mem_ren) begin
          if (ld_misaligned) begin
            lsu_done = 1'b1;
            state_n  = IDLE;
          end else begin
            b_valid = 1'b1;
            b_wstrb = WSTRB_NONE;
            state_n = WAIT_RD;
          end
        end else begin
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

      default: state_n = IDLE;
    endcase
  end

  // ----------------------------
  // Combinational load data output
  // ----------------------------
  always @* begin
    load_data = load_data_q;

    if (state == WAIT_RD && b_rvalid) begin
      logic [7:0]  ld_byte;
      logic [15:0] ld_half;

      ld_byte = b_rdata >> (8 * addr_q[1:0]);
      ld_half = b_rdata >> (8 * {addr_q[1], 1'b0});

      case (wstrb_q)
        WSTRB_B: load_data = unsigned_q ? {24'd0, ld_byte} : {{24{ld_byte[7]}},  ld_byte};
        WSTRB_H: load_data = unsigned_q ? {16'd0, ld_half} : {{16{ld_half[15]}}, ld_half};
        WSTRB_W: load_data = b_rdata;
        default: load_data = 32'd0;
      endcase
    end
  end

  // ----------------------------
  // State + registered load_data
  // ----------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= IDLE;
      addr_q     <= 32'd0;
      wstrb_q    <= WSTRB_NONE;
      unsigned_q <= 1'b0;
      load_data_q <= 32'd0;
    end else begin
      state <= state_n;

      if (state == ISSUE && mem_ren && ram_hit && !ld_misaligned) begin
        addr_q     <= eff_addr;
        wstrb_q    <= mem_wstrb;
        unsigned_q <= load_unsigned;
      end

      if (state == ISSUE && mem_ren && (!ram_hit || ld_misaligned))
        load_data_q <= 32'd0;

      if (state == WAIT_RD && b_rvalid)
        load_data_q <= load_data;
    end
  end

endmodule