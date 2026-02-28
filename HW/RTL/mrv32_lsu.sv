module mrv32_lsu (
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic                  mem_valid,
    input  logic                  mem_ren,
    input  logic                  mem_wen,
    input  logic [3:0]            mem_wstrb,
    input  logic [2:0]            load_funct3,
    input  logic [31:0]           eff_addr,
    input  logic [31:0]           store_data,

    output logic                  lsu_done,
    //output logic [31:0]           load_data_q,       // registered, stable after response
    output logic [31:0]           load_data,  // combinational, valid same cycle as lsu_done

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
  // ----------------------------
  logic ld_misaligned;

  always @* begin
    ld_misaligned = 1'b1;
    case (load_funct3)
      3'b000, 3'b100: ld_misaligned = 1'b0;
      3'b001, 3'b101: ld_misaligned = eff_addr[0];
      3'b010:         ld_misaligned = (eff_addr[1:0] != 2'b00);
      default:        ld_misaligned = 1'b1;
    endcase
  end

  // ----------------------------
  // FSM: IDLE -> ISSUE -> WAIT_RD
  // ----------------------------
  typedef enum logic [1:0] {IDLE, ISSUE, WAIT_RD} state_t;
  state_t state, state_n;

  logic [31:0] addr_q;
  logic [2:0]  f3_q;

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
  // Valid on the same cycle lsu_done fires (WAIT_RD + b_rvalid)
  // Core must use this to latch into reg_mem_wb
  // ----------------------------
  always @* begin
    load_data = load_data_q; // default: hold registered value

    if (state == WAIT_RD && b_rvalid) begin
      logic [31:0] rshift_b;
      logic [31:0] rshift_h;
      logic [7:0]  ld_byte;
      logic [15:0] ld_half;

      rshift_b = b_rdata >> (8 * addr_q[1:0]);
      ld_byte  = rshift_b[7:0];
      rshift_h = b_rdata >> (8 * {addr_q[1], 1'b0});
      ld_half  = rshift_h[15:0];

      case (f3_q)
        3'b000: load_data = {{24{ld_byte[7]}}, ld_byte};
        3'b100: load_data = {24'd0, ld_byte};
        3'b001: load_data = {{16{ld_half[15]}}, ld_half};
        3'b101: load_data = {16'd0, ld_half};
        3'b010: load_data = b_rdata;
        default: load_data = 32'd0;
      endcase
    end
  end

  // ----------------------------
  // State + registered load_data
  // ----------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= IDLE;
      addr_q    <= 32'd0;
      f3_q      <= 3'd0;
      load_data_q <= 32'd0;
    end else begin
      state <= state_n;

      if (state == ISSUE && mem_ren && ram_hit && !ld_misaligned) begin
        addr_q <= eff_addr;
        f3_q   <= load_funct3;
      end

      if (state == ISSUE && mem_ren && (!ram_hit || ld_misaligned))
        load_data_q <= 32'd0;

      // Register the combinational result for stability after response cycle
      if (state == WAIT_RD && b_rvalid)
        load_data_q <= load_data;
    end
  end

endmodule