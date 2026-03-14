//==============================================================================
// Package: mrv32_pkg v1.0
//------------------------------------------------------------------------------
// Description:
//   Global definitions for the MRV32 core.
//
// Contents:
//   - Opcode encodings
//   - ALU operation encodings
//   - Immediate selectors
//   - LSU width encodings
//   - Architectural constants (XLEN, register count, etc.)
//
// Notes:
//   Centralizes ISA-related definitions to ensure consistency
//   across all modules.
//
// Author: Martim Bento
// Date  : 01/03/2026
//==============================================================================

package mrv32_pkg;

  // ----------------------------
  // RV32I opcodes (instr[6:0])
  // ----------------------------
  localparam logic [6:0] OPCODE_LUI    = 7'b0110111;
  localparam logic [6:0] OPCODE_AUIPC  = 7'b0010111;
  localparam logic [6:0] OPCODE_JAL    = 7'b1101111;
  localparam logic [6:0] OPCODE_JALR   = 7'b1100111;
  localparam logic [6:0] OPCODE_BTYPE  = 7'b1100011;
  localparam logic [6:0] OPCODE_LOAD   = 7'b0000011;
  localparam logic [6:0] OPCODE_STYPE  = 7'b0100011;
  localparam logic [6:0] OPCODE_ITYPE  = 7'b0010011;
  localparam logic [6:0] OPCODE_RTYPE  = 7'b0110011;
  localparam logic [6:0] OPCODE_SYSTEM = 7'b1110011; // ecall/ebreak/csrs (future)

  // ----------------------------
  // ALU op encoding
  // ----------------------------
  localparam logic [3:0]
      ALU_ADD  = 4'd0,
      ALU_SUB  = 4'd1,
      ALU_AND  = 4'd2,
      ALU_OR   = 4'd3,
      ALU_XOR  = 4'd4,
      ALU_SLL  = 4'd5,
      ALU_SRL  = 4'd6,
      ALU_SRA  = 4'd7,
      ALU_SLT  = 4'd8,
      ALU_SLTU = 4'd9;

  // ----------------------------
  // Immediate selector
  // ----------------------------
  localparam logic [2:0]
      IMM_I = 3'd0,
      IMM_S = 3'd1,
      IMM_B = 3'd2,
      IMM_U = 3'd3,
      IMM_J = 3'd4;

  localparam logic [1:0]
      BR_NONE   = 2'b00, // not a branch (default)
      BR_ALWAYS = 2'b01, // unconditional jump (JAL/JALR)
      BR_EQLT   = 2'b10, // branch (BEQ, BLT, BLTU)
      BR_NQLT   = 2'b11; // branch (BNE, BGE, BGEU)

  // ----------------------------
  // Write strobe helpers (byte enables)
  // ----------------------------
  localparam logic [3:0] WSTRB_NONE = 4'b0000;
  localparam logic [3:0] WSTRB_B    = 4'b0001; // store byte (lane select handled elsewhere)
  localparam logic [3:0] WSTRB_H    = 4'b0011; // store halfword (lane select handled elsewhere)
  localparam logic [3:0] WSTRB_W    = 4'b1111; // store word

  localparam MEM_BYTES  = 1024 * 1024;
  localparam ADDR_WIDTH = $clog2(MEM_BYTES);

  // ----------------------------
  // Common widths / constants
  // ----------------------------
  localparam int unsigned XLEN    = 32;
  localparam int unsigned REGADDR = 5;

endpackage : mrv32_pkg
//==============================================================================
// Module: mrv32_fetch v1.1
//------------------------------------------------------------------------------
// Description:
//   Instruction Fetch stage for pipelined execution.
//
// Behavior:
//   - Continuously fetches instructions, advancing PC each cycle.
//   - PC advances by 4 each cycle unless take_branch is asserted,
//     in which case PC jumps to branch_target.
//   - Outputs NOP (0x00000013) and instr_valid=0 while waiting for
//     memory to return data or after a branch redirect.
//   - instr_valid=1 only when a_rvalid returns a real instruction.
//
// Notes:
//   Branch flush (squashing in-flight instructions) is not yet implemented.
//   Pipeline stages behind the branch will need to be invalidated separately.
//
// Interfaces:
//   - Memory request/response handshake (always fetching)
//   - take_branch / branch_target from MEM stage for PC redirect
//
// Author: Martim Bento
// Date  : 08/03/2026
//==============================================================================

import mrv32_pkg::*;

module mrv32_fetch (
    input  logic                  clk,
    input  logic                  rst_n,

    // Memory port A (instruction side)
    output logic                  a_valid,
    output logic [ADDR_WIDTH-1:0] a_addr,
    output logic [31:0]           a_wdata,
    output logic [3:0]            a_wstrb,
    input  logic [31:0]           a_rdata,
    input  logic                  a_rvalid,

    // Branch redirect from MEM stage
    input  logic                  take_branch,
    input  logic [31:0]           branch_target,

    // Stall from hazard unit
    input  logic                  stall,

    // Output to IF/ID stage register
    output logic [31:0]           instr,
    output logic [31:0]           pc,
    output logic                  instr_valid
);

  localparam NOP = 32'h00000013; // ADDI x0, x0, 0

  logic [31:0] pc_fetch; // PC of the instruction currently being fetched

  // PC register — advances every cycle unless stalled
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pc_fetch <= 32'd0;
    else if (!stall) begin
      if (take_branch)
        pc_fetch <= branch_target;
      else
        pc_fetch <= pc_fetch + 32'd4;
    end
  end

  // Always request a fetch
  assign a_valid = 1'b1;
  assign a_addr  = pc_fetch[ADDR_WIDTH-1:0];
  assign a_wdata = 32'd0;
  assign a_wstrb = 4'b0000;

  // Output to IF/ID register
  // pc_fetch is registered so it trails the fetch address by one cycle,
  // matching the cycle when a_rvalid returns
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      instr       <= NOP;
      pc          <= 32'd0;
      instr_valid <= 1'b0;
    end else if (!stall) begin
      instr       <= a_rvalid ? a_rdata : NOP;
      pc          <= pc_fetch;
      instr_valid <= a_rvalid;
    end
  end

endmodule//==============================================================================
// Module: mrv32_imm_gen v1.0
//------------------------------------------------------------------------------
// Description:
//   Immediate value generator for RV32I instructions.
//
// Supported Formats:
//   - I-type
//   - S-type
//   - B-type
//   - U-type
//   - J-type
//
// Notes:
//   Immediate format selected via imm_sel control from decode stage.
//   Outputs properly sign-extended 32-bit immediate.
//
// Author: Martim Bento
// Date  : 01/03/2026
//==============================================================================

module mrv32_imm_gen (
    input  logic [31:0] instr,
    input  logic [2:0]  imm_sel,
    output logic [31:0] imm
);
  import mrv32_pkg::*;

  // Precompute all immediates (pure wiring)
  wire [31:0] imm_i = {{20{instr[31]}}, instr[31:20]};
  wire [31:0] imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  wire [31:0] imm_b = {{19{instr[31]}}, instr[31], instr[7],
                       instr[30:25], instr[11:8], 1'b0};
  wire [31:0] imm_u = {instr[31:12], 12'b0};
  wire [31:0] imm_j = {{11{instr[31]}}, instr[31], instr[19:12],
                       instr[20], instr[30:21], 1'b0};

  // Select immediate
  assign imm =
      (imm_sel == IMM_I) ? imm_i :
      (imm_sel == IMM_S) ? imm_s :
      (imm_sel == IMM_B) ? imm_b :
      (imm_sel == IMM_U) ? imm_u :
      (imm_sel == IMM_J) ? imm_j :
                           32'd0;
endmodule//==============================================================================
// Module: mrv32_decode v1.0
//------------------------------------------------------------------------------
// Description:
//   Instruction decode stage for RV32I subset.
//
// Responsibilities:
//   - Decode opcode/funct3/funct7
//   - Generate ALU control signals
//   - Select immediate format
//   - Generate LSU control signals
//   - Identify branch and jump instructions
//
// Output:
//   - Control signals for EX, MEM, WB
//   - Immediate value
//   - Register file addresses
//
// Notes:
//   Designed to be compatible with future fully pipelined operation.
//
// Author: Martim Bento
// Date  : 01/03/2026
//==============================================================================

module mrv32_decode(
    input  logic [31:0] instr,
    output logic [4:0]  rs1_addr,
    output logic [4:0]  rs2_addr,
    output logic [4:0]  rd_addr,

    output logic [3:0]  aluop,
    output logic        alusrc,      // 1 => use immediate for op2
    output logic        mem_ren,
    output logic        mem_wen,
    output logic [3:0]  mem_wstrb,
    output logic        load_unsigned,
    output logic        reg_wen,

    output logic        is_lui,
    output logic        is_auipc,
    output logic        is_jalr,
    output logic [1:0]  br_sel,

    output logic [31:0] imm,
    output logic        unsupported_instr
);
  import mrv32_pkg::*;

  logic [6:0] opcode, funct7;
  logic [2:0] funct3;
  logic [2:0] imm_sel;
  logic [1:0] funct3_upper;
  logic funct3_lower;

    assign opcode   = instr[6:0];
    assign rd_addr  = instr[11:7];
    assign funct3   = instr[14:12];
    assign rs1_addr = instr[19:15];
    assign rs2_addr = instr[24:20];
    assign funct7   = instr[31:25];
    assign funct3_upper = funct3[2:1];
    assign funct3_lower = funct3[0] ~^ funct3[2];

  always_comb begin

    // Defaults
    aluop            = ALU_ADD;
    alusrc           = 1'b0;
    imm_sel          = IMM_I;
    mem_ren          = 1'b0;
    mem_wen          = 1'b0;
    mem_wstrb        = 4'b0000;
    load_unsigned    = 1'b0;
    reg_wen          = 1'b0;
    is_lui           = 1'b0;
    is_auipc         = 1'b0;
    is_jalr          = 1'b0;
    br_sel           = BR_NONE;
    unsupported_instr = 1'b0;

    case (opcode)
        OPCODE_LUI: begin // LUI
            is_lui  = 1'b1;
            reg_wen = (rd_addr != 5'd0);
            imm_sel = IMM_U;
        end

        OPCODE_AUIPC: begin // AUIPC
            is_auipc = 1'b1;
            reg_wen  = (rd_addr != 5'd0);
            imm_sel  = IMM_U;
        end

        OPCODE_ITYPE: begin // I-TYPE (IMM OPs)
            alusrc = 1'b1;
            reg_wen = (rd_addr != 5'd0);
            imm_sel = IMM_I;
            case (funct3)
            3'b000: aluop = ALU_ADD; // ADDI
            3'b001: aluop = ALU_SLL; // SLLI
            3'b010: aluop = ALU_SLT; // SLTI
            3'b011: aluop = ALU_SLTU; // SLTIU
            3'b100: aluop = ALU_XOR; // XORI
            3'b101: begin
                if (funct7 == 7'b0000000) aluop = ALU_SRL; // SRLI
                else if (funct7 == 7'b0100000) aluop = ALU_SRA; // SRAI
                else // Unsupported Instruction
                begin
                    reg_wen = 1'b0;
                    unsupported_instr = 1'b1;               
                end
            end
            3'b110: aluop = ALU_OR; // ORI
            3'b111: aluop = ALU_AND; // ANDI
            default: begin
                reg_wen = 1'b0;
                unsupported_instr = 1'b1;
            end
            endcase
        end

        OPCODE_RTYPE: begin // R-TYPE
            reg_wen = (rd_addr != 5'd0);
            case (funct3)
            3'b000: begin
                if (funct7 == 7'b0000000) aluop = ALU_ADD;
                else if (funct7 == 7'b0100000) aluop = ALU_SUB;
                else begin
                reg_wen = 1'b0;
                unsupported_instr = 1'b1;
                end
            end
            3'b001: aluop = ALU_SLL; // SLL
            3'b010: aluop = ALU_SLT; // SLT
            3'b011: aluop = ALU_SLTU; // SLTU
            3'b100: aluop = ALU_XOR; // XOR
            3'b101: begin
                if (funct7 == 7'b0000000) aluop = ALU_SRL; // SRL
                else if (funct7 == 7'b0100000) aluop = ALU_SRA; // SRA
                else // Unsupported Instruction
                begin
                    reg_wen = 1'b0;
                    unsupported_instr = 1'b1;               
                end
            end
            3'b110: aluop = ALU_OR; // OR
            3'b111: aluop = ALU_AND; // AND
            default: begin
                reg_wen = 1'b0;
                unsupported_instr = 1'b1;
            end
            endcase
        end

        OPCODE_LOAD: begin /// I-TYPE (LOAD)
           alusrc = 1'b1;
           mem_ren = 1'b1;
           reg_wen = (rd_addr != 5'd0);
           imm_sel = IMM_I;
           aluop = ALU_ADD;
           case (funct3)
           3'b000: mem_wstrb = WSTRB_B; // LB
           3'b001: mem_wstrb = WSTRB_H; // LH
           3'b010: mem_wstrb = WSTRB_W; // LW
           3'b100: begin mem_wstrb = WSTRB_B; load_unsigned = 1'b1; end // LBU
           3'b101: begin mem_wstrb = WSTRB_H; load_unsigned = 1'b1; end// LHU
           default: begin
            mem_ren = 1'b0;
            unsupported_instr = 1'b1;
           end
           endcase 
        end

        OPCODE_STYPE: begin // S-TYPE (STORE)
            alusrc = 1'b1;
            mem_wen = 1'b1;
            imm_sel = IMM_S;
            aluop = ALU_ADD;
            case (funct3)
            3'b000: mem_wstrb = WSTRB_B; // SB
            3'b001: mem_wstrb = WSTRB_H; // SH
            3'b010: mem_wstrb = WSTRB_W; // SW
            default: begin
                mem_wen = 1'b0;
                unsupported_instr = 1'b1;
            end
            endcase
        end

        OPCODE_BTYPE: begin // B-TYPE (BRANCHES)
            alusrc = 1'b0;
            imm_sel = IMM_B;
            br_sel = (funct3_lower) ? BR_EQLT : BR_NQLT; // BNE/BEQ, BLT/BGE, BLTU/BGEU
            case(funct3_upper)
            2'b00: aluop = ALU_SUB;  // BEQ  (000) or BNE  (001)
            2'b10: aluop = ALU_SLT;  // BLT  (100) or BGE  (101)
            2'b11: aluop = ALU_SLTU; // BLTU (110) or BGEU (111)
            default: begin
                unsupported_instr = 1'b1;
            end
            endcase
        end

        OPCODE_JAL: begin // JAL
            br_sel = BR_ALWAYS;
            imm_sel = IMM_J;
            reg_wen = (rd_addr != 5'd0);
        end

        OPCODE_JALR: begin // JALR
            is_jalr = 1'b1;
            br_sel = BR_ALWAYS;
            imm_sel = IMM_I;
            reg_wen = (rd_addr != 5'd0);
            alusrc = 1'b1; // op with IMM
            aluop = ALU_ADD; // target addr = rs1 + imm
        end

        default: begin
            unsupported_instr = 1'b1;
        end
    endcase
  end

  // Immediate generator
  mrv32_imm_gen immgen (
    .instr(instr),
    .imm_sel(imm_sel),
    .imm(imm)
  );

endmodule//==============================================================================
// Module: mrv32_alu v1.0
//------------------------------------------------------------------------------
// Description:
//   RV32I arithmetic logic unit.
//
// Supported Operations:
//   - ADD / SUB
//   - AND / OR / XOR
//   - SLT / SLTU
//   - Shift left/right (logical and arithmetic)
//
// Notes:
//   Operation selected via alu_op control from decode stage.
//   ALU result is used for arithmetic, branch comparison, and address
//   generation.
//
// Author: Martim Bento
// Date  : 01/03/2026
//==============================================================================

module mrv32_alu (
    input  logic [31:0] op1,
    input  logic [31:0] op2,
    input  logic [3:0]  aluop,
    output logic [31:0] result
);
  import mrv32_pkg::*;

  wire [4:0] shamt = op2[4:0];  // extract shift amount outside always block

  always_comb begin
    case (aluop)
      ALU_ADD: result = op1 + op2;
      ALU_SUB: result = op1 - op2;
      ALU_AND: result = op1 & op2;
      ALU_OR:  result = op1 | op2;
      ALU_XOR: result = op1 ^ op2;
      ALU_SLL:  result = op1 << shamt;
      ALU_SRL:  result = op1 >> shamt;
      ALU_SRA:  result = 32'($signed(op1) >>> shamt);
      ALU_SLT:  result = {31'd0, $signed(op1) < $signed(op2)};
      ALU_SLTU: result = {31'd0, op1 < op2};
      default: result = 32'd0; // Default case for unsupported operations
    endcase
  end

endmodule//==============================================================================
// Module: mrv32_bru v1.0
//------------------------------------------------------------------------------
// Description:
//   Branch resolution unit.
//
// Function:
//   - Evaluates branch conditions using ALU result flags
//   - Generates take_branch signal
//
// Supported Branches:
//   - BEQ, BNE
//   - BLT, BGE
//   - BLTU, BGEU
//
//   - JAL, JALR (Unconditional Branches)
//
// Notes:
//   Branch resolution currently occurs in MEM stage.
//
// Author: Martim Bento
// Date  : 01/03/2026
//==============================================================================

module mrv32_bru (

    input logic [1:0] br_sel,
    input logic [31:0] alu_result,
    output logic take_branch

);

    logic is_Zero;
    assign is_Zero = alu_result == 32'd0;

    always_comb begin
        case (br_sel)
            2'b00: take_branch = 1'b0; // not a branch
            2'b01: take_branch = 1'b1; // unconditional jump (JAL/JALR)
            2'b10: take_branch = is_Zero ? 1'b1 : 1'b0; // branch (BEQ, BGE, BGEU)
            2'b11: take_branch = is_Zero ? 1'b0 : 1'b1; // branch (BNE, BLT, BLTU)
            default: take_branch = 1'b0;
        endcase
    end

endmodule//==============================================================================
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
// Date  : 01/03/2026
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

endmodule//==============================================================================
// Module: mrv32_periph_decoder v1.0
//------------------------------------------------------------------------------
// Description:
//   Peripheral address decoder for the MRV32 core.
//
// Responsibilities:
//   - Detect store operations targeting the peripheral address range
//   - Forward address and write data to the peripheral bus
//   - Assert periph_valid for exactly one cycle per peripheral store
//
// Notes:
//   Peripheral address range is parameterizable via PERIPH_BASE and PERIPH_END.
//   Module is intentionally agnostic to pipeline internals — qualification
//   of mem_wen with lsu_done is handled at the instantiation site in mrv32_core.
//   Reads from peripheral addresses are not supported in this version.
//
// Author: Martim Bento
// Date  : 07/03/2026
//==============================================================================

module mrv32_periph_decoder #(
    parameter logic [31:0] PERIPH_BASE,
    parameter logic [31:0] PERIPH_END
) (
    input logic mem_wen,
    input logic [31:0] eff_addr,
    input logic [31:0] store_data,
    
    output logic periph_valid,
    output logic [31:0] periph_addr,
    output logic [31:0] periph_wdata
);

assign periph_valid = mem_wen & (eff_addr >= PERIPH_BASE && eff_addr < PERIPH_END);
assign periph_addr = eff_addr;
assign periph_wdata = store_data;

endmodule//==============================================================================
// Module: mrv32_wb v1.1
//------------------------------------------------------------------------------
// Description:
//   Writeback stage of the RV32I core.
//
// Responsibilities:
//   - Select final writeback data (ALU, memory, immediate, PC+4)
//   - Generate register file write enable and data
//   - Compute next PC value
//   - Assert instr_accept to allow next instruction fetch
//
// Notes:
//   PC update occurs in this stage.
//   Only one instruction is retired at a time in bring-up mode.
//   Designed to support future pipelined execution.
//
// Author: Martim Bento
// Date  : 08/03/2026
//==============================================================================

module mrv32_wb (
    // WB-stage valid token
    input  logic        wb_valid,

    // Decoded/latched control for this instruction
    input  logic        reg_wen_in,
    input  logic        mem_ren_in,

    input  logic        take_branch,

    input  logic [4:0]  rd_addr_in,

    // Latched data inputs
    input  logic [31:0] alu_result_in,
    input  logic [31:0] load_data_in,
    input  logic [31:0] pc_in,         // PC of the retiring instruction

    // Outputs to register file write port
    output logic        rf_wen,
    output logic [4:0]  rf_waddr,
    output logic [31:0] rf_wdata,

    // Outputs to fetch / core control
    output logic        instr_accept   // commit pulse
);

  // default outputs
  always_comb begin
    rf_wen      = 1'b0;
    rf_waddr    = rd_addr_in;
    rf_wdata    = 32'd0;

    instr_accept = 1'b0;

    if (wb_valid) begin
      // Commit this instruction (blocking core)
      instr_accept = 1'b1;

      // Writeback mux
      if (take_branch) begin
        rf_wdata = pc_in + 32'd4;
      end else if (mem_ren_in) begin
        rf_wdata = load_data_in;
      end else begin
        rf_wdata = alu_result_in;
      end

      // Register write enable gating
      rf_wen = reg_wen_in && (rd_addr_in != 5'd0);
    end
  end

endmodule/** Hazard Detection Unit **/

module mrv32_hzdu (
    input logic reg_wen_ex,
    input logic [4:0] rd_ex, // alu_result_ex
    input logic reg_wen_mem,
    input logic [4:0] rd_mem, // alu_result_mem
    input logic [4:0] rs1_id,
    input logic [4:0] rs2_id,
    input logic rs2_id_used,
    input logic mem_ren_ex,
    output logic fwd_rs1_ex,
    output logic fwd_rs2_ex,
    output logic fwd_rs1_mem,
    output logic fwd_rs2_mem,
    output logic load_stall
);

assign fwd_rs1_ex = (rd_ex == rs1_id) & reg_wen_ex & (rd_ex != 0);
assign fwd_rs2_ex = (rd_ex == rs2_id) & reg_wen_ex & (rd_ex != 0) & rs2_id_used;

assign fwd_rs1_mem = (rd_mem == rs1_id) & reg_wen_mem & (rd_mem != 0);
assign fwd_rs2_mem = (rd_mem == rs2_id) & reg_wen_mem & (rd_mem != 0) & rs2_id_used;

assign load_stall = mem_ren_ex & ((rd_ex == rs1_id) | ((rd_ex == rs2_id) & rs2_id_used)) & (rd_ex != 0);

endmodule//==============================================================================
// Module: mrv32_regfile v1.0
//------------------------------------------------------------------------------
// Description:
//   32 x 32-bit register file.
//
// Features:
//   - 2 read ports
//   - 1 write port
//   - x0 hardwired to zero
//
// Notes:
//   Writes occur in WB stage.
//   Designed to support forwarding in future revisions.
//
// Author: Martim Bento
// Date  : 01/03/2026
//==============================================================================

module mrv32_regfile (

    input logic clk,
    input logic rst_n,
    input logic [4:0] rs1_addr,
    input logic [4:0] rs2_addr,
    input logic [4:0] rd_addr,
    input logic [31:0] rd_data,
    input logic reg_wen,
    output logic [31:0] rs1_data,
    output logic [31:0] rs2_data
);

logic [31:0] registers [0: 30]; // 31 real registers, x1 - x31; x0 is hardwired to 0

// Read Logic
always_comb begin
    rs1_data = (rs1_addr == 5'b00000) ? 32'b0 : registers[rs1_addr - 1];
    rs2_data = (rs2_addr == 5'b00000) ? 32'b0 : registers[rs2_addr - 1];
end

// Write Logic
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < 31; i++) begin
            registers[i] <= 32'b0;
        end
    end else if (reg_wen && rd_addr != 5'b00000) begin
        registers[rd_addr - 1] <= rd_data;
    end
end


endmodule//==============================================================================
// Module: mrv32_core v1.1
//------------------------------------------------------------------------------
// Description:
//   Single-issue RV32I core with staged pipeline registers.
//   Only one instruction is allowed in flight at a time (serialized execution).
//
//   The design maintains IF/ID/EX/MEM/WB stage registers to provide a clean
//   structural foundation for future conversion into a fully pipelined
//   5-stage processor with hazard detection and forwarding.
//
// Architecture:
//   - RV32I subset (no FENCE, ECALL, EBREAK or CSR instructions)
//   - Branch resolved in MEM stage
//   - PC updated in WB stage (single instruction in flight)
//
// Future Extensions:
//   - Enable overlapping execution (true 5-stage pipeline)
//   - Add hazard detection unit
//   - Add forwarding paths
//   - Add pipeline flush on branch
//
// Author: Martim Bento
// Date  : 01/03/2026
//==============================================================================

import mrv32_pkg::*;

module mrv32_core #(
    parameter logic [31:0] PERIPH_BASE = 32'h10000000,
    parameter logic [31:0] PERIPH_END = 32'h10001000
) (
    input logic clk,
    input logic rst_n,

    output logic                  a_valid,
    output logic [ADDR_WIDTH-1:0] a_addr,
    output logic [31:0]           a_wdata,
    output logic [3:0]            a_wstrb,
    input  logic [31:0]           a_rdata,
    input  logic                  a_rvalid,
    
    output logic                  b_valid,
    output logic [ADDR_WIDTH-1:0] b_addr,
    output logic [31:0]           b_wdata,
    output logic [3:0]            b_wstrb,
    input  logic [31:0]           b_rdata,
    input  logic                  b_rvalid,

    // Peripherals
    output logic periph_valid,
    output logic [31:0] periph_addr,
    output logic [31:0] periph_wdata,
    
    output logic illegal_instr
);

logic [31:0] instr_if, instr;
logic [31:0] pc_if, pc_id, pc_ex, pc_mem, pc_wb, pc_next;
logic instr_valid_fetch, iv_if_id, instr_valid_decode;
logic instr_accept;

// From WB back to RF
logic rf_wen;
logic [4:0] rf_waddr;
logic [31:0] rf_wdata;

/** Fetch **/
mrv32_fetch fetch(.clk(clk), .rst_n(rst_n), .a_rvalid(a_rvalid), .a_valid(a_valid), .a_addr(a_addr),
                .a_wdata(a_wdata), .a_wstrb(a_wstrb), .a_rdata(a_rdata), .instr(instr_if), .pc(pc_if),
                .instr_valid(instr_valid_fetch), .branch_target(jal_target), .take_branch(take_branch), .stall(stall));

logic [64:0] reg_if_id;
// Stage Register between IF and ID
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_if_id <= 0;
    else if (!stall)
        reg_if_id <= {instr_if, pc_if, instr_valid_fetch};
end

assign instr = reg_if_id[64:33];
assign pc_id = reg_if_id[32:1];
assign iv_if_id = reg_if_id[0];


logic [4:0] rs1_addr;
logic [4:0] rs2_addr;
logic [4:0] rd_addr;
logic [3:0] aluop;
logic alusrc; // 1 => use immediate for op2

logic mem_ren, mem_wen, mem_valid;
logic [3:0] mem_wstrb;
logic reg_wen;
logic is_lui, is_auipc, is_jalr, unsupported;
logic [1:0] br_sel, br_sel_ex, br_sel_mem;

logic [31:0] imm;

/** Decode **/
mrv32_decode decode(.instr(instr), .rs1_addr(rs1_addr), .rs2_addr(rs2_addr), .rd_addr(rd_addr), .load_unsigned(load_unsigned),
                    .aluop(aluop), .alusrc(alusrc), .mem_ren(mem_ren), .mem_wen(mem_wen), .mem_wstrb(mem_wstrb), .br_sel(br_sel),
                    .reg_wen(reg_wen), .is_lui(is_lui), .is_auipc(is_auipc), .is_jalr(is_jalr), .imm(imm), .unsupported_instr(unsupported));

assign instr_valid_decode = iv_if_id & ~unsupported;
assign mem_valid = mem_ren | mem_wen; // if either load or store then it's a mem op
assign illegal_instr = unsupported & iv_if_id;

logic load_unsigned, load_unsigned_ex, is_jalr_ex;
logic [4:0] rs1_addr_ex, rs2_addr_ex, rd_addr_ex;
logic [3:0] aluop_ex;
logic [3:0] mem_wstrb_ex;
logic alusrc_ex, mem_ren_ex, mem_wen_ex, reg_wen_ex, is_lui_ex, is_auipc_ex, mem_valid_ex, instr_valid_ex;
logic [31:0] imm_ex;

logic [98:0] reg_id_ex;
// Stage Register between ID and EX
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_id_ex <= 0;
    else if (!stall)
        reg_id_ex <= {br_sel, is_jalr, is_auipc, pc_id, instr_valid_decode,
        load_unsigned, rs1_addr, rs2_addr, rd_addr, aluop, alusrc, mem_ren, mem_wen, mem_wstrb, reg_wen, is_lui, imm,
        mem_valid};
end

assign br_sel_ex = reg_id_ex[98:97];
assign is_jalr_ex = reg_id_ex[96];
assign is_auipc_ex = reg_id_ex[95];
assign pc_ex = reg_id_ex[94:63];
assign instr_valid_ex = reg_id_ex[62];
assign load_unsigned_ex = reg_id_ex[61];
assign rs1_addr_ex  = reg_id_ex[60:56];
assign rs2_addr_ex  = reg_id_ex[55:51];
assign rd_addr_ex   = reg_id_ex[50:46];
assign aluop_ex     = reg_id_ex[45:42];
assign alusrc_ex    = reg_id_ex[41];
assign mem_ren_ex   = reg_id_ex[40];
assign mem_wen_ex   = reg_id_ex[39];
assign mem_wstrb_ex = reg_id_ex[38:35];
assign reg_wen_ex   = reg_id_ex[34];
assign is_lui_ex    = reg_id_ex[33];
assign imm_ex       = reg_id_ex[32:1];
assign mem_valid_ex = reg_id_ex[0];


logic [31:0] rs1_data, rs2_data, op2, op1, alu_result;

mrv32_regfile mrv_rf(.clk(clk), .rst_n(rst_n), .rs1_addr(rs1_addr_ex), .rs2_addr(rs2_addr_ex), .rd_addr(rf_waddr),
                    .rd_data(rf_wdata), .reg_wen(rf_wen), .rs1_data(rs1_data), .rs2_data(rs2_data));

// RS1 may be forwarded from EX or MEM
assign op1 = is_lui_ex  ? 32'd0   :
             is_auipc_ex ? pc_ex  :
                           rs1_fwd;

// RS2 may be forwared from EX or MEM
assign op2 = (is_lui_ex | is_auipc_ex) ? imm_ex :
             alusrc_ex                 ? imm_ex  :
                                         rs2_fwd;

/** ALU (EX) **/
mrv32_alu mrv_alu(.op1(op1), .op2(op2), .aluop(aluop_ex), .result(alu_result));


logic [4:0] rd_addr_mem;
logic mem_ren_mem, mem_wen_mem, mem_valid_mem, reg_wen_mem, instr_valid_mem;
logic [3:0] mem_wstrb_mem;
logic [31:0] alu_result_mem, imm_mem, rs2_mem;
logic load_unsigned_mem, is_jalr_mem;
logic [145:0] reg_ex_mem;
// Stage Register between EX and MEM
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_ex_mem <= 0;
    else if (!stall)
        reg_ex_mem <= {br_sel_ex, is_jalr_ex, rs2_fwd, pc_ex, imm_ex, instr_valid_ex,
        load_unsigned_ex, rd_addr_ex, mem_ren_ex, mem_wen_ex, mem_wstrb_ex, reg_wen_ex, mem_valid_ex,
        alu_result};
end

assign br_sel_mem = reg_ex_mem[145:144];
assign is_jalr_mem = reg_ex_mem[143];
assign rs2_mem = reg_ex_mem[142:111];
assign pc_mem = reg_ex_mem[110:79];
assign imm_mem = reg_ex_mem[78:47];
assign instr_valid_mem = reg_ex_mem[46];
assign load_unsigned_mem = reg_ex_mem[45];
assign rd_addr_mem = reg_ex_mem[44:40];
assign mem_ren_mem = reg_ex_mem[39];
assign mem_wen_mem = reg_ex_mem[38];
assign mem_wstrb_mem = reg_ex_mem[37:34];
assign reg_wen_mem = reg_ex_mem[33];
assign mem_valid_mem = reg_ex_mem[32];
assign alu_result_mem = reg_ex_mem[31:0];


logic take_branch, take_branch_wb;
mrv32_bru mrv_bru(.br_sel(br_sel_mem), .alu_result(alu_result_mem), .take_branch(take_branch));


// Jump target, wired from MEM back to Fetch
logic [31:0] jal_target;
assign jal_target = is_jalr_mem ? (alu_result_mem & ~32'd1) : (pc_mem + imm_mem);

logic lsu_done;
logic [31:0] load_data;

/** LSU (MEM) **/
mrv32_lsu lsu(.clk(clk), .rst_n(rst_n), .b_valid(b_valid), .b_addr(b_addr), .b_wdata(b_wdata), .b_wstrb(b_wstrb),
              .b_rdata(b_rdata), .b_rvalid(b_rvalid), .mem_valid(mem_valid_mem), .mem_ren(mem_ren_mem),
              .mem_wen(mem_wen_mem), .mem_wstrb(mem_wstrb_mem), .load_unsigned(load_unsigned_mem), .eff_addr(alu_result_mem),
              .lsu_done(lsu_done), .load_data(load_data), .store_data(rs2_mem));

/** Peripheral Decoder (within MEM stage) **/
mrv32_periph_decoder #(
    .PERIPH_BASE(PERIPH_BASE),
    .PERIPH_END (PERIPH_END)
) periph_dec (
    .mem_wen    (mem_wen_mem & lsu_done),
    .eff_addr   (alu_result_mem),
    .store_data (rs2_mem),
    .periph_valid(periph_valid),
    .periph_addr (periph_addr),
    .periph_wdata(periph_wdata)
);


logic stall; // 1 => stall the pipeline

assign stall = mem_valid_mem & !lsu_done;

logic [4:0] rd_addr_wb;
logic instr_valid_wb, reg_wen_wb, mem_ren_wb;
logic [31:0] alu_result_wb, load_data_wb;

logic true_instr_valid;
assign true_instr_valid = instr_valid_mem & (!mem_valid_mem | lsu_done);

logic [104:0] reg_mem_wb;
// Stage Register between MEM and WB
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_mem_wb <= 0;
    else if (!stall)
        reg_mem_wb <= {take_branch, pc_mem, mem_ren_mem, true_instr_valid, rd_addr_mem, reg_wen_mem, alu_result_mem, load_data};
end

assign take_branch_wb = reg_mem_wb[104];
assign pc_wb = reg_mem_wb[103:72];
assign mem_ren_wb = reg_mem_wb[71];
assign instr_valid_wb = reg_mem_wb[70];
assign rd_addr_wb = reg_mem_wb[69:65];
assign reg_wen_wb = reg_mem_wb[64];
assign alu_result_wb = reg_mem_wb[63:32];
assign load_data_wb = reg_mem_wb[31:0];

// instr_accept: debug signal, to count accepted instructions. TODO: Add CSR with minstret and wire this there instead

/** Write Back **/
mrv32_wb wb(.wb_valid(instr_valid_wb), .reg_wen_in(reg_wen_wb), .mem_ren_in(mem_ren_wb), .take_branch(take_branch_wb),
            .rd_addr_in(rd_addr_wb), .alu_result_in(alu_result_wb), .load_data_in(load_data_wb),
            .pc_in(pc_wb), .instr_accept(instr_accept), .rf_wen(rf_wen), .rf_waddr(rf_waddr), .rf_wdata(rf_wdata));


logic fwd_rs1_ex, fwd_rs2_ex, fwd_rs1_mem, fwd_rs2_mem, load_stall;

/** Hazard Detection Unit **/
mrv32_hzdu hzdu (
    .reg_wen_ex  (reg_wen_mem),    // producer one stage ahead of EX = MEM
    .rd_ex       (rd_addr_mem),
    .reg_wen_mem (reg_wen_wb),     // producer two stages ahead = WB
    .rd_mem      (rd_addr_wb),
    .rs1_id      (rs1_addr_ex),    // consumer in EX
    .rs2_id      (rs2_addr_ex),    // consumer in EX
    .rs2_id_used (!alusrc_ex | mem_wen_ex),
    .mem_ren_ex  (mem_ren_mem),
    .fwd_rs1_ex  (fwd_rs1_ex),
    .fwd_rs2_ex  (fwd_rs2_ex),
    .fwd_rs1_mem (fwd_rs1_mem),
    .fwd_rs2_mem (fwd_rs2_mem),
    .load_stall  (load_stall)
);

/** Forwarding Connections **/
logic [31:0] rs1_fwd, rs2_fwd, fwd_val_ex, fwd_val_mem;

// fwd_rs1_ex => producer is in MEM => use br_sel_mem
assign fwd_val_ex  = (br_sel_mem == BR_ALWAYS) ? pc_mem + 32'd4 : alu_result_mem;

// fwd_rs1_mem => producer is in WB => rf_wdata already correct
assign fwd_val_mem = rf_wdata;

assign rs1_fwd = fwd_rs1_ex  ? fwd_val_ex  :
                 fwd_rs1_mem ? fwd_val_mem  :
                               rs1_data;

assign rs2_fwd = fwd_rs2_ex  ? fwd_val_ex  :
                 fwd_rs2_mem ? fwd_val_mem  :
                               rs2_data;

endmodule;