/*
 * MRV32 Core Package
 *
 * Shared constants, encodings, and architectural parameters for the MRV32
 * RV32I-compatible CPU core.
 *
 * This package defines:
 *   - RV32I instruction opcode encodings (instr[6:0])
 *   - ALU operation encodings used by decode and execute stages
 *   - Immediate format selectors for the immediate generator
 *   - Write strobe (byte enable) constants for memory stores
 *   - Common architectural constants (XLEN, register address width)
 *
 * The purpose of this package is to centralize all cross-module encodings
 * and architectural constants to ensure consistency between RTL blocks
 * such as:
 *   - instruction decode
 *   - ALU / execute stage
 *   - immediate generator
 *   - load/store unit
 *
 * All RTL modules that rely on these definitions should import this package:
 *
 *   import mrv32_pkg::*;
 *
 * This package contains no logic and is fully synthesizable.
 */

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
/*
 * MRV32 Instruction Fetch (IF)
 *
 * Works with dual_port_byte_mem Port A:
 *   - Request: a_valid=1 for one cycle, a_wstrb=0 (read)
 *   - Response: a_rvalid indicates a_rdata is valid (32-bit little-endian word)
 *
 * This IF is suitable for a simple multi-cycle core (one instruction in flight).
 * PC update is controlled externally by the core via pc_set/pc_next, so later
 * stages can decide the next PC (e.g., JAL/branch resolution).
 */

import mrv32_pkg::*;

module instr_fetch (
    input  logic                  clk,
    input  logic                  rst_n,

    // Memory port A (instruction side)
    output logic                  a_valid,
    output logic [ADDR_WIDTH-1:0] a_addr,
    output logic [31:0]           a_wdata,
    output logic [3:0]            a_wstrb,
    input  logic [31:0]           a_rdata,
    input  logic                  a_rvalid,

    // Control from core
    input  logic [31:0]           pc_next,    // next PC value (byte address)

    // Output to core (latched instruction)
    output logic [31:0]           instr,
    output logic [31:0]           pc,
    output logic                  instr_valid, // 1 when instr holds a valid fetched instruction
    input  logic                  instr_accept // core pulses when it consumed instr (update PC)
);

  typedef enum logic [1:0] { IF_IDLE, IF_REQ, IF_WAIT, IF_HAVE } if_state_t;
  if_state_t state, state_n;

  logic [31:0] pc_n;
  logic [31:0] instr_n;
  logic        instr_valid_n;

  // Memory read is always (wstrb==0)
  assign a_wdata = 32'd0;
  assign a_wstrb = 4'b0000;

  // Address is current PC truncated to memory address width (byte addressing)
  assign a_addr  = pc[ADDR_WIDTH-1:0];

  // State / registers
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state       <= IF_IDLE;
      pc          <= 32'd0;
      instr       <= 32'd0;
      instr_valid <= 1'b0;
    end else begin
      state       <= state_n;
      pc          <= pc_n;
      instr       <= instr_n;
      instr_valid <= instr_valid_n;
    end
  end

  // Next-state / outputs
  always_comb begin
    // defaults
    state_n       = state;
    pc_n          = pc;
    instr_n       = instr;
    instr_valid_n = instr_valid;

    a_valid       = 1'b0;

    // PC update from core (commit)
    if (instr_accept) begin
      pc_n = pc_next;
    end

    case (state)
      IF_IDLE: begin
        // Start by requesting first fetch immediately after reset release
        state_n = IF_REQ;
      end

      IF_REQ: begin
        // Enqueue a fetch request (one-cycle pulse)
        a_valid = 1'b1;
        state_n = IF_WAIT;
      end

      IF_WAIT: begin
        // Wait for memory to return instruction
        if (a_rvalid) begin
          instr_n       = a_rdata;
          instr_valid_n = 1'b1;
          state_n       = IF_HAVE;
        end
      end

      IF_HAVE: begin
        // Hold instruction stable until core accepts it
        if (instr_accept) begin
          state_n       = IF_REQ; // request next instruction (PC is updated by core via pc_set)
        end
        instr_valid_n = 1'b0;
        instr_n = 0;
      end

      default: begin
        state_n = IF_IDLE;
      end
    endcase
  end

endmodule/*
 * MRV32 Immediate Generator (RV32I)
 *
 * Generates a sign-extended immediate value from a 32-bit RISC-V instruction,
 * based on the immediate format selected by the decoder.
 *
 * Supported immediate formats:
 *   - I-type : instr[31:20]
 *   - S-type : {instr[31:25], instr[11:7]}
 *   - B-type : {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}
 *   - U-type : instr[31:12] << 12
 *   - J-type : {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}
 *
 * All immediates are sign-extended to 32 bits as defined by the RV32I ISA.
 *
 * Notes:
 * - This module is purely combinational.
 * - No instruction decoding is performed here; the decoder is responsible
 *   for selecting the correct immediate format via imm_sel.
 * - The module is reusable across single-cycle, multi-cycle, and pipelined
 *   implementations of the MRV32 core.
 */

module imm_gen (
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
endmodule/*
 * MRV32 Instruction Decoder (RV32I Subset)
 *
 * Decodes a 32-bit RISC-V instruction and produces:
 *   - Register specifiers: rs1, rs2, rd
 *   - ALU control: aluop + alusrc (selects imm vs rs2 as operand 2)
 *   - Memory control: mem_ren, mem_wen, mem_wstrb (byte enables)
 *   - Writeback control: reg_wen
 *   - A sign-extended immediate value (imm) via the imm_gen module
 *   - unsupported_instr flag for illegal/unsupported encodings
 *
 * Currently supported instructions (bring-up subset):
 *   - LUI
 *   - OP-IMM: ADDI
 *   - OP: ADD, SUB
 *   - STORE: SW
 *   - JAL
 *
 * Notes:
 * - This module is purely combinational (no internal state).
 * - x0 handling (write suppression) is performed by reg_wen gating (rd != 0);
 *   the register file must still enforce x0=0 for safety.
 * - Immediate selection (imm_sel) is internal; imm_gen performs format
 *   extraction and sign-extension.
 * - For unsupported instructions, unsupported_instr is asserted and all other
 *   control outputs remain in their safe default (inactive) states.
 */

module instr_decode(
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
    assign funct3_lower = funct3[0];

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
            br_sel = (funct3_lower) ? BR_NQLT : BR_EQLT; // BNE/BEQ, BLT/BGE, BLTU/BGEU
            case(funct3_upper)
            2'b00: aluop = ALU_SUB;  // BEQ  (000) or BNE  (001)
            2'b10: aluop = ALU_SLT;  // BLT  (100) or BGE  (101)
            2'b11: aluop = ALU_SLTU; // BLTU (110) or BGEU (111)
            default: begin
                unsupported_instr = 1'b0;
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
  imm_gen immgen (
    .instr(instr),
    .imm_sel(imm_sel),
    .imm(imm)
  );

endmodule/*
 * MRV32 Register File (RV32I Integer RF)
 *
 * - 32 architectural registers (x0..x31), 32-bit each
 * - 2 read ports (rs1, rs2), combinational/asynchronous reads
 * - 1 write port (rd), synchronous write on rising clock edge
 * - x0 is hardwired to zero:
 *     * reads of x0 return 0
 *     * writes to x0 are ignored
 * - Active-low reset optionally clears registers to 0 (for bring-up/debug)
 *
 * Notes:
 * - This module implements the integer register file required by RV32I.
 * - Read-during-write behavior is tool/implementation dependent unless
 *   explicit bypass/forwarding is added.
 */

module registerFile (

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


endmodule/*
 * MRV32 Arithmetic Logic Unit (ALU)
 *
 * Combinational ALU used by the MRV32 RV32I core.
 *
 * Inputs:
 *   - op1, op2 : 32-bit operands
 *   - aluop    : operation selector (encoding defined in mrv32_pkg)
 *
 * Output:
 *   - result   : 32-bit operation result
 *
 * Notes:
 * - Purely combinational (no internal state).
 * - ALU operation encodings (ALU_ADD, ALU_SUB, etc.) are defined in mrv32_pkg
 *   and must be kept consistent with the decoder and execute stage.
 * - Unsupported operations return zero by default.
 */

module alu (
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

endmodulemodule mrv32_lsu (
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

endmodule/** MRV32 Branch Control Unit **/

module br_control (

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
            2'b10: take_branch = is_Zero ? 1'b1 : 1'b0; // branch (BEQ, BLT, BLTU)
            2'b11: take_branch = is_Zero ? 1'b0 : 1'b1; // branch (BNE, BGE, BGEU)
            default: take_branch = 1'b0;
        endcase
    end

endmodule/*
 * MRV32 Writeback (WB) Stage
 *
 * For the blocking (one-instruction-in-flight) MRV32 core.
 *
 * Responsibilities:
 *   - Select writeback data (rd_data) for the register file
 *   - Gate register writes (rf_wen) using wb_valid and reg_wen_in
 *   - Generate instr_accept (commit pulse) back to instruction fetch
 *   - Compute pc_next for instruction fetch (pc+4 or jump target)
 *
 * Supported WB sources (bring-up subset):
 *   - LUI   : rd_data = imm_in
 *   - JAL   : rd_data = pc_in + 4
 *   - LOAD  : rd_data = load_data_in
 *   - ALU   : rd_data = alu_result_in
 *
 * Notes:
 *   - This module is combinational; the wrapper should latch all *_in signals.
 *   - x0 write suppression is handled by the register file, but we also gate rf_wen
 *     with (rd_addr_in != 0) here for extra safety.
 */

module mrv32_wb (
    // WB-stage valid token
    input  logic        wb_valid,

    // Decoded/latched control for this instruction
    input  logic        reg_wen_in,
    input  logic        mem_ren_in,
    input  logic        is_lui_in,
    input  logic        is_auipc_in,
    input  logic        take_branch,

    input  logic [4:0]  rd_addr_in,

    // Latched data inputs
    input  logic [31:0] alu_result_in,
    input  logic [31:0] imm_in,
    input  logic [31:0] load_data_in,
    input  logic [31:0] pc_in,         // PC of the retiring instruction
    input  logic [31:0] jal_target_in,  // already computed target (pc + imm) in wrapper/EX

    // Outputs to register file write port
    output logic        rf_wen,
    output logic [4:0]  rf_waddr,
    output logic [31:0] rf_wdata,

    // Outputs to fetch / core control
    output logic        instr_accept,   // commit pulse
    output logic [31:0] pc_next         // next PC on commit
);

  // default outputs
  always_comb begin
    rf_wen      = 1'b0;
    rf_waddr    = rd_addr_in;
    rf_wdata    = 32'd0;

    instr_accept = 1'b0;
    pc_next      = pc_in + 32'd4;

    if (wb_valid) begin
      // Commit this instruction (blocking core)
      instr_accept = 1'b1;

      // Next PC: sequential unless JAL or JALR
      if (take_branch) pc_next = jal_target_in;

      // Writeback mux
      if (is_lui_in) begin
        rf_wdata = imm_in;
      end else if (is_auipc_in) begin
        rf_wdata = imm_in + pc_in;
      end else if (take_branch) begin
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

endmodule/*** Core Wrapper ***/

import mrv32_pkg::*;

module mrv32_core (
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
    
    output logic unsupported_instr
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
instr_fetch fetch(.clk(clk), .rst_n(rst_n), .a_rvalid(a_rvalid), .a_valid(a_valid), .a_addr(a_addr),
                .a_wdata(a_wdata), .a_wstrb(a_wstrb), .a_rdata(a_rdata), .instr(instr_if), .pc(pc_if), .pc_next(pc_next),
                .instr_valid(instr_valid_fetch), .instr_accept(instr_accept));

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
instr_decode decode(.instr(instr), .rs1_addr(rs1_addr), .rs2_addr(rs2_addr), .rd_addr(rd_addr), .load_unsigned(load_unsigned),
                    .aluop(aluop), .alusrc(alusrc), .mem_ren(mem_ren), .mem_wen(mem_wen), .mem_wstrb(mem_wstrb), .br_sel(br_sel),
                    .reg_wen(reg_wen), .is_lui(is_lui), .is_auipc(is_auipc), .is_jalr(is_jalr), .imm(imm), .unsupported_instr(unsupported));

assign instr_valid_decode = iv_if_id & ~unsupported;
assign mem_valid = mem_ren | mem_wen; // if either load or store then it's a mem op
assign unsupported_instr = unsupported & iv_if_id;

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


logic [31:0] rs1_data, rs2_data, op2, alu_result;

registerFile mrv_rf(.clk(clk), .rst_n(rst_n), .rs1_addr(rs1_addr_ex), .rs2_addr(rs2_addr_ex), .rd_addr(rf_waddr),
                    .rd_data(rf_wdata), .reg_wen(rf_wen), .rs1_data(rs1_data), .rs2_data(rs2_data));

assign op2 = alusrc_ex ? imm_ex : rs2_data;

/** ALU (EX) **/
alu mrv_alu(.op1(rs1_data), .op2(op2), .aluop(aluop_ex), .result(alu_result));


logic [4:0] rd_addr_mem;
logic mem_ren_mem, mem_wen_mem, mem_valid_mem, reg_wen_mem, is_lui_mem, is_auipc_mem, instr_valid_mem;
logic [3:0] mem_wstrb_mem;
logic [31:0] alu_result_mem, imm_mem, rs2_mem;
logic load_unsigned_mem, is_jalr_mem;
logic [147:0] reg_ex_mem;
// Stage Register between EX and MEM
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_ex_mem <= 0;
    else if (!stall)
        reg_ex_mem <= {br_sel_ex, is_jalr_ex, is_auipc_ex, rs2_data, pc_ex, imm_ex, instr_valid_ex,
        load_unsigned_ex, rd_addr_ex, mem_ren_ex, mem_wen_ex, mem_wstrb_ex, reg_wen_ex, is_lui_ex, mem_valid_ex,
        alu_result};
end

assign br_sel_mem = reg_ex_mem[147:146];
assign is_jalr_mem = reg_ex_mem[145];
assign is_auipc_mem = reg_ex_mem[144];
assign rs2_mem = reg_ex_mem[143:112];
assign pc_mem = reg_ex_mem[111:80];
assign imm_mem = reg_ex_mem[79:48];
assign instr_valid_mem = reg_ex_mem[47];
assign load_unsigned_mem = reg_ex_mem[46];
assign rd_addr_mem = reg_ex_mem[45:41];
assign mem_ren_mem = reg_ex_mem[40];
assign mem_wen_mem = reg_ex_mem[39];
assign mem_wstrb_mem = reg_ex_mem[38:35];
assign reg_wen_mem = reg_ex_mem[34];
assign is_lui_mem = reg_ex_mem[33];
assign mem_valid_mem = reg_ex_mem[32];
assign alu_result_mem = reg_ex_mem[31:0];


logic take_branch, take_branch_wb;
br_control mrv_bru(.br_sel(br_sel_mem), .alu_result(alu_result_mem), .take_branch(take_branch));


logic lsu_done;
logic [31:0] load_data;

/** LSU (MEM) **/
mrv32_lsu lsu(.clk(clk), .rst_n(rst_n), .b_valid(b_valid), .b_addr(b_addr), .b_wdata(b_wdata), .b_wstrb(b_wstrb),
              .b_rdata(b_rdata), .b_rvalid(b_rvalid), .mem_valid(mem_valid_mem), .mem_ren(mem_ren_mem),
              .mem_wen(mem_wen_mem), .mem_wstrb(mem_wstrb_mem), .load_unsigned(load_unsigned_mem), .eff_addr(alu_result_mem),
              .lsu_done(lsu_done), .load_data(load_data), .store_data(rs2_mem));

logic stall; // 1 => stall the pipeline

assign stall = mem_valid_mem & !lsu_done;

logic [4:0] rd_addr_wb;
logic instr_valid_wb, reg_wen_wb, is_lui_wb, is_auipc_wb, mem_ren_wb, is_jalr_wb;
logic [31:0] alu_result_wb, load_data_wb, imm_wb;

logic true_instr_valid;
assign true_instr_valid = instr_valid_mem & (!mem_valid_mem | lsu_done);

logic [139:0] reg_mem_wb;
// Stage Register between MEM and WB
always_ff @(posedge clk) begin
    if (!rst_n)
        reg_mem_wb <= 0;
    else if (!stall)
        reg_mem_wb <= {take_branch, is_jalr_mem, is_auipc_mem, pc_mem, imm_mem, mem_ren_mem, true_instr_valid, rd_addr_mem, reg_wen_mem, is_lui_mem, alu_result_mem, load_data};
end

assign take_branch_wb = reg_mem_wb[139];
assign is_jalr_wb = reg_mem_wb[138];
assign is_auipc_wb = reg_mem_wb[137];
assign pc_wb = reg_mem_wb[136:105];
assign imm_wb = reg_mem_wb[104:73];
assign mem_ren_wb = reg_mem_wb[72];
assign instr_valid_wb = reg_mem_wb[71];
assign rd_addr_wb = reg_mem_wb[70:66];
assign reg_wen_wb = reg_mem_wb[65];
assign is_lui_wb = reg_mem_wb[64];
assign alu_result_wb = reg_mem_wb[63:32];
assign load_data_wb = reg_mem_wb[31:0];

logic [31:0] jal_target;
assign jal_target = is_jalr_wb ? (alu_result_wb & ~32'd1) : (pc_wb + imm_wb);

/** Write Back **/
mrv32_wb wb(.wb_valid(instr_valid_wb), .reg_wen_in(reg_wen_wb), .mem_ren_in(mem_ren_wb), .is_lui_in(is_lui_wb),
            .is_auipc_in(is_auipc_wb), .take_branch(take_branch_wb),.rd_addr_in(rd_addr_wb), .alu_result_in(alu_result_wb), .load_data_in(load_data_wb),
            .imm_in(imm_wb), .pc_in(pc_wb), .jal_target_in(jal_target),
            .instr_accept(instr_accept), .pc_next(pc_next), .rf_wen(rf_wen), .rf_waddr(rf_waddr), .rf_wdata(rf_wdata));

endmodule;// =============================================================================
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