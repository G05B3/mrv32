//==============================================================================
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
// Date  : 28/02/2026
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


endmodule