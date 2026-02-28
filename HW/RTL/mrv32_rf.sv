/*
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


endmodule