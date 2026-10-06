//==============================================================================
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

endmodule