import mrv32_pkg::*;

module dual_port_byte_mem #(
    parameter integer RD_LATENCY = 2,
    parameter int MEM_BYTES = 1024*1024,
    parameter int ADDR_WIDTH = $clog2(MEM_BYTES)
) (
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic                  a_valid,
    input  logic [ADDR_WIDTH-1:0] a_addr,
    input  logic [31:0]           a_wdata,
    input  logic [3:0]            a_wstrb,
    output logic [31:0]           a_rdata,
    output logic                  a_rvalid,

    input  logic                  b_valid,
    input  logic [ADDR_WIDTH-1:0] b_addr,
    input  logic [31:0]           b_wdata,
    input  logic [3:0]            b_wstrb,
    output logic [31:0]           b_rdata,
    output logic                  b_rvalid
);

    logic [7:0] mem [0:MEM_BYTES-1];

    // ---------------- Port A (instruction) ----------------
    logic [ADDR_WIDTH-1:0] a_addr_pipe [0:RD_LATENCY-1];
    logic                  a_valid_pipe [0:RD_LATENCY-1];

    integer ai;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (ai = 0; ai < RD_LATENCY; ai = ai + 1) begin
                a_addr_pipe[ai]  <= '0;
                a_valid_pipe[ai] <= 1'b0;
            end
        end else begin
            a_addr_pipe[0]  <= a_addr;
            a_valid_pipe[0] <= a_valid;
            for (ai = 1; ai < RD_LATENCY; ai = ai + 1) begin
                a_addr_pipe[ai]  <= a_addr_pipe[ai-1];
                a_valid_pipe[ai] <= a_valid_pipe[ai-1];
            end
            // writes on port A (unused normally, instruction port is read-only)
            if (a_valid && a_wstrb != WSTRB_NONE) begin
                if (a_wstrb[0]) mem[a_addr+0] <= a_wdata[7:0];
                if (a_wstrb[1]) mem[a_addr+1] <= a_wdata[15:8];
                if (a_wstrb[2]) mem[a_addr+2] <= a_wdata[23:16];
                if (a_wstrb[3]) mem[a_addr+3] <= a_wdata[31:24];
            end
        end
    end

    logic [31:0] a_word;
    assign a_word = {mem[a_addr_pipe[RD_LATENCY-1]+3], mem[a_addr_pipe[RD_LATENCY-1]+2],
                      mem[a_addr_pipe[RD_LATENCY-1]+1], mem[a_addr_pipe[RD_LATENCY-1]+0]};

    assign a_rdata  = a_word;
    assign a_rvalid = a_valid_pipe[RD_LATENCY-1];

    // ---------------- Port B (data) ----------------
    logic [ADDR_WIDTH-1:0] b_addr_pipe [0:RD_LATENCY-1];
    logic                  b_valid_pipe [0:RD_LATENCY-1];
    logic                  b_is_write_pipe [0:RD_LATENCY-1];

    integer bi;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (bi = 0; bi < RD_LATENCY; bi = bi + 1) begin
                b_addr_pipe[bi]     <= '0;
                b_valid_pipe[bi]    <= 1'b0;
                b_is_write_pipe[bi] <= 1'b0;
            end
        end else begin
            b_addr_pipe[0]     <= b_addr;
            b_valid_pipe[0]    <= b_valid;
            b_is_write_pipe[0] <= b_valid && (b_wstrb != WSTRB_NONE);
            for (bi = 1; bi < RD_LATENCY; bi = bi + 1) begin
                b_addr_pipe[bi]     <= b_addr_pipe[bi-1];
                b_valid_pipe[bi]    <= b_valid_pipe[bi-1];
                b_is_write_pipe[bi] <= b_is_write_pipe[bi-1];
            end
            if (b_valid && b_wstrb != WSTRB_NONE) begin
                if (b_wstrb[0]) mem[b_addr+0] <= b_wdata[7:0];
                if (b_wstrb[1]) mem[b_addr+1] <= b_wdata[15:8];
                if (b_wstrb[2]) mem[b_addr+2] <= b_wdata[23:16];
                if (b_wstrb[3]) mem[b_addr+3] <= b_wdata[31:24];
            end
        end
    end

    logic [31:0] b_word;
    assign b_word = {mem[b_addr_pipe[RD_LATENCY-1]+3], mem[b_addr_pipe[RD_LATENCY-1]+2],
                      mem[b_addr_pipe[RD_LATENCY-1]+1], mem[b_addr_pipe[RD_LATENCY-1]+0]};

    assign b_rdata  = b_word;
    assign b_rvalid = b_valid_pipe[RD_LATENCY-1] && !b_is_write_pipe[RD_LATENCY-1];

    // Preload
    initial begin
        for (int k = 0; k < MEM_BYTES; k++) mem[k] = 8'h00;
    end

endmodule