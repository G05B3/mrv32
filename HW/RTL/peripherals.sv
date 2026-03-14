module sim_peripherals (
    input  logic        clk,
    input  logic        periph_valid,
    input  logic [31:0] periph_addr,
    input  logic [31:0] periph_wdata,
    output logic        tohost_fired
);

  localparam logic [31:0] UART_ADDR   = 32'h10000000;
  localparam logic [31:0] TOHOST_ADDR = 32'h10000010;

  initial tohost_fired = 1'b0;

  always @(posedge clk) begin
    if (periph_valid) begin
        //$display("periph @ %h called with data: %h", periph_addr, periph_wdata);
      case (periph_addr)

        UART_ADDR: begin
          $write("%c", periph_wdata[7:0]);
        end

        TOHOST_ADDR: begin
            $display("\n[TOHOST] exit code: %0d", periph_wdata);
            tohost_fired <= 1'b1;
        end

        default: begin
          $display("[PERIPH] unhandled write: addr=0x%08x data=0x%08x",
                   periph_addr, periph_wdata);
        end

      endcase
    end
  end

endmodule