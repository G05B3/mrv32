iverilog -g2012 -o sim \
  ../RTL/mrv32_pkg.sv ../RTL/mrv32_alu.sv ../RTL/mrv32_fetch.sv ../RTL/mrv32_id.sv ../RTL/mrv32_rf.sv ../RTL/mrv32_imm_gen.sv \
  ../RTL/mrv32_lsu.sv \
  ../RTL/mem_dual_port.sv lsu_tb.sv

vvp sim +PROG=test1.hex +START=0x00000000 +LEN=256