include ../Link/memmap.mk

MEMMAP_LDH := ../Link/memmap.ldh
MEMMAP_H   := include/mrv_memmap.h

CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra

.PHONY: all default clean

all: default

# Build ISS and ensure memmap header exists first
default: $(MEMMAP_H) $(MEMMAP_LDH)
	$(CXX) $(CXXFLAGS) -Iinclude *.cpp -o mrv_iss

# Generate linker include
$(MEMMAP_LDH): ../Link/memmap.mk
	@mkdir -p $(dir $@)
	@{ \
	  echo "/* Auto-generated from memmap.mk */"; \
	  echo "MRV_RAM_BASE  = $(MRV_RAM_BASE);"; \
	  echo "MRV_RAM_SIZE  = $(MRV_RAM_SIZE);"; \
	  echo ""; \
	  echo "MRV_MMIO_BASE   = $(MRV_MMIO_BASE);"; \
	  echo "MRV_UART_TX     = $(MRV_UART_TX);"; \
	  echo "MRV_UART_STATUS = $(MRV_UART_STATUS);"; \
	  echo "MRV_TOHOST      = $(MRV_TOHOST);"; \
	  echo "MRV_GPIO_OUT    = $(MRV_GPIO_OUT);"; \
	  echo "MRV_GPIO_IN     = $(MRV_GPIO_IN);"; \
	} > $@

# Generate C header
$(MEMMAP_H): ../Link/memmap.mk
	@mkdir -p $(dir $@)
	@{ \
	  echo "/* Auto-generated from memmap.mk */"; \
	  echo "#pragma once"; \
	  echo "#include <stdint.h>"; \
	  echo ""; \
	  echo "#define MRV_RAM_BASE     ($(MRV_RAM_BASE)u)"; \
	  echo "#define MRV_RAM_SIZE     ($(MRV_RAM_SIZE)u)"; \
	  echo ""; \
	  echo "#define MRV_MMIO_BASE    ($(MRV_MMIO_BASE)u)"; \
	  echo "#define MRV_UART_TX      ($(MRV_UART_TX)u)"; \
	  echo "#define MRV_UART_STATUS  ($(MRV_UART_STATUS)u)"; \
	  echo "#define MRV_TOHOST       ($(MRV_TOHOST)u)"; \
	  echo "#define MRV_GPIO_OUT     ($(MRV_GPIO_OUT)u)"; \
	  echo "#define MRV_GPIO_IN      ($(MRV_GPIO_IN)u)"; \
	} > $@

clean:
	rm -f mrv_iss
	rm -f $(MEMMAP_H)
	# remove memmap.ldh only if you want it generated here:
	# rm -f $(MEMMAP_LDH)
