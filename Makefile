# =============================================================================
# Makefile – Verilator simulation build
# =============================================================================

RTL_DIR  := rtl
SIM_DIR  := sim
OBJ_DIR  := obj_dir

RTL_SRCS := \
    $(RTL_DIR)/sdp_bram.sv                   \
    $(RTL_DIR)/projection_unit.sv            \
    $(RTL_DIR)/smooth_grad_unit.sv           \
    $(RTL_DIR)/bounds_calc.sv                \
    $(RTL_DIR)/prewitt_edge_unit.sv          \
    $(RTL_DIR)/complexity_unit.sv            \
    $(RTL_DIR)/drowsiness_detection_top.sv

TOP      := drowsiness_detection_top
SIM_MAIN := $(SIM_DIR)/sim_main.cpp
STB_HDR  := $(SIM_DIR)/stb_image.h $(SIM_DIR)/stb_image_write.h

IMG ?= test.jpg

# =============================================================================
.PHONY: all
all: $(OBJ_DIR)/V$(TOP)

$(OBJ_DIR)/V$(TOP): $(RTL_SRCS) $(SIM_MAIN) $(STB_HDR)
	verilator \
	    --cc \
	    --exe \
	    --build \
	    --sv \
	    -Wall \
	    -Wno-UNUSED \
	    -Wno-UNDRIVEN \
	    --top-module $(TOP) \
	    -I$(RTL_DIR) \
	    -I$(SIM_DIR) \
	    -CFLAGS "-std=c++17 -O2 -I$(SIM_DIR)" \
	    $(RTL_SRCS) \
	    $(SIM_MAIN) \
	    -o V$(TOP)

# =============================================================================
.PHONY: trace
trace: $(RTL_SRCS) $(SIM_MAIN) $(STB_HDR)
	verilator \
	    --cc \
	    --exe \
	    --build \
	    --sv \
	    --trace \
	    -Wall \
	    -Wno-UNUSED \
	    -Wno-UNDRIVEN \
	    --top-module $(TOP) \
	    -I$(RTL_DIR) \
	    -I$(SIM_DIR) \
	    -CFLAGS "-std=c++17 -O2 -I$(SIM_DIR) -DTRACE_ENABLE" \
	    $(RTL_SRCS) \
	    $(SIM_MAIN) \
	    -o V$(TOP)

# =============================================================================
.PHONY: run
run: all
	$(OBJ_DIR)/V$(TOP) $(IMG)

# Download stb headers if missing
$(STB_HDR):
	bash $(SIM_DIR)/get_stb.sh

.PHONY: clean
clean:
	rm -rf $(OBJ_DIR)
