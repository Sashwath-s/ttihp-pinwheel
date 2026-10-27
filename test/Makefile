# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info
# (Tiny Tapeout cmos5l template; PROJECT_SOURCES updated for Pinwheel)

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
# RTL tests run without PW_MEM_IHP_SRAM (flop memory), so the macro black box is unused here
PROJECT_SOURCES = tt_um_pinwheel.v pw_top.v pw_core.v pw_lane.v pw_host_spi.v pw_mem.v pw_fifo.v RM_IHPSG13_1P_512x16_c2_bm_bist.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_io/verilog/sg13cmos5l_io.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_udp.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_stdcell.v
# Pinwheel memory macro: vendored behavioural model (FUNCTIONAL variant), same files as the GDS/LEF
VERILOG_SOURCES += $(PWD)/../macro/RM_IHPSG13_1P_512x16_c2_bm_bist/RM_IHPSG13_1P_core_behavioral_bm_bist.v
VERILOG_SOURCES += $(PWD)/../macro/RM_IHPSG13_1P_512x16_c2_bm_bist/RM_IHPSG13_1P_512x16_c2_bm_bist.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
