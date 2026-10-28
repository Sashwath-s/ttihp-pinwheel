# Pinwheel PDN for Tiny Tapeout IHP sg13cmos5l with the RM_IHPSG13_1P_512x16 SRAM macro.
#
# Based on LibreLane's default pdn_cfg.tcl and on urish/ttihp-sram-test (SG13G2).
#
# On CMOS5L, Tiny Tapeout limits user blocks to Metal1-Metal4 (RT_MAX_LAYER=Metal4) and
# owns TopMetal1, so the standard-cell stripes are on Metal4 -- the same layer as the
# SRAM's VDD!/VDDARRAY!/VSS! pins (vertical straps). A macro grid can therefore never
# create a via (an empty grid is fatal: PDN-0232/0233), and pins exported to the top
# level leave the macro disconnected inside the block (PSM-0069).
#
# Instead the macro is placed so that ordinary standard-cell stripes run straight over
# same-net straps: one VPWR stripe on a full-height VDD! strap, one VGND stripe on a
# full-height VSS! strap, and one VPWR stripe that enters from above onto a VDDARRAY!
# strap. PDN_OBSTRUCTIONS in src/config.json cut every other stripe over the macro.
# The stripes continue into the core and reach the rails above and below the SRAM.
# The geometry is verified against the macro LEF by scripts/check_tt_config.py.

source $::env(SCRIPTS_DIR)/openroad/common/set_global_connections.tcl
set_global_connections

set secondary []
foreach vdd $::env(VDD_NETS) gnd $::env(GND_NETS) {
    if { $vdd != $::env(VDD_NET)} {
        lappend secondary $vdd

        set db_net [[ord::get_db_block] findNet $vdd]
        if {$db_net == "NULL"} {
            set net [odb::dbNet_create [ord::get_db_block] $vdd]
            $net setSpecial
            $net setSigType "POWER"
        }
    }

    if { $gnd != $::env(GND_NET)} {
        lappend secondary $gnd

        set db_net [[ord::get_db_block] findNet $gnd]
        if {$db_net == "NULL"} {
            set net [odb::dbNet_create [ord::get_db_block] $gnd]
            $net setSpecial
            $net setSigType "GROUND"
        }
    }
}

set_voltage_domain -name CORE -power $::env(VDD_NET) -ground $::env(GND_NET) \
    -secondary_power $secondary

# Standard-cell grid: Metal4 vertical stripes only (FP_PDN_MULTILAYER=0), promoted to pins
define_pdn_grid \
    -name stdcell_grid \
    -starts_with POWER \
    -voltage_domain CORE \
    -pins $::env(PDN_VERTICAL_LAYER)

add_pdn_stripe \
    -grid stdcell_grid \
    -layer $::env(PDN_VERTICAL_LAYER) \
    -width $::env(PDN_VWIDTH) \
    -pitch $::env(PDN_VPITCH) \
    -offset $::env(PDN_VOFFSET) \
    -spacing $::env(PDN_VSPACING) \
    -starts_with POWER -extend_to_core_ring

# Standard-cell rails on Metal1, stacked up to the Metal4 stripes
if { $::env(PDN_ENABLE_RAILS) == 1 } {
    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(PDN_RAIL_LAYER) \
        -width $::env(PDN_RAIL_WIDTH) \
        -followpins

    add_pdn_connect \
        -grid stdcell_grid \
        -layers "$::env(PDN_RAIL_LAYER) $::env(PDN_VERTICAL_LAYER)"
}

# No macro grid: see the header.
