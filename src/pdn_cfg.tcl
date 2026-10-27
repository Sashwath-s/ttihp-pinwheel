# Pinwheel PDN for Tiny Tapeout IHP sg13cmos5l with the RM_IHPSG13_1P_512x16 SRAM macro.
#
# Based on LibreLane's default pdn_cfg.tcl and on urish/ttihp-sram-test (SG13G2).
#
# Why this differs from the SG13G2 reference: on CMOS5L, Tiny Tapeout limits user
# blocks to Metal1-Metal4 (RT_MAX_LAYER=Metal4) and owns TopMetal1 for its top-level
# power grid. The SRAM blocks Metal1-Metal4 over its whole footprint, and its
# VDD!/VDDARRAY!/VSS! pins are vertical Metal4 straps, so there is no free layer to
# strap them inside this block (the reference connected Metal4 to TopMetal1 here).
#
# Instead every Metal4 power strap of the macro is exported as a VPWR/VGND block pin,
# exactly like the standard-cell Metal4 stripes. Tiny Tapeout's TopMetal1 grid then
# connects to the macro straps wherever it crosses them, the same way it connects to
# the stripes. The global connections (PDN_MACRO_CONNECTIONS) tie the macro supply
# pins to VPWR/VGND logically for LVS.
#
# VERIFY on the first GDS run: (1) the log line from pw_export_macro_power_pins,
# (2) LVS clean with the macro abstracted, (3) the block LEF lists the macro straps
# as VPWR/VGND pins. Confirm with the Tiny Tapeout team that their top-level grid
# connects to every Metal4 power pin of the block, not only to full-height stripes.

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

# No macro grid: on CMOS5L the stripe layer is the macro's pin layer (Metal4), so a
# macro grid can never create a via, and pdngen rejects the empty grid
# (PDN-0232/PDN-0233, first GDS run). The stripes are kept off the SRAM by
# PDN_OBSTRUCTIONS in src/config.json (macro + 10 um halo on Metal4) instead.

# Export the SRAM's Metal4 power straps as VPWR/VGND block pins (see header).
proc pw_export_macro_power_pins {} {
    set block [ord::get_db_block]
    set count 0
    foreach inst [$block getInsts] {
        set master [$inst getMaster]
        if { ![string match "RM_IHPSG13_*" [$master getName]] } { continue }
        if { [$inst getOrient] != "R0" } {
            error "pw_export_macro_power_pins: [$inst getName] must be placed R0 (got [$inst getOrient])"
        }
        lassign [$inst getLocation] ix iy
        foreach mterm [$master getMTerms] {
            set pname [$mterm getName]
            if { $pname == "VDD!" || $pname == "VDDARRAY!" } {
                set net_name $::env(VDD_NET)
            } elseif { $pname == "VSS!" } {
                set net_name $::env(GND_NET)
            } else {
                continue
            }
            set net [$block findNet $net_name]
            set bterm [$block findBTerm $net_name]
            if { $bterm == "NULL" } {
                set bterm [odb::dbBTerm_create $net $net_name]
                $bterm setSigType [$net getSigType]
                $bterm setIoType INOUT
                $bterm setSpecial
            }
            foreach mpin [$mterm getMPins] {
                foreach box [$mpin getGeometry] {
                    set layer [$box getTechLayer]
                    if { $layer == "NULL" || [$layer getName] != "Metal4" } { continue }
                    set bpin [odb::dbBPin_create $bterm]
                    odb::dbBox_create $bpin $layer \
                        [expr {$ix + [$box xMin]}] [expr {$iy + [$box yMin]}] \
                        [expr {$ix + [$box xMax]}] [expr {$iy + [$box yMax]}]
                    $bpin setPlacementStatus FIRM
                    incr count
                }
            }
        }
    }
    puts "\[INFO\] pw_export_macro_power_pins: exported $count macro Metal4 power straps as block pins"
    if { $count == 0 } {
        error "pw_export_macro_power_pins: no SRAM power straps found"
    }
}
pw_export_macro_power_pins
