# Shared init + verification helpers for blockage tests.
# Assumes CWD is src/cms/test/ASAP7/blockage_tests/

source "../../helpers.tcl"

proc init_design {} {
    read_db "../3_3_place_gp.odb"
    read_liberty "../asap7/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz"
    read_liberty "../asap7/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz"
    read_liberty "../asap7/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz"
    read_liberty "../asap7/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz"
    read_liberty "../asap7/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib"
    read_sdc "../3_place.sdc"
    detailed_placement
    source "../asap7/setRC.tcl"
}

# Returns list of blockage rects [list x1 y1 x2 y2 name] from both
# dbBlockages and macros (dbInst::isBlock()), so verification checks
# against BOTH paths in the code.
proc collect_all_blockages {block} {
    set rects {}
    foreach blk [$block getBlockages] {
        set bb [$blk getBBox]
        lappend rects [list [$bb xMin] [$bb yMin] [$bb xMax] [$bb yMax] "blockage"]
    }
    foreach inst [$block getInsts] {
        if { [$inst isBlock] } {
            set bb [$inst getBBox]
            lappend rects [list [$bb xMin] [$bb yMin] [$bb xMax] [$bb yMax] [$inst getName]]
        }
    }
    return $rects
}

# Run create_clock_mesh and verify NO wire/buffer lands in ANY blockage rect.
# Returns a dict with {wires vias buffers wire_violations buf_violations}.
proc run_and_verify { {halo_um 0.5} } {
    set block [ord::get_db_block]
    set blockage_rects [collect_all_blockages $block]

    create_clock_mesh \
        -clock core_clock \
        -h_layer M4 \
        -v_layer M5 \
        -pitch 0.7 \
        -buffers [list BUFx4_ASAP7_75t_R] \
        -macro_halo $halo_um

    set mesh_net [$block findNet "clk_mesh"]
    if { $mesh_net == "NULL" } {
        return [dict create ok 0 error "mesh net 'clk_mesh' not found"]
    }

    set wires 0; set vias 0
    set wire_violations 0
    foreach swire [$mesh_net getSWires] {
        foreach sbox [$swire getWires] {
            if { [$sbox isVia] } {
                incr vias
                continue
            }
            incr wires
            set wx1 [$sbox xMin]; set wy1 [$sbox yMin]
            set wx2 [$sbox xMax]; set wy2 [$sbox yMax]
            foreach r $blockage_rects {
                lassign $r mx1 my1 mx2 my2 mname
                if { $wx1 < $mx2 && $wx2 > $mx1 && $wy1 < $my2 && $wy2 > $my1 } {
                    puts "VIOLATION wire: ($wx1,$wy1)-($wx2,$wy2) overlaps $mname"
                    incr wire_violations
                }
            }
        }
    }

    set buffers 0; set buf_violations 0
    foreach inst [$block getInsts] {
        if { ! [string match "mesh_buf_*" [$inst getName]] } { continue }
        incr buffers
        set origin [$inst getOrigin]
        set bx [lindex $origin 0]; set by [lindex $origin 1]
        foreach r $blockage_rects {
            lassign $r mx1 my1 mx2 my2 mname
            if { $bx >= $mx1 && $bx <= $mx2 && $by >= $my1 && $by <= $my2 } {
                puts "VIOLATION buffer: [$inst getName] at ($bx,$by) inside $mname"
                incr buf_violations
            }
        }
    }

    return [dict create \
        ok [expr {$wire_violations == 0 && $buf_violations == 0}] \
        wires $wires vias $vias buffers $buffers \
        wire_violations $wire_violations buf_violations $buf_violations \
        blockage_count [llength $blockage_rects]]
}

proc report_and_exit {name result} {
    puts ""
    puts "===== $name ====="
    dict for {k v} $result { puts [format "  %-18s %s" $k $v] }
    puts "================================"
    if { [dict get $result ok] } {
        puts "PASS: $name"
        exit 0
    } else {
        puts "FAIL: $name"
        exit 1
    }
}
