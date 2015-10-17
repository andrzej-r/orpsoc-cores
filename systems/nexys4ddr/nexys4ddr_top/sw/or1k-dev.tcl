set TAP_TYPE XILINX_BSCAN

source [find digilent_jtag_test.cfg]
source [find xilinx-xc7.cfg]
source [find or1k.cfg]

proc or1k_test_init {} {
    
    
}

gdb_port 50001

init
#echo "Halting processor"
halt

foreach name [target names] {
    set y [$name cget -endian]
    set z [$name cget -type]
    puts [format "Chip is %s, Endian: %s, type: %s" \
                  $name $y $z]
}
