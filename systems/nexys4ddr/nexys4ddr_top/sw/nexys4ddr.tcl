interface ftdi
ftdi_vid_pid 0x0403 0x6010

ftdi_layout_init 0x2088 0x3f8b
ftdi_layout_signal nSRST -data 0x2000
ftdi_layout_signal GPIO2 -data 0x2000
ftdi_layout_signal GPIO1 -data 0x0200
ftdi_layout_signal GPIO0 -data 0x0100

#poll_period 1
adapter_khz 10000

set  _CHIPNAME or1k
set  _ENDIAN big

set FPGATAPID  0x13631093
#set FPGATAPID  0x03631093
set TAP_TYPE XILINX_BSCAN
source [find target/or1k.cfg]
#source [find or1k.cfg]

gdb_target_description enable
addreg rtest 0x1234 org.gnu.gdb.or1k.group0 system

proc init_reset {mode} {
	soft_reset_halt
	resume
}

#proc or1k_test_init {} {
#    
#    
#}

init
echo "Halting processor"
halt

foreach name [target names] {
    set y [$name cget -endian]
    set z [$name cget -type]
    puts [format "Chip is %s, Endian: %s, type: %s" \
                  $name $y $z]
}

set c_blue  "\033\[01;34m"
set c_reset "\033\[0m"

puts [format "%sTarget ready...%s" $c_blue $c_reset]
