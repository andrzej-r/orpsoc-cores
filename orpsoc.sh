
export PATH=/opt/or1k-elf/bin:$PATH
export PATH=/opt/cross/or1k-linux-musl/bin:$PATH
export PATH=/opt/Xilinx/Vivado/2015.1/bin:$PATH
export PATH=/opt/Xilinx/14.7/ISE_DS/ISE/bin/lin64:$PATH

export ARCH=openrisc
export CROSS_COMPILE=or1k-elf-
#export KBUILD_DEFCONFIG=generic_defconfig
export KBUILD_DEFCONFIG=nexys4ddr_defconfig
