//
// orpsoc-defines
//
`define MOR1KX

`ifndef MOR1KX
 `define OR1200
`endif

`define UART0
`define SPI_FLASH
`define SPI_ACCEL
`define I2C_TEMP
`undef VGA0
`undef ETH0
`define PS2_0
`define DEBUG_JTAG
//`undef DEBUG_JTAG
`ifdef DEBUG_JTAG
 `define JTAG_TAP_XILINX
 `undef JTAG_TAP_CUSTOM
`endif
`define DEBUG_RS232
`define BOOTROM
`define XADC0
`ifdef XADC0
 `define DDR2
`endif

`define GPIO_SW
`define GPIO_PBTN
`define GPIO_LED
`define GPIO_PMOD
`define SSEG_DISP
`define RGB_LED

// end of included module defines - keep this comment line here
