#include <stdio.h>
#include <or1k-support.h>

//#define REG32(add) *((volatile unsigned long *) (add))

#define GPIO_LED_BASE  0x91002000
#define GPIO_LED_DIR   (GPIO_LED_BASE + 4)
#define GPIO_LED_DATA  (GPIO_LED_BASE + 0)
#define RGB_LED_BASE   0x91005000
#define RGB_LED_EN     (GPIO_LED_BASE + 0)
#define RGB_LED_0      (GPIO_LED_BASE + 0x8)
#define RGB_LED_1      (GPIO_LED_BASE + 0xc)
#define SSEG_CTRL_BASE 0x91004000
#define SSEG_CTRL_EN   (SSEG_CTRL_BASE)
#define SSEG_CTRL_DIG0 (SSEG_CTRL_BASE + 0x20)
#define SSEG_CTRL_DIG1 (SSEG_CTRL_BASE + 0x24)
#define SSEG_CTRL_DIG2 (SSEG_CTRL_BASE + 0x28)
#define SSEG_CTRL_DIG3 (SSEG_CTRL_BASE + 0x2c)
#define SSEG_CTRL_DIG4 (SSEG_CTRL_BASE + 0x30)
#define SSEG_CTRL_DIG5 (SSEG_CTRL_BASE + 0x34)
#define SSEG_CTRL_DIG6 (SSEG_CTRL_BASE + 0x38)
#define SSEG_CTRL_DIG7 (SSEG_CTRL_BASE + 0x3c)
#define XADC_BASE      0x95000000
#define XADC_TEMP      (XADC_BASE + 0x200)
#define DDR2_CTRL_BASE 0x96000000
#define DDR2_CTRL_TEMP (DDR2_CTRL_BASE + 0x4)

static unsigned char hexDigits[16] = {
  0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
  0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};

void displayHex (unsigned long val)
{
  int i;
  int nibble;
  unsigned long digit;
  REG16(SSEG_CTRL_EN) = (unsigned long) 0x1;
  for (i = 0; i < 32; i += 4)
    {
      nibble = (val >> i) & 0xf;
      digit = hexDigits[nibble];
      REG16(SSEG_CTRL_DIG7 - i) = digit;
      //printf("Val: %08x, nibble: %x, digit: %02x\n", val, nibble, digit);
    }
}

void memTest (unsigned long start, unsigned long end)
{
  unsigned long addr;
  unsigned long val;
  for (addr = start; addr < end; addr += 4)
    {
      REG32(addr) = addr;
      if (addr % 256 == 0)
        {
          displayHex(addr);
          printf("Write32 address: %08x\n", addr);
        }
    }
  for (addr = start; addr < end; addr += 4)
    {
      val = REG32(addr);
      if (addr % 256 == 0)
        {
          displayHex(addr);
          printf("Read32 address: %08x\n", addr);
        }
      if (val != addr)
        printf ("Incorrect value %08x at address %08x, expected %08x\n", val, addr, addr);
    }

  for (addr = start; addr < end; addr += 2)
    {
      REG16(addr) = addr % 0xffff;
      if (addr % 256 == 0)
        {
          displayHex(addr);
          printf("Write16 address: %08x\n", addr);
        }
    }
  for (addr = start; addr < end; addr += 2)
    {
      val = REG16(addr);
      if (addr % 256 == 0)
        {
          displayHex(addr);
          printf("Read16 address: %08x\n", addr);
        }
      if (val != addr % 0xffff)
        printf ("Incorrect value %04x at address %08x, expected %04x\n", val, addr, addr % 0xffff);
    }

  for (addr = start; addr < end; addr += 1)
    {
      REG8(addr) = addr % 0xff;
      if (addr % 256 == 0)
        {
          displayHex(addr);
          printf("Write8 address: %08x\n", addr);
        }
    }
  for (addr = start; addr < end; addr += 1)
    {
      val = REG8(addr);
      if (addr % 256 == 0)
        {
          displayHex(addr);
          printf("Read8 address: %08x\n", addr);
        }
      if (val != addr % 0xff)
        printf ("Incorrect value %02x at address %08x, expected %02x\n", val, addr, addr % 0xff);
    }
}

unsigned long memFind (unsigned long val)
{
  unsigned long addr;
  unsigned long val_mem;
  for (addr = 0; addr < 0x04000000; addr = addr + 4)
    {
      val_mem = REG32(addr);
      if (val_mem == val)
        break;
    }
  printf("Address: %08x\n", addr);
  return addr;
}

unsigned long readTemperature (void)
{
  unsigned long val;
  val = REG16(XADC_TEMP);
  printf ("Die temperature: %f Cdeg (raw value: %08x)\n", val * 503.975 / (1<<16) - 273.15, val);
  return val;
}

void calibrateTemperature (unsigned long val)
{
  unsigned long valShifted = val >> 4;
  REG16(DDR2_CTRL_TEMP) = valShifted;
}

int main (int argc, char *argv[])
{
  REG16(GPIO_LED_DIR)  = 0xaaaa;
  REG16(GPIO_LED_DATA) = 0xaaaa;
  printf("Hello World!\n");
  //calibrateTemperature(readTemperature());
  memTest(0x02200000,0x02210000);
  //memFind(0x01b77f90);
  return 0;
}
