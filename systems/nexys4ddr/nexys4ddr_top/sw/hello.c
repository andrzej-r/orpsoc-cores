#include <stdio.h>

#define GPIO_LED_BASE 0x91002000
#define GPIO_LED_DIR  (GPIO_LED_BASE + 4)
#define GPIO_LED_DATA (GPIO_LED_BASE + 0)
#define RGB_LED_BASE 0x93000000
#define RGB_LED_EN   (GPIO_LED_BASE + 0)
#define RGB_LED_0    (GPIO_LED_BASE + 0x8)
#define RGB_LED_1    (GPIO_LED_BASE + 0xc)

int main (int argc, char *argv[])
{
  //*((volatile unsigned long *)(GPIO_LED_DIR)) = 0xffffffff;
  //*((volatile unsigned long *)(GPIO_LED_DATA)) = 0xffffffff;
  //*((volatile unsigned long *)(RGB_LED_EN)) = 0x00000001;
  //*((volatile unsigned long *)(RGB_LED_0)) = 0x00ff00ff;
  //*((volatile unsigned long *)(RGB_LED_1)) = 0x00ffffff;
  //REG32(GPIO_LED_DIR)  = 0xaaaaaaaa;
  //REG32(GPIO_LED_DATA) = 0xaaaaaaaa;
  printf("Hello World!\n");
  return 0;
}
