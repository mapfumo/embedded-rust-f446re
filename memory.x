/* memory.x for STM32F446RE (128KB Flash, 128KB RAM) */

MEMORY
{
  /* NOTE K = 1024, M = 1024*1024 */
  
  /* Flash Memory (ReadOnly/Executable) */
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  
  /* SRAM1 (Data/Stack/Heap) */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K 
  
  /* CCM Data RAM (for critical data) - Not strictly needed for Blinky, but good practice */
  CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K
}