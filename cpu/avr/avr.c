#include <avr/io.h>

#include "contiki-conf.h"

void
cpu_init(void)
{
  asm volatile ("clr r1");	/* No longer needed. */
}

extern int __bss_end;

#define STACK_EXTRA 32
static char *cur_break = (char *)(&__bss_end + 1);

/*
 * Allocate memory from the heap. Check that we don't collide with the
 * stack right now (some other routine might later). A watchdog might
 * be used to check if cur_break and the stack pointer meet during
 * runtime.
 */
void *
sbrk(int incr)
{
  char *stack_pointer;

  stack_pointer = (char *)SP;
  stack_pointer -= STACK_EXTRA;
  if(incr > (stack_pointer - cur_break))
    return (void *)-1;          /* ENOMEM */

  void *old_break = cur_break;
  cur_break += incr;
  /*
   * If the stack was never here then [old_break .. cur_break] should
   * be filled with zeros.
  */
  return old_break;
}

#ifdef __XMEGA__
#include <avr/pgmspace.h>

uint8_t xmega_read_calibration_byte(uint8_t index)
{
	uint8_t result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return result;
}
#endif
