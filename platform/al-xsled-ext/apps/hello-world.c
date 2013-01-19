/**
 *
 */

#include <stdio.h>
#include <contiki.h>


PROCESS(hello_world_process, "Hello world process");

PROCESS_THREAD(hello_world_process, ev, data)
{
	PROCESS_BEGIN();

	printf("Hello world from Contiki OS running on AL-XSLED_EXT\n");

	PROCESS_END();
}

