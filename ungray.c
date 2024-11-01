#include <stdint.h>
#include <stdio.h>

uint32_t
ungray(uint32_t a_u)
{
	a_u ^= a_u >> 16;
	a_u ^= a_u >>  8;
	a_u ^= a_u >>  4;
	a_u ^= a_u >>  2;
	a_u ^= a_u >>  1;
	return a_u;
}

int
main()
{
	uint32_t i;

	printf("uint32_t g_ungray[] = {\n");
	for (i = 0; i < 0xfff; ++i) {
		printf("\t0x%04x,\n", ungray(i));
	}
	printf("\t0x%04x\n", ungray(0xfff));
	printf("};\n");
	return 0;
}
