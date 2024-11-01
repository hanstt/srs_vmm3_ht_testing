/*
 * Generates vmm data.
 */

#include <err.h>
#include <stdlib.h>
#include <unistd.h>

int
main()
{
	char buf[1 << 20];

	for (;;) {
		ssize_t ret;
		size_t bytes;

		bytes = sizeof buf;
		ret = write(STDOUT_FILENO, buf, bytes);
		if (-1 == ret) {
			err(EXIT_FAILURE, "write");
		}
	}
	return 0;
}
