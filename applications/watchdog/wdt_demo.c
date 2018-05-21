#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/watchdog.h>

int main(void)
{

	int ii = 3;
	int fd = open("/dev/watchdog", O_WRONLY);
	if (fd == -1) {
		perror("watchdog");
		exit(EXIT_FAILURE);
	}

	printf("Open watchdog ok\n");

	while (1) {
		printf("ii = %d\n", ii);
		if(ii-- > 1)
			ioctl(fd, WDIOC_KEEPALIVE);
		sleep(5);
	}
	close(fd);
	return 0;
}
