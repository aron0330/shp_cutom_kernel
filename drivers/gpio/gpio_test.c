/*
 * gpio_test.c
 *
 * Develop with Eclipse (LUNA)
 *
 *  Created on: 2014. 8. 13.
 *      Author: shellbt
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include "gpio.h"

void printUsage(char *app)
{
	printf("USAGE : %s [options]\n", app);
	printf(" [options] : \n");
	printf(" 	-k : key led on/off\n");
	printf(" 	-o : online led on/off\n");
	printf(" 	-i : icc led on/off\n");
}

int main(int argc, char *argv[])
{
	int data = 0;
	int fd = -1;
	char options=0;
	char opt = 0;
	char cmd = 0;
	char version[10] = {0};


	if ( argc > 1 )
	{
		while( (opt = getopt(argc, argv, "koi")) != -1 )
		{
			switch( opt )
			{
			case 'k': case 'o': case 'i':
				cmd = opt;
				printf("cmd = %c\n",cmd);
				break;


			default:
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
			}
			if (cmd) break;
		}
	}



	if ( (fd = open("/dev/gpio", O_RDONLY)) < 0 )
	{
		printf("open error %s\n", strerror(errno) );
		exit(EXIT_FAILURE);
	}


	if ( ioctl(fd, GPIO_VERSION, &version) < 0 )
	{
		printf("ioctl(GPIO_VERSION) error : %s\n", strerror(errno) );
	}
	else
		printf("GPIO Device Driver Version : %s\n", version);

	if ( !cmd )
	{
		if ( ioctl(fd, GPIO_MODEM, &data) < 0 )
			goto failed;
		printf("GPIO_MODEM = %d\n", data);

		data = 0;
		if ( ioctl(fd, GPIO_PRIVATE, &data) < 0 )
			goto failed;
		printf("GPIO_PRIVATE = %d\n", data);

		data = 0;
		if ( ioctl(fd, GPIO_PINPAD2, &data) < 0 )
			goto failed;
		printf("GPIO_PINPAD2 = %d\n", data);

		data = 0;
		if ( ioctl(fd, GPIO_DEBUG, &data) < 0 )
			goto failed;
		printf("GPIO_DEBUG = %d\n", data);

		if ( ioctl(fd, GPIO_KEY_LED, &data) < 0 )
			goto failed;
		printf("GPIO_KEY_LED = %d\n", data);

		if ( ioctl(fd, GPIO_ON_LED, &data) < 0 )
			goto failed;
		printf("GPIO_ON_LED = %d\n", data);

		if ( ioctl(fd, GPIO_ICC_LED, &data) < 0 )
			goto failed;
		printf("GPIO_ICC_LED = %d\n", data);
	}
	else
	{	/* cmd */
		switch( cmd )
		{
			case 'k':
				data = -1;
				if ( ioctl(fd, GPIO_KEY_LED, &data) < 0 )
					goto failed;

				if ( data )
				{
					data = 0;
					printf("KEY_LED will be OFF\n");
				}
				else
				{
					data = 1;
					printf("KEY_LED will be ON\n");
				}

				if ( ioctl(fd, GPIO_KEY_LED, &data) < 0 )
					goto failed;
				break;

			case 'o':
				data = -1;
				if ( ioctl(fd, GPIO_ON_LED, &data) < 0 )
					goto failed;

				if ( data )
				{
					data = 0;
					printf("ON_LED will be OFF\n");
				}
				else
				{
					data = 1;
					printf("ON_LED will be ON\n");
				}

				if ( ioctl(fd, GPIO_ON_LED, &data) < 0 )
					goto failed;
				break;

			case 'i':
				data = -1;
				if ( ioctl(fd, GPIO_ICC_LED, &data) < 0 )
					goto failed;

				if ( data )
				{
					data = 0;
					printf("ICC_LED will be OFF\n");
				}
				else
				{
					data = 1;
					printf("ICC_LED will be ON\n");
				}

				if ( ioctl(fd, GPIO_ICC_LED, &data) < 0 )
					goto failed;
				break;
		}

	}

	return 0;

failed:
	printf("ioctl error %s\n", strerror(errno) );
	return 127;
}

