/*acc  calibration*/



/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/fs.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>


#include <nuttx/sensors/dac60096.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * acc_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int dac60096_example_main(int argc, char *argv[])
#endif
{
	int fd = -1,ret = -1;

	// Open the device.
	fd = open("/dev/dac60096_1",O_RDWR);
	if(fd < 0){
		printf("Failed open dac60096_1:%d\n",fd);

		return fd;
	}

	// Configure the device.
	ret = ioctl(fd,DACIOC_CONFIG_DEFAULT,0);
	if(ret < 0){
		printf("Failed to configure dac60096:%d\n",ret);

		return ret;
	}

	// Set channel value
	struct dac60096_chan_value_s cv;
	cv.channel = atoi(argv[1]);
	cv.value = atoi(argv[2]);

	ret = ioctl(fd,DACIOC_SET_CHANNEL_VALUE,(unsigned long)&cv);
	if(ret < 0){
		printf("Failed to set dac60096 channal[%d] value[%d]\n",atoi(argv[1]),atoi(argv[2]));

		return ret;
	}

	// Enable channel output
	ret = ioctl(fd,DACIOC_ENABLE_OUTPUT,0);
	if(ret < 0){
		printf("Failed to enable dac60096 channal output:%d\n",atoi(argv[1]));

		return fd;
	}


	return 0;
}

