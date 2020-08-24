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


#include <nuttx/sensors/adis16488.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * acc_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int adis16488a_example_main(int argc, char *argv[])
#endif
{
	int fd = -1, ret = -1;
	uint16_t	tmp= 0;
	char id[2];

	fd = open("/dev/adis16488_1",O_RDWR);
	if(fd < 0){
		printf("Failed open adis16488_1:%d\n",fd);
	}

	/*id*/
	ret = lseek(fd,ADIS16488_REG_PROD_ID,SEEK_SET);
	if(ret < 0){
		printf("Failed seek register:%d\n",ret);
	}
	ret = read(fd,id,sizeof(id));
	if(ret < 0){
			printf("Failed read register:%d\n",ret);
	}

    printf("id:   %d\n",*(short *)id);


	char dec_rate[2];

	ret = lseek(fd,ADIS16488_REG_DEC_RATE,SEEK_SET);
	if(ret < 0){
		printf("Failed seek register:%d\n",ret);
	}
	ret = read(fd,dec_rate,sizeof(dec_rate));
	if(ret < 0){
			printf("Failed read register:%d\n",ret);
	}

	printf("dec_rate:   %d\n",*(short *)dec_rate);


	char bnk0[2];
	char bnk1[2];

	ret = lseek(fd,ADIS16488_REG_FILTER_BNK0,SEEK_SET);
	if(ret < 0){
		printf("Failed seek register:%d\n",ret);
	}
	ret = read(fd,bnk0,sizeof(bnk0));
	if(ret < 0){
			printf("Failed read register:%d\n",ret);
	}

	printf("bnk0:   %d\n",*(short *)bnk0);

	ret = lseek(fd,ADIS16488_REG_FILTER_BNK0,SEEK_SET);
	if(ret < 0){
		printf("Failed seek register:0x%04x\n",ret);
	}
	ret = read(fd,bnk1,sizeof(bnk1));
	if(ret < 0){
			printf("Failed read register:0x%04x\n",ret);
	}

	printf("bnk1:   %d\n",*(short *)bnk1);

	return 0;
}

