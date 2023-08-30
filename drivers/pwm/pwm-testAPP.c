#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#define EN_ADDRESS

#define CMD_IOC_MAGIC   'a'
#define PWM_CAMERA_SET_PERIOD _IOW(CMD_IOC_MAGIC,1,int)
#define PWM_CAMERA_SET_DUTY   _IOW(CMD_IOC_MAGIC,5,int)
#define PWM_CAMERA_ENABLE     _IO(CMD_IOC_MAGIC,3)
#define PWM_CAMERA_DISABLE    _IO(CMD_IOC_MAGIC,4)

int main()                                                                                                                                                                                                                                                                                                                                                                                                                              
{
    unsigned int i=0;
    int duty,period;
    int rc,fd;

    fd = open("/dev/camera_led,pwm",O_RDWR);
    if (fd < 0)
    {
        printf("open camera,pwm fail\n");
        return -1;
    }

    /*set period*/
    period = 5000;
    
    #ifdef EN_ADDRESS
        rc = ioctl(fd,PWM_CAMERA_SET_PERIOD,&period);
    #else
        rc= ioctl(fd,PWM_CAMERA_SET_PERIOD,period);
    #endif

    printf("rc =%d,Set pwm priod = %d\n",rc,period);

    duty = 0;
    for(i=1;i<7;i++)
    {
        #ifdef EN_ADDRESS
            rc = ioctl(fd,PWM_CAMERA_SET_DUTY,&duty);
        #else
            rc = ioctl(fd,PWM_CAMERA_SET_DUTY,duty);
        #endif
        
        if (rc ==-1)
        {
            printf("ioctl: %s\n",strerror(errno));
        }
        printf("i=%d,rc =%d,set duty =%d\n",i,rc,duty);
        duty=duty+1000;
        sleep(5);
    }

    rc=ioctl(fd,PWM_CAMERA_DISABLE);

        printf("rc=%d,\n",rc);
        
    close(fd);
    
    return 0;
}