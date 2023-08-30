#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/ioctl.h>


#define EN_ADDRESS

#define CMD_IOC_MAGIC   'a'
#define PWM_CAMERA_SET_PERIOD _IOW(CMD_IOC_MAGIC,1,int)
#define PWM_CAMERA_SET_DUTY   _IOW(CMD_IOC_MAGIC,5,int)
#define PWM_CAMERA_ENABLE     _IO(CMD_IOC_MAGIC,3)
#define PWM_CAMERA_DISABLE    _IO(CMD_IOC_MAGIC,4)


struct pwm_param{
    int duty_ns;
    int period_ns;
    struct pwm_device *pwm;
};

struct pwm_param camera;


static void pwm_camera_update(void)
{    
    pwm_config(camera.pwm,camera.duty_ns,camera.period_ns);
}


static int pwm_camera_open(struct inode *inode,struct file *filp)
{
    int ret = 0;
    pwm_set_polarity(camera.pwm,PWM_POLARITY_NORMAL);
    //camera.period_ns = 5000;
    //camera.duty_ns = 1000;
    pwm_enable(camera.pwm);

    printk("camera_led_pwm open \r\n");
    return ret;
}

static int pwm_camera_release(struct inode *inode,struct file *filp)
{
    pwm_disable(camera.pwm);
    return 0;
}

static  ssize_t pwm_camera_read(struct file *filp, char __user *buf, size_t len, loff_t *pos)
{
    return 0;
}

static ssize_t pwm_camera_write(struct file *filp, const char __user *buf, size_t len, loff_t *pos)
{
    return 0;
}

static long pwm_camera_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    unsigned int period;
    unsigned int duty;
    int rc = 0;

    void __user *argp = (void __user *)arg;

    switch (cmd)
    {
        case PWM_CAMERA_SET_PERIOD:
             #ifdef  EN_ADDRESS
            if(argp == NULL){
                printk("camera:invallid argument.");
                return -EINVAL;
            }
           
            if  (copy_from_user(&camera.period_ns, argp, sizeof(camera.period_ns)))
            {
                printk("copy_from_user failed.");
                return -EFAULT;
            }
            #endif

            rc =1;

            #ifndef EN_ADDRESS

                camera.period_ns = arg;
            #endif

            printk("%s: ioc read period = %d",__func__,camera.period_ns);

            pwm_config(camera.pwm,camera.duty_ns,camera.period_ns);
            break;

  

        case PWM_CAMERA_SET_DUTY:
              #ifdef EN_ADDRESS
            if (argp == NULL)
            {
                printk("camera: invalid argument.");
                return -EINVAL;
            }
          
            if (copy_from_user(&camera.duty_ns, argp, sizeof(camera.duty_ns))){
                printk("copy_from_user failed.");
                return -EFAULT;
            }
            #endif

            
            rc =2;
            #ifndef EN_ADDRESS
              camera.duty_ns=arg;
            #endif

                printk("%s: ioc read duty = %d",__func__,camera.duty_ns);

                
            if ((camera.duty_ns<0) ||(camera.duty_ns >5000)){
                printk("camera:invalid argument.");
                return -EINVAL;
            }
          
            pwm_config(camera.pwm,camera.duty_ns,camera.period_ns);
            break;
        
   
        case PWM_CAMERA_ENABLE:
            rc = 3;
            pwm_enable(camera.pwm);
            break;

        case PWM_CAMERA_DISABLE:
            rc =4;
            pwm_disable(camera.pwm);
            break;

        default:
            printk("camera: cmd error!\n");
            return -EFAULT;
    }
    return rc;
}

struct file_operations pwm_camera_fops = {
    .owner = THIS_MODULE,
    .open = pwm_camera_open,
    .release = pwm_camera_release,
    .write = pwm_camera_write,
    .unlocked_ioctl = pwm_camera_ioctl,
};

struct miscdevice pwm_camera_dev ={
    .minor = MISC_DYNAMIC_MINOR,
    .fops = &pwm_camera_fops,
    .name = "camera_led,pwm",
};

static void pwm_camera_test(void)
{
    int i =0,led_bar=1000;
    pwm_enable(camera.pwm);
    //while (1)
    {
        for (i=0;i<5;i++)
        {
            camera.duty_ns=i*led_bar;
            
             printk("%s: ioc read i = %d",__func__,i);
            pwm_camera_update();

             printk("%s: ioc read camera.duty = %d,camera.period =%d\n",__func__,camera.duty_ns,camera.period_ns);

            msleep(1000);
        }
        
    }
}

static int pwm_camera_probe(struct  platform_device *pdev)
{
    int ret = 0;
    struct device_node *child;
    struct device *dev = &pdev->dev;

    printk("match pwm_camera success\n");

    child = of_get_next_child(dev->of_node, NULL);
    if (child){
       
        camera.pwm = devm_of_pwm_get(dev, child, NULL);
        if (IS_ERR(camera.pwm)){
            dev_err(&pdev->dev,"camera: unable to request legacy PWM\n");
            
            return ret-1;
        }
    }
    else{
        printk(KERN_ERR"pwm_camera of get_next_child error!!!\n");
        return -1;
    }
    

    //set pwm
    camera.period_ns = 5000;
    camera.duty_ns = 1000;
    pwm_config(camera.pwm,camera.duty_ns,camera.period_ns);
    ret = pwm_set_polarity(camera.pwm, PWM_POLARITY_NORMAL);
    if (ret < 0)
    {
        printk("pwm set polarity fail, ret = %d\n",ret);
        return ret;
    }

    pwm_enable(camera.pwm) ;

        //
    misc_register(&pwm_camera_dev);

        //lgh
        msleep(1000);
        pwm_camera_test();
    return 0;
}

static int pwm_camera_remove(struct platform_device *pdev)
{
    pwm_disable(camera.pwm);
    pwm_free(camera.pwm);

    misc_deregister(&pwm_camera_dev);
    return 0;
}

int pwm_camera_suspend(struct device *dev)
{
    return 0;
}

int pwm_camera_resume(struct device *dev)
{
    return 0;
}

const struct dev_pm_ops pwm_camera_pm_ops ={
    .suspend = pwm_camera_suspend,
    .resume = pwm_camera_resume,
};

static const struct of_device_id of_pwm_camera_match[] = {
    {.compatible = "camera_led,pwm", },
    {},
};

static struct platform_driver pwm_camera_driver = {

    .probe = pwm_camera_probe,
    .remove = pwm_camera_remove,
    .driver ={
        .name = "camera_led,pwm",
        .owner = THIS_MODULE,
        //.pm = &pwm_camera_pm_ops,
        .of_match_table = of_pwm_camera_match,
    },
};

module_platform_driver(pwm_camera_driver);

MODULE_DESCRIPTION("pwm control driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("lgh");