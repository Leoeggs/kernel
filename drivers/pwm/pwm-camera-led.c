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

#define PWM_CAMERA_SET_PERIOD 0x01
#define PWM_CAMERA_SET_DUTY   0x02
#define PWM_CAMERA_ENABLE     0x03
#define PWM_CAMERA_DISABLE    0x04


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

static void pwm_camera_enable(void)
{
    pwm_enable(camera.pwm);
}

static void pwm_camera_disable(void)
{
    //

    pwm_camera_update();
    pwm_disable(camera.pwm);

}

static void pwm_camera_set_period(unsigned int period)
{
    
    pwm_set_period(camera.pwm,camera.period_ns);
}


static int pwm_camera_open(struct inode *inode,struct file *filp)
{
    int ret = 0;
    pwm_set_polarity(camera.pwm,PWM_POLARITY_NORMAL);
    pwm_enable(camera.pwm);
    printk("camera_led_pwm open \r\n");
    return ret;
}

static int pwm_camera_release(struct inode *inode,struct file *filp)
{
    pwm_camera_disable();
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

    void __user *argp = (void __user *)arg;

    switch (cmd)
    {
        case PWM_CAMERA_SET_PERIOD:
            if(argp == NULL){
                printk("camera:invallid argument.");
                return -EINVAL;
            }

            if  (copy_from_user(&period, argp, sizeof(unsigned int)))
            {
                printk("copy_from_user failed.");
                return -EFAULT;
            }

            pwm_camera_set_period(period);
            break;

  

        case PWM_CAMERA_SET_DUTY:

            if (argp == NULL)
            {
                printk("camera: invalid argument.");
                return -EINVAL;
            }

            if (copy_from_user(&duty, argp, sizeof(unsigned int))){
                printk("copy_from_user failed.");
                return -EFAULT;
            }

            if ((duty<0) ||(duty >100)){
                printk("camera:invalid argument.");
                return -EINVAL;
            }
            camera.duty_ns = duty;
            pwm_camera_update();
            break;
        
   
        case PWM_CAMERA_ENABLE:
            pwm_camera_update();
            pwm_camera_enable();
            break;

        case PWM_CAMERA_DISABLE:
            pwm_camera_disable();
            break;

        default:
            printk("camera: cmd error!\n");
            return -EFAULT;
    }
    return 0;
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
    pwm_config(camera.pwm,1000,5000);
    ret = pwm_set_polarity(camera.pwm, PWM_POLARITY_NORMAL);
    if (ret < 0)
    {
        printk("pwm set polarity fail, ret = %d\n",ret);
        return ret;
    }

    pwm_enable(camera.pwm) ;

        //
    misc_register(&pwm_camera_dev);

        //
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
        .pm = &pwm_camera_pm_ops,
        .of_match_table = of_pwm_camera_match,
    },
};

module_platform_driver(pwm_camera_driver);

MODULE_DESCRIPTION("pwm control driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("lgh");