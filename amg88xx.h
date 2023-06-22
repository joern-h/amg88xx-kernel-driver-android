#ifndef __LINUX_FOCALTECH_CORE_H__
#define __LINUX_FOCALTECH_CORE_H__

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#define DRIVER_NAME "amg88xx"

/*
 * Since we're dealing with 12-bit numbers the sign-bit needs to be extended
 * for the number to be represented correctly
 */
#define convert_to_s16(dst, src) \
    dst = src & 0x800 ? (src | (0xf << 12)) : src

/* i2c register addresses */
#define DEVICE_MODE_REG 0x00
#define RESET_REG 0x01
#define FRAMERATE_REG 0x02
#define INTERRUPT_CTRL_REG 0x03
#define STATUS_FLAG_REG 0x04     // TODO | one sysfs entry
#define STATUS_FLAG_CLR_REG 0x05 // TODO |
#define MOVING_AVERAGE_REG 0x07  // TODO
#define UPPER_INTERRUPT_LOW_REG 0x08
#define UPPER_INTERRUPT_HIGH_REG 0x09
#define LOWER_INTERRUPT_LOW_REG 0x0a
#define LOWER_INTERRUPT_HIGH_REG 0x0b
#define INTERRUPT_HYST_LOW_REG 0x0c
#define INTERRUPT_HYST_HIGH_REG 0x0d
#define THERM_LOW_REG 0x0e
#define THERM_HIGH_REG 0x0f
#define PIXEL_ROW1_REG 0x10
#define PIXEL_ROW2_REG 0x11
#define PIXEL_ROW3_REG 0x12
#define PIXEL_ROW4_REG 0x13
#define PIXEL_ROW5_REG 0x14
#define PIXEL_ROW6_REG 0x15
#define PIXEL_ROW7_REG 0x16
#define PIXEL_ROW8_REG 0x17
#define SENSOR_FIRST_REG 0x80 // Pixel 1 low bits register
#define SENSOR_LAST_REG 0xFF  // Pixel 64 high bits register

#define AMG_DEBUG_EN 1

#if AMG_DEBUG_EN
#define AMG_DEBUG(fmt, args...)                          \
    do                                                   \
    {                                                    \
        printk("[AMG/D]%s:" fmt "\n", __func__, ##args); \
    } while (0)

#define AMG_FUNC_ENTER()                      \
    do                                        \
    {                                         \
        printk("[AMG]%s: Enter\n", __func__); \
    } while (0)

#define AMG_FUNC_EXIT()                                    \
    do                                                     \
    {                                                      \
        printk("[AMG]%s: Exit(%d)\n", __func__, __LINE__); \
    } while (0)
#else /* #if AMG_DEBUG_EN*/
#define AMG_DEBUG(fmt, args...)
#define AMG_FUNC_ENTER()
#define AMG_FUNC_EXIT()
#endif

#define AMG_INFO(fmt, args...)                                    \
    do                                                            \
    {                                                             \
        printk(KERN_ERR "[AMG/I]%s:" fmt "\n", __func__, ##args); \
    } while (0)

#define AMG_ERROR(fmt, args...)                                   \
    do                                                            \
    {                                                             \
        printk(KERN_ERR "[AMG/E]%s:" fmt "\n", __func__, ##args); \
    } while (0)

#define ENABLE 1
#define DISABLE 0
#define AMG_I2C_VTG_MIN_UV 1800000
#define AMG_I2C_VTG_MAX_UV 1800000

/*
 * Device configuration options that are mapped to register values
 */
enum amg88xx_device_mode
{
    NORMAL_MODE = 0x0,
    SLEEP_MODE = 0x10,
    STANDBY60_MODE = 0x20,
    STANDBY10_MODE = 0x21
};

static const char *mode_strs[4] = {
    "normal",
    "sleep",
    "standby_60",
    "standby_10"};

enum amg88xx_reset_mode
{
    PARTIAL_RST = 0x30,
    FULL_RST = 0x3f
};

enum amg88xx_fps
{
    FPS10 = 0,
    FPS1 = 1
};

enum amg88xx_interrupt_mode
{
    DIFFERENCE_MODE = 0, // FIXME
    ABSOLUTE_VALUE_MODE = 1
};

enum amg88xx_interrupt_state
{
    INT_DISABLED = 0,
    INT_ENABLED = 1
};

/*
 * Structure for holding device related data
 */
struct amg88xx
{
    struct i2c_client *client;
    struct gpio_desc *int_gpio;
    struct device *dev;
    u32 en_gpio;
    u32 en_gpio_flags;
    int en;
    bool power_disabled;
    struct regulator *vcc_i2c;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_active;
    struct pinctrl_state *pins_suspend;
    struct pinctrl_state *pins_release;
};


#endif