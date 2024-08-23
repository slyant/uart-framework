# 基于rt-thread的串口通讯框架

#### 主、从通讯模式的完整例子：

##### 主机发送端（主）：
###### uart_master.c
```
# include <rtthread.h>
# include <rtdevice.h>
# include <board.h>
# include <stdio.h>
# include "uart_framework.h"

# define LOG_TAG              "uart.tx"
# define LOG_LVL              LOG_LVL_DBG
# include <ulog.h>

# define UART_NAME           "uart1"
# define UART_BAUD_RATE      BAUD_RATE_115200
# define RECV_BUF_SIZE       (128)
# define FRAME_TIMEOUT_MS    80
# define SEND_INTERVAL_MS    0
# define REQUEST_TIMEOUT_MS  2000

#define RS485RD             GPIOA_10
#define RS485_RX()          rt_pin_write(RS485RD, 0)
#define RS485_TX()          rt_pin_write(RS485RD, 1)

static uart_framework_t uf = RT_NULL;

static void rs485_set_tx(void)
{
    RS485_TX();
    rt_thread_mdelay(1);
}

static void rs485_set_rx(void)
{
    rt_thread_mdelay(1);
    RS485_RX();
}

rt_err_t uart_rs485_send(char *text)
{
    rt_err_t res = RT_EOK;

    rt_uint8_t *frame_data = (rt_uint8_t *) text;
    rt_uint16_t frame_len = rt_strlen(text);

    LOG_HEX("req_buf", 16, frame_data, frame_len);
    uart_framework_send_take(uf, frame_data, frame_len);
    res = uart_framework_receive_release(uf, REQUEST_TIMEOUT_MS, RT_NULL, RT_NULL, 0);
    if (res == RT_EOK)
    {
        LOG_HEX("uf->rx_buf", 16, uf->rx_buf, uf->rx_size);
        if (rt_memcmp(frame_data, uf->rx_buf, frame_len) == 0)
        {
            res = RT_EOK;
            LOG_I("test OK");
        }
        else
        {
            res = -RT_ERROR;
            LOG_E("test error");
        }
    }
    if (res != RT_EOK)
    {
        LOG_E("test failed(%d)", res);
    }

    return res;
}

static void rs485_gpio_init(void)
{
    rt_pin_mode(RS485RD, PIN_MODE_OUTPUT);
    rs485_set_rx();
}

static rt_err_t uart_rs485_init(void)
{
    rt_device_t serial_device = rt_device_find(UART_NAME);
    if (serial_device == RT_NULL)
    {
        LOG_E("not find the serial device:%s!", UART_NAME);
        return -RT_ERROR;
    }

    /* 修改串口配置 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = UART_BAUD_RATE;
    config.rx_bufsz = RECV_BUF_SIZE;
    config.tx_bufsz = 0;    //必须重新配置为0：阻塞发送，非阻塞接收
    rt_device_control(serial_device, RT_DEVICE_CTRL_CONFIG, &config);

    rs485_gpio_init();
    struct uart_framework_cfg cfg =
            { .uart_name = UART_NAME, .max_frame_size = RECV_BUF_SIZE, .frame_interval_ms =
            FRAME_TIMEOUT_MS, .send_interval_ms = SEND_INTERVAL_MS, .rs485_txd = rs485_set_tx, .rs485_rxd = rs485_set_rx };
    //如果不是485，则配置rs485_txd和rs485_rxd为RT_NULL

    uf = uart_framework_create(&cfg);
    if (uf == RT_NULL)
    {
        LOG_E("uart_framework_create failed on %s", UART_NAME);
        return -RT_ERROR;
    }
    else
    {
        LOG_I("uart_framework_create success on %s!", UART_NAME);
        return RT_EOK;
    }
}

static void rs485_tx_handler(void *params)
{
    while (1)
    {
        uart_rs485_send("test");
        rt_thread_mdelay(2000);
    }
}

int app_uart_rs485_startup(void)
{
    rt_err_t rst;
    rst = uart_rs485_init();
    if (rst != RT_EOK)
        return rst;

    rt_thread_t t = rt_thread_create("rs485tx", rs485_tx_handler, RT_NULL, 1024 * 4, 12, 20);
    if (t == RT_NULL)
        return -RT_ENOMEM;

    rst = rt_thread_startup(t);

    return rst;
}
INIT_APP_EXPORT(app_uart_rs485_startup);
```

##### 从机接收响应端（从）：
###### uart_slave.c

```
# include <rtthread.h>
# include <rtdevice.h>
# include <board.h>
# include <stdio.h>
#include "uart_framework.h"

#define LOG_TAG              "uart.rx"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

#define UART_NAME           "uart1"
#define UART_BAUD_RATE      BAUD_RATE_115200

#define RS485RD             GPIOA_10
#define RS485_RX()          rt_pin_write(RS485RD, 0)
#define RS485_TX()          rt_pin_write(RS485RD, 1)

#define RECV_BUF_SIZE       (512)
#define FRAME_TIMEOUT_MS    20
#define SEND_INTERVAL_MS    0
#define REQUEST_TIMEOUT_MS  250

static uart_framework_t uf = RT_NULL;

static void rs485_set_tx(void)
{
    RS485_TX();
    rt_thread_mdelay(1);
}

static void rs485_set_rx(void)
{
    rt_thread_mdelay(1);
    RS485_RX();
}

static void rs485_gpio_init(void)
{
    rt_pin_mode(RS485RD, PIN_MODE_OUTPUT);
    rs485_set_rx();
}

static rt_err_t uart_rs485_init(void)
{
    rt_device_t serial_device = rt_device_find(UART_NAME);
    if (serial_device == RT_NULL)
    {
        LOG_E("not find the serial device:%s!", UART_NAME);
        return -RT_ERROR;
    }

    /* 修改串口配置 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = UART_BAUD_RATE;
    config.rx_bufsz = RECV_BUF_SIZE;
    config.tx_bufsz = 0;    //必须重新配置为0：阻塞发送，非阻塞接收
    rt_device_control(serial_device, RT_DEVICE_CTRL_CONFIG, &config);

    rs485_gpio_init();
    struct uart_framework_cfg cfg = { .uart_name = UART_NAME, .max_frame_size = RECV_BUF_SIZE, .frame_interval_ms =
    FRAME_TIMEOUT_MS, .send_interval_ms = SEND_INTERVAL_MS, .rs485_txd = rs485_set_tx, .rs485_rxd = rs485_set_rx };
    //如果不是485，则配置rs485_txd和rs485_rxd为RT_NULL

    uf = uart_framework_create(&cfg);
    if (uf == RT_NULL)
    {
        LOG_E("uart_framework_create failed on %s", UART_NAME);
        return -RT_ERROR;
    }
    else
    {
        LOG_I("uart_framework_create success on %s!", UART_NAME);
        return RT_EOK;
    }
}

static rt_err_t frame_handler(rt_uint8_t *data, rt_size_t size)
{
    rt_uint8_t *frame_data = data;
    rt_uint16_t frame_len = size;
    //解析数据
    //这里直接返回接收到的数据
    LOG_HEX("rx_buf", 16, frame_data, frame_len);
    rt_err_t res = uart_framework_send(uf, frame_data, frame_len);

    return res;
}

static void rs485_rx_handler(void *params)
{
    while (1)
    {
        uart_framework_receive(uf, RT_WAITING_FOREVER, frame_handler, RT_NULL, 0);
    }
}

int app_uart_rs485_startup(void)
{
    rt_err_t rst;
    rst = uart_rs485_init();
    if (rst != RT_EOK)
        return rst;

    rt_thread_t t = rt_thread_create("rs485rx", rs485_rx_handler, RT_NULL, 1024 * 4, 12, 20);
    if (t == RT_NULL)
        return -RT_ENOMEM;

    rst = rt_thread_startup(t);

    return rst;
}
INIT_APP_EXPORT(app_uart_rs485_startup);

```
