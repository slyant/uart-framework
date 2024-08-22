# 基于rt-thread的串口通讯框架

#### 介绍
基于rt-thread的串口通讯框架

#### 使用说明

```
# include <rtthread.h>
# include <rtdevice.h>
# include <board.h>
# include <stdio.h>
# include "uart_framework.h"

# define LOG_TAG              "uart.xxx"
# define LOG_LVL              LOG_LVL_INFO
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

rt_err_t uart_send(char *text)
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
        if (uf->rx_size < uf->cfg.max_frame_size)
            uf->rx_buf[uf->rx_size] = '\0';
        if (rt_strcmp("test", (const char*) uf->rx_buf) == 0)
        {
            res = RT_EOK;
            LOG_D("test OK");
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

static rt_err_t rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(uf->rx_sem);
    return RT_EOK;
}

int uart_init(void)
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

    struct uart_framework_cfg cfg =
            { .uart_name = UART_NAME, .max_frame_size = RECV_BUF_SIZE, .frame_interval_ms =
            FRAME_TIMEOUT_MS, .send_interval_ms = SEND_INTERVAL_MS, .rs485_txd = rs485_set_tx, .rs485_rxd = rs485_set_rx,
                    .rx_ind = rx_ind };
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
```
