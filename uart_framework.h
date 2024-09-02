/*
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-24     Slyant       the first version
 */
#ifndef _UART_FRAMEWORK_H_
#define _UART_FRAMEWORK_H_

struct uart_framework_cfg
{
  char *uart_name;
  rt_size_t max_frame_size;
  rt_uint32_t frame_interval_ms;
  rt_uint32_t send_interval_ms;
  rt_err_t (*rx_ind)(rt_device_t dev, rt_size_t size);
  void (*rs485_txd)(void);
  void (*rs485_rxd)(void);
};

struct uart_framework
{
  rt_device_t uart_device;
  rt_uint8_t *rx_buf;
  rt_uint8_t rx_ch;
  rt_size_t rx_size;
  rt_sem_t rx_sem;
  rt_mutex_t dev_lock;
  rt_tick_t last_tick;
  rt_tick_t cur_tick;
  rt_tick_t send_tick;
  struct uart_framework_cfg cfg;
};
typedef struct uart_framework *uart_framework_t;

uart_framework_t uart_framework_create(struct uart_framework_cfg *cfg);
rt_size_t uart_framework_send(uart_framework_t uf, rt_uint8_t *data, rt_size_t size);
rt_size_t uart_framework_send_take(uart_framework_t uf, rt_uint8_t *data, rt_size_t size);
rt_size_t uart_framework_send_take_release(uart_framework_t uf, rt_uint8_t *data, rt_size_t size);
rt_err_t uart_framework_receive(uart_framework_t uf, rt_uint32_t timeout_ms,
                                rt_err_t (*frame_handler)(rt_uint8_t *data, rt_size_t size), rt_uint8_t *out, rt_size_t out_max_size);
rt_err_t uart_framework_receive_release(uart_framework_t uf, rt_uint32_t timeout_ms,
                                        rt_err_t (*frame_handler)(rt_uint8_t *data, rt_size_t size), rt_uint8_t *out, rt_size_t out_max_size);
rt_err_t uart_framework_receive_take_release(uart_framework_t uf, rt_uint32_t timeout_ms,
                                             rt_err_t (*frame_handler)(rt_uint8_t *data, rt_size_t size), rt_uint8_t *out, rt_size_t out_max_size);

#endif /* _UART_FRAMEWORK_H_ */
