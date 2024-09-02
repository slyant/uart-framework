/*
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-24     Slyant       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <uart_framework.h>

enum work_model
{
  WORK_NORMAL = 0,
  WORK_TAKE,
  WORK_RELEASE,
  WORK_TAKE_RELEASE
};

static rt_err_t rx_ind(rt_device_t dev, rt_size_t size)
{
  uart_framework_t uf = (uart_framework_t)dev->user_data;
  rt_sem_release(uf->rx_sem);
  return RT_EOK;
}

/**
 * @brief 创建 UART 框架
 *
 * 根据给定的 UART 框架配置，创建并初始化 UART 框架。
 *
 * @param cfg UART 框架配置指针
 *
 * @return UART 框架指针，如果创建失败则返回 RT_NULL
 */
uart_framework_t uart_framework_create(struct uart_framework_cfg *cfg)
{
  rt_err_t open_result;
  static int rx_sem_count = 0;
  static int rx_mut_count = 0;
  char ufsem_name[RT_NAME_MAX] = {0};
  char ufmut_name[RT_NAME_MAX] = {0};
  uart_framework_t uf = rt_calloc(1, sizeof(struct uart_framework));
  if (uf == RT_NULL)
    return RT_NULL;
  uf->uart_device = rt_device_find(cfg->uart_name);
  if (uf->uart_device == RT_NULL || uf->uart_device->type != RT_Device_Class_Char)
  {
    rt_free(uf);
    return RT_NULL;
  }
  rt_snprintf(ufsem_name, RT_NAME_MAX, "ufsem%d", rx_sem_count++);
  uf->rx_sem = rt_sem_create(ufsem_name, 0, RT_IPC_FLAG_FIFO);
  if (uf->rx_sem == RT_NULL)
  {
    rt_free(uf);
    return RT_NULL;
  }
  rt_snprintf(ufmut_name, RT_NAME_MAX, "ufmut%d", rx_mut_count++);
  uf->dev_lock = rt_mutex_create(ufmut_name, RT_IPC_FLAG_FIFO);
  if (uf->dev_lock == RT_NULL)
  {
    rt_sem_delete(uf->rx_sem);
    rt_free(uf);
    return RT_NULL;
  }
  uf->rx_buf = rt_calloc(1, cfg->max_frame_size);
  if (uf->rx_buf == RT_NULL)
  {
    rt_sem_delete(uf->rx_sem);
    rt_mutex_delete(uf->dev_lock);
    rt_free(uf);
    return RT_NULL;
  }
  rt_memcpy(&uf->cfg, cfg, sizeof(struct uart_framework_cfg));

#ifdef RT_USING_SERIAL_V2
  open_result = rt_device_open(uf->uart_device, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
#else
#ifdef RT_SERIAL_USING_DMA
  /* using DMA mode first */
  open_result = rt_device_open(uf->uart_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
  /* using interrupt mode when DMA mode not supported */
  if (open_result == RT_EOK)
  {
  }
  else if (open_result == -RT_EIO)
#endif
  {
    open_result = rt_device_open(uf->uart_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
  }
#endif
  RT_ASSERT(open_result == RT_EOK);

  uf->uart_device->user_data = uf;
  if (uf->cfg.rx_ind)
  {
    rt_device_set_rx_indicate(uf->uart_device, uf->cfg.rx_ind);
  }
  else
  {
    rt_device_set_rx_indicate(uf->uart_device, rx_ind);
  }

  return uf;
}

/**
 * @brief 发送数据
 *
 * 根据给定的工作模式和 UART 框架，发送数据到 UART 设备。
 *
 * @param model 工作模式
 * @param uf UART 框架结构体指针
 * @param data 数据指针
 * @param size 数据大小
 *
 * @return 实际发送的字节数
 */
static rt_size_t _send(enum work_model model, uart_framework_t uf, rt_uint8_t *data, rt_size_t size)
{
  while (rt_tick_get() - uf->send_tick < rt_tick_from_millisecond(uf->cfg.send_interval_ms))
  {
    rt_thread_mdelay(10);
  }
  if (model == WORK_TAKE || model == WORK_TAKE_RELEASE)
  {
    rt_mutex_take(uf->dev_lock, RT_WAITING_FOREVER);
    rt_sem_control(uf->rx_sem, RT_IPC_CMD_RESET, 0);
    while (rt_device_read(uf->uart_device, 0, &uf->rx_ch, 1))
      ;
  }
  if (uf->cfg.rs485_txd)
    uf->cfg.rs485_txd();
  rt_size_t wsize = rt_device_write(uf->uart_device, 0, data, size);
  if (uf->cfg.rs485_rxd)
    uf->cfg.rs485_rxd();
  uf->send_tick = rt_tick_get(); // 重置定时器
  if (model == WORK_TAKE_RELEASE)
  {
    rt_mutex_release(uf->dev_lock);
  }
  return wsize;
}

/**
 * @brief 接收串口数据
 *
 * 根据指定的工作模型，从串口框架中接收数据，并处理接收到的数据帧。
 *
 * @param model 工作模型
 * @param uf 串口框架对象
 * @param timeout_ms 超时时间（毫秒）
 * @param frame_handler 数据帧处理函数
 * @param out 输出缓冲区
 * @param out_max_size 输出缓冲区最大大小
 *
 * @return 错误码，RT_EOK 表示成功，其他值表示错误
 */
static rt_err_t _receive(enum work_model model, uart_framework_t uf, rt_uint32_t timeout_ms,
                         rt_err_t (*frame_handler)(rt_uint8_t *data, rt_size_t size), rt_uint8_t *out, rt_size_t out_max_size)
{
  //    rt_memset(uf->rx_buf, 0, uf->cfg.max_frame_size);
  uf->rx_size = 0;
  uf->last_tick = rt_tick_get();
  while (1)
  {
    if (rt_sem_take(uf->rx_sem, rt_tick_from_millisecond(timeout_ms)) == RT_EOK)
    {
      if (model == WORK_TAKE_RELEASE)
      {
        rt_mutex_take(uf->dev_lock, RT_WAITING_FOREVER);
      }
      while (1)
      {
        while (rt_device_read(uf->uart_device, 0, &uf->rx_ch, 1))
        {
          uf->last_tick = rt_tick_get(); // 重置定时器
          if (uf->rx_size < uf->cfg.max_frame_size)
          {
            uf->rx_buf[uf->rx_size++] = uf->rx_ch;
          }
        }
        if (rt_tick_get() - uf->last_tick > rt_tick_from_millisecond(uf->cfg.frame_interval_ms))
        {
          if (uf->rx_size > 0)
          {
            if (out && out_max_size > 0)
            {
              rt_memcpy(out, uf->rx_buf, uf->rx_size > out_max_size ? out_max_size : uf->rx_size);
            }
            if (frame_handler)
            {
              rt_err_t err = frame_handler(uf->rx_buf, uf->rx_size);
              if (model == WORK_RELEASE || model == WORK_TAKE_RELEASE)
              {
                rt_mutex_release(uf->dev_lock);
              }
              return err;
            }
            else
            {
              if (model == WORK_RELEASE || model == WORK_TAKE_RELEASE)
              {
                rt_mutex_release(uf->dev_lock);
              }
              return RT_EOK;
            }
          }
          else
          {
            break;
          }
        }
        rt_thread_mdelay(10);
      }
      if (model == WORK_RELEASE || model == WORK_TAKE_RELEASE)
      {
        rt_mutex_release(uf->dev_lock);
      }
    }
    else
    {
      if (model == WORK_RELEASE)
      {
        rt_mutex_release(uf->dev_lock);
      }
      return -RT_ETIMEOUT;
    }
  }
}

/**
 * @brief 发送数据到UART框架
 *
 * 通过UART框架发送指定大小的数据。
 *
 * @param uf UART框架对象指针
 * @param data 要发送的数据指针
 * @param size 数据大小
 *
 * @return 发送的字节数
 */
rt_size_t uart_framework_send(uart_framework_t uf, rt_uint8_t *data, rt_size_t size)
{
  return _send(WORK_NORMAL, uf, data, size);
}
/**
 * @brief UART框架发送数据（独占）
 *
 * 通过UART框架发送指定大小的数据。
 *
 * @param uf UART框架对象
 * @param data 要发送的数据指针
 * @param size 要发送的数据大小
 *
 * @return 实际发送的数据大小
 */
rt_size_t uart_framework_send_take(uart_framework_t uf, rt_uint8_t *data, rt_size_t size)
{
  return _send(WORK_TAKE, uf, data, size);
}
/**
 * @brief UART框架发送数据（独占发送后释放）
 *
 * 通过UART框架发送指定大小的数据。
 *
 * @param uf UART框架对象指针
 * @param data 要发送的数据指针
 * @param size 要发送的数据大小
 *
 * @return 实际发送的字节数
 */
rt_size_t uart_framework_send_take_release(uart_framework_t uf, rt_uint8_t *data, rt_size_t size)
{
  return _send(WORK_TAKE_RELEASE, uf, data, size);
}

/**
 * @brief 接收UART框架数据
 *
 * 在给定的超时时间内，从UART框架中接收数据，并调用帧处理器处理接收到的数据。
 *
 * @param uf UART框架对象
 * @param timeout_ms 超时时间（毫秒）
 * @param frame_handler 帧处理器函数指针，用于处理接收到的数据
 * @param out 存储处理结果的缓冲区
 * @param out_max_size 缓冲区最大容量
 *
 * @return 返回错误码，表示操作是否成功
 */
rt_err_t uart_framework_receive(uart_framework_t uf, rt_uint32_t timeout_ms,
                                rt_err_t (*frame_handler)(rt_uint8_t *data, rt_size_t size), rt_uint8_t *out, rt_size_t out_max_size)
{
  return _receive(WORK_NORMAL, uf, timeout_ms, frame_handler, out, out_max_size);
}

/**
 * @brief UART 框架接收数据（接收后释放独占）
 *
 * 从 UART 框架收数据，通过回调函数处理接收到的数据帧。
 *
 * @param uf UART 框架对象
 * @param timeout_ms 超时时间（毫秒）
 * @param frame_handler 数据帧处理回调函数
 * @param out 存储处理结果的缓冲区
 * @param out_max_size 缓冲区最大大小
 *
 * @return 返回错误码，表示操作结果
 */
rt_err_t uart_framework_receive_release(uart_framework_t uf, rt_uint32_t timeout_ms,
                                        rt_err_t (*frame_handler)(rt_uint8_t *data, rt_size_t size), rt_uint8_t *out, rt_size_t out_max_size)
{
  return _receive(WORK_RELEASE, uf, timeout_ms, frame_handler, out, out_max_size);
}

/**
 * @brief UART框架接收数据（独占接收后释放）
 *
 * 从UART框架中接收数据，并在处理完成后释放相关资源。
 *
 * @param uf UART框架指针
 * @param timeout_ms 超时时间（单位：毫秒）
 * @param frame_handler 帧处理函数指针
 * @param out 输出缓冲区指针
 * @param out_max_size 输出缓冲区最大大小
 *
 * @return 返回错误码，表示操作是否成功
 */
rt_err_t uart_framework_receive_take_release(uart_framework_t uf, rt_uint32_t timeout_ms,
                                             rt_err_t (*frame_handler)(rt_uint8_t *data, rt_size_t size), rt_uint8_t *out, rt_size_t out_max_size)
{
  return _receive(WORK_TAKE_RELEASE, uf, timeout_ms, frame_handler, out, out_max_size);
}
