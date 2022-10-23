/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include <rtthread.h>
#include "mlx9039x.h"

/* Default configuration, please change according to the actual situation, support i2c and spi device name */
#define MLX9039x_DEVICE_NAME  "i2c2"

/* Test function */
static int mlx9039x_test()
{
    struct mlx9039x_device *dev;

    /* Initialize mlx9039x, The parameter is RT_NULL, means auto probing for i2c*/
    dev = mlx9039x_init(MLX9039x_DEVICE_NAME, RT_NULL);

    if (dev == RT_NULL)
    {
        rt_kprintf("mlx9039x init failed\n");
        return -1;
    }
    rt_kprintf("mlx9039x init succeed\n");

    for (int i = 0; i < 5; i++)
    {
        rt_thread_mdelay(100);
    }

    mlx9039x_deinit(dev);

    return 0;
}
MSH_CMD_EXPORT(mlx9039x_test, mlx9039x sensor test function);
