/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-12-19     zhangsz      add virtual sensor device
 */

#include <rtthread.h>

#ifdef PKG_USING_VIRTUAL_SENSOR_TEMP

#include "sensor.h"
#include <stdlib.h>

#define DBG_TAG    "v_temp"
#ifdef PKG_USING_VIRTUAL_SENSOR_DBG
    #define DBG_LVL    DBG_LOG
#else
    #define DBG_LVL    DBG_INFO
#endif
#include <rtdbg.h>

enum SENS_TEMP_ID
{
    SENS_TEMP_01 = 0, //Temperature
    SENS_TEMP_MAX,
};

#define SENS_BUS_NAME                       "sens_bus"
#define SENS_TEMP_01_SENSOR_ID                 (RT_SENSOR_CLASS_TEMP + 0x10)

struct sens_temp
{
    char* dev_name;
    rt_uint8_t sens_id;
};

static struct sens_temp sens_temp_tbl[SENS_TEMP_MAX] =
{
    {V_SENS_TEMP_DEV_NAME,         0x00 }, /* Temperature */
};

static struct rt_sensor_info temp_info_tbl[SENS_TEMP_MAX] =
{
    {RT_SENSOR_CLASS_TEMP,  RT_SENSOR_VENDOR_DALLAS,   RT_NULL,    RT_SENSOR_UNIT_DCELSIUS,      RT_SENSOR_INTF_ONEWIRE,     125,   -45,   1 },
};

static rt_uint8_t sensor_get_id(rt_uint8_t sens_index)
{
    rt_uint8_t chip_id = 0x00;

    switch (sens_index)
    {
    case SENS_TEMP_01:
        chip_id = SENS_TEMP_01_SENSOR_ID;
        break;
    default:
        break;
    }

    return chip_id;
}

static int sensor_init(rt_uint8_t index)
{
    sens_temp_tbl[index].sens_id = sensor_get_id(index);

    return RT_EOK;
}

static void* temp_sensor_create(struct rt_sensor_intf* intf, rt_uint8_t index)
{
    if (sensor_init(index) != RT_EOK)
    {
        LOG_E("%s:error!", __func__);
    }

    return 0;
}

static rt_err_t temp_sensor_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    LOG_D("%s:odr=%d", __func__, odr);
    return RT_EOK;
}

static rt_err_t temp_sensor_set_range(rt_sensor_t sensor, rt_uint16_t range)
{
    LOG_D("%s:range=%d", __func__, range);
    return RT_EOK;
}

static rt_err_t temp_sensor_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    rt_int8_t rslt = 0;
    LOG_D("%s:power=%d", __func__, power);
    return rslt;
}

static rt_size_t temp_sensor_fetch_data(struct rt_sensor_device* sensor, void* buf, rt_size_t size)
{
    struct rt_sensor_data* data = buf;
    rt_int16_t max_range = 0;

    if (size < 1)
    {
        LOG_E("%s:read size err! size=%d", __func__, size);
        return 0;
    }

    if (buf == RT_NULL)
    {
        LOG_E("%s:read buf is NULL!", __func__);
        return 0;
    }

    max_range = temp_info_tbl[SENS_TEMP_01].range_max - temp_info_tbl[SENS_TEMP_01].range_min;

    for (int i = 0; i < size; i++)
    {
        data->type = RT_SENSOR_CLASS_TEMP;
        data->data.temp = rand() % max_range + temp_info_tbl[SENS_TEMP_01].range_min;
        data->timestamp = rt_sensor_get_ts();
        LOG_D("%s:%d", __func__, data->data.temp);
        data++;
    }

    return size;
}

static rt_err_t temp_sensor_control(struct rt_sensor_device* sensor, int cmd, void* args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(rt_uint8_t*)args = sens_temp_tbl[SENS_TEMP_01].sens_id;
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = temp_sensor_set_odr(sensor, (rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = temp_sensor_set_range(sensor, (rt_uint32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = temp_sensor_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        /* TODO */
        result = -RT_EINVAL;
        break;
    default:
        return -RT_EINVAL;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops[] =
{
    {temp_sensor_fetch_data, temp_sensor_control},
};

int rt_vd_sens_temp_init(void)
{
    rt_int8_t result;
    rt_uint8_t index = 0;
    rt_sensor_t sensor_dat = RT_NULL;
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = SENS_BUS_NAME;
    cfg.intf.user_data = RT_NULL;
    cfg.irq_pin.pin = RT_PIN_NONE;

    for (index = 0; index < SENS_TEMP_MAX; index++)
    {
        temp_sensor_create(&cfg.intf, index);
        sensor_dat = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_dat == RT_NULL)
        {
            LOG_E("%s:rt_calloc err!", __func__);
            return -RT_ERROR;
        }

        sensor_dat->info.type = temp_info_tbl[index].type;
        sensor_dat->info.vendor = temp_info_tbl[index].vendor;
        sensor_dat->info.model = temp_info_tbl[index].model;
        sensor_dat->info.unit = temp_info_tbl[index].unit;
        sensor_dat->info.intf_type = temp_info_tbl[index].intf_type;
        sensor_dat->info.range_max = temp_info_tbl[index].range_max;
        sensor_dat->info.range_min = temp_info_tbl[index].range_min;
        sensor_dat->info.period_min = temp_info_tbl[index].period_min;

        rt_memcpy(&sensor_dat->config, &cfg, sizeof(struct rt_sensor_config));
        sensor_dat->ops = &sensor_ops[index];

        result = rt_hw_sensor_register(sensor_dat, sens_temp_tbl[index].dev_name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("%s:device register err code: %d", __func__, result);
            rt_free(sensor_dat);
            return -RT_ERROR;
        }
    }

    return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_vd_sens_temp_init);

#endif

