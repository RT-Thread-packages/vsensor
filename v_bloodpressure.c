/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-23     zhangsz      add virtual sensor device
 */

#include <rtthread.h>

#ifdef PKG_USING_VIRTUAL_SENSOR_BP

#include "sensor.h"
#include <stdlib.h>

#define DBG_TAG    "v_bp"
#ifdef PKG_USING_VIRTUAL_SENSOR_DBG
    #define DBG_LVL    DBG_LOG
#else
    #define DBG_LVL    DBG_INFO
#endif
#include <rtdbg.h>

enum SENS_BP_ID
{
    SENS_BP_01 = 0, //BP
    SENS_BP_MAX,
};

#define SENS_BUS_NAME                       "sens_bus"
#define SENS_BP_01_SENSOR_ID                 (RT_SENSOR_CLASS_BP + 0x10)

struct _sens_bp
{
    char* dev_name;
    rt_uint8_t sens_id;
};

static struct _sens_bp _sens_tbl[SENS_BP_MAX] =
{
    {V_SENS_BP_DEV_NAME,                0x00 }, /* BloodPressure */
};

static struct rt_sensor_info _sens_info_tbl[SENS_BP_MAX] =
{
    {RT_SENSOR_CLASS_BP,  RT_SENSOR_VENDOR_STM,   RT_NULL,    RT_SENSOR_UNIT_MMHG,      RT_SENSOR_INTF_SPI,     200,   0,   1 },
};

static rt_uint8_t sensor_get_id(rt_uint8_t sens_index)
{
    rt_uint8_t chip_id = 0x00;

    switch (sens_index)
    {
    case SENS_BP_01:
        chip_id = SENS_BP_01_SENSOR_ID;
        break;
    default:
        break;
    }

    return chip_id;
}

static int sensor_init(rt_uint8_t index)
{
    _sens_tbl[index].sens_id = sensor_get_id(index);

    return RT_EOK;
}

static void* bp_sensor_create(struct rt_sensor_intf* intf, rt_uint8_t index)
{
    if (sensor_init(index) != RT_EOK)
    {
        LOG_E("%s:error!", __func__);
    }

    return 0;
}

static rt_err_t bp_sensor_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    LOG_D("%s:odr=%d", __func__, odr);
    return RT_EOK;
}

static rt_err_t bp_sensor_set_range(rt_sensor_t sensor, rt_uint16_t range)
{
    LOG_D("%s:range=%d", __func__, range);
    return RT_EOK;
}

static rt_err_t bp_sensor_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    rt_int8_t rslt = 0;
    LOG_D("%s:power=%d", __func__, power);
    return rslt;
}

static rt_size_t bp_sensor_fetch_data(struct rt_sensor_device* sensor, void* buf, rt_size_t size)
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

    max_range = _sens_info_tbl[SENS_BP_01].range_max - _sens_info_tbl[SENS_BP_01].range_min;

    for (int i = 0; i < size; i++)
    {
        data->type = RT_SENSOR_CLASS_BP;
        data->data.bp.sbp = rand() % max_range + _sens_info_tbl[SENS_BP_01].range_min;
        data->data.bp.dbp = rand() % max_range + _sens_info_tbl[SENS_BP_01].range_min;
        data->timestamp = rt_sensor_get_ts();
        LOG_D("%s:[%d,%d]", __func__, data->data.bp.sbp, data->data.bp.dbp);
        data++;
    }

    return size;
}

static rt_err_t bp_sensor_control(struct rt_sensor_device* sensor, int cmd, void* args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(rt_uint8_t*)args = _sens_tbl[SENS_BP_01].sens_id;
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = bp_sensor_set_odr(sensor, (rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = bp_sensor_set_range(sensor, (rt_uint32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = bp_sensor_set_power(sensor, (rt_uint32_t)args & 0xff);
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
    {bp_sensor_fetch_data, bp_sensor_control},
};

int rt_vd_sens_bp_init(void)
{
    rt_int8_t result;
    rt_uint8_t index = 0;
    rt_sensor_t sensor_dat = RT_NULL;
    struct rt_sensor_config cfg;

    cfg.intf.dev_name = SENS_BUS_NAME;
    cfg.intf.user_data = RT_NULL;
    cfg.irq_pin.pin = RT_PIN_NONE;

    for (index = 0; index < SENS_BP_MAX; index++)
    {
        bp_sensor_create(&cfg.intf, index);
        sensor_dat = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_dat == RT_NULL)
        {
            LOG_E("%s:rt_calloc err!", __func__);
            return -RT_ERROR;
        }

        sensor_dat->info.type = _sens_info_tbl[index].type;
        sensor_dat->info.vendor = _sens_info_tbl[index].vendor;
        sensor_dat->info.model = _sens_info_tbl[index].model;
        sensor_dat->info.unit = _sens_info_tbl[index].unit;
        sensor_dat->info.intf_type = _sens_info_tbl[index].intf_type;
        sensor_dat->info.range_max = _sens_info_tbl[index].range_max;
        sensor_dat->info.range_min = _sens_info_tbl[index].range_min;
        sensor_dat->info.period_min = _sens_info_tbl[index].period_min;

        rt_memcpy(&sensor_dat->config, &cfg, sizeof(struct rt_sensor_config));
        sensor_dat->ops = &sensor_ops[index];

        result = rt_hw_sensor_register(sensor_dat, _sens_tbl[index].dev_name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("%s:device register err! code=%d", __func__, result);
            rt_free(sensor_dat);
            return -RT_ERROR;
        }
    }

    return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_vd_sens_bp_init);

#endif

