/*
 * aw88194.c   aw88194 codec module
 *
 * Version: v1.0.5
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/syscalls.h>
#include <sound/tlv.h>
#include "aw88194.h"
#include "aw88194_reg.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW88194_I2C_NAME "aw88194_smartpa"

#define AW88194_VERSION "v1.0.5"

#define AW88194_RATES SNDRV_PCM_RATE_8000_48000
#define AW88194_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
                                    SNDRV_PCM_FMTBIT_S24_LE | \
                                    SNDRV_PCM_FMTBIT_S32_LE)


#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 5  // 5ms
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 5

/******************************************************
 *
 * Value
 *
 ******************************************************/
static int aw88194_spk_control = 0;
static int aw88194_rcv_control = 0;

#define AW88194_MAX_FIRMWARE_LOAD_CNT 20
#define AW88194_CFG_NAME_MAX    64
static char aw88194_cfg_name[][AW88194_CFG_NAME_MAX] = {
    {"aw88194_spk_reg.bin"},
    {"aw88194_spk_fw.bin"},
    {"aw88194_spk_cfg.bin"},
    {"aw88194_rcv_reg.bin"},
    {"aw88194_rcv_fw.bin"},
    {"aw88194_rcv_cfg.bin"},
};

static unsigned int aw88194_base_addr[AW88194_BASE_ADDR_MAX] = {
    AW88194_REG_ADDR,
    AW88194_DSP_FW_ADDR,
    AW88194_DSP_CFG_ADDR,
    AW88194_REG_ADDR,
    AW88194_DSP_FW_ADDR,
    AW88194_DSP_CFG_ADDR,
};

static unsigned int aw88194_mode_cfg_shift[AW88194_MODE_SHIFT_MAX] = {
    AW88194_MODE_SPK_SHIFT,
    AW88194_MODE_RCV_SHIFT,
};

/******************************************************
 *
 * Functions
 *
 ******************************************************/
static int aw88194_vbat_monitor_start(struct aw88194 *aw88194);
static int aw88194_vbat_monitor_stop(struct aw88194 *aw88194);
static int aw88194_get_bst_ipeak(struct aw88194 *aw88194);

/******************************************************
 *
 * aw88194 i2c write/read
 *
 ******************************************************/
static int i2c_write(struct aw88194 *aw88194,
        unsigned char addr, unsigned int reg_data)
{
    int ret = -1;
    u8 wbuf[512] = {0};

    struct i2c_msg msgs[] = {
        {
            .addr   = aw88194->i2c->addr,
            .flags  = 0,
            .len    = 3,
            .buf    = wbuf,
        },
    };

    wbuf[0] = addr;
    wbuf[1] = (unsigned char)((reg_data & 0xff00)>>8);
    wbuf[2] = (unsigned char)(reg_data & 0x00ff);

    ret = i2c_transfer(aw88194->i2c->adapter, msgs, 1);
    if (ret < 0) {
        pr_err("%s: i2c write error: %d\n", __func__, ret);
    }

    return ret;
}

static int i2c_read(struct aw88194 *aw88194,
        unsigned char addr, unsigned int *reg_data)
{
    int ret = -1;
    unsigned char rbuf[512] = {0};
    unsigned int get_data = 0;

    struct i2c_msg msgs[] = {
        {
            .addr   = aw88194->i2c->addr,
            .flags  = 0,
            .len    = 1,
            .buf    = &addr,
        },
        {
            .addr   = aw88194->i2c->addr,
            .flags  = I2C_M_RD,
            .len    = 2,
            .buf    = rbuf,
        },
    };

    ret = i2c_transfer(aw88194->i2c->adapter, msgs, 2);
    if (ret < 0) {
        pr_err("%s: i2c read error: %d\n", __func__, ret);
        return ret;
    }

    get_data = (unsigned int)(rbuf[0] & 0x00ff);
    get_data <<= 8;
    get_data |= (unsigned int)rbuf[1];

    *reg_data = get_data;

    return ret;
}

static int aw88194_i2c_write(struct aw88194 *aw88194,
        unsigned char reg_addr, unsigned int reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
        ret = i2c_write(aw88194, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
            cnt ++;
    }

    return ret;
}

static int aw88194_i2c_read(struct aw88194 *aw88194,
        unsigned char reg_addr, unsigned int *reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;

    while(cnt < AW_I2C_RETRIES) {
        ret = i2c_read(aw88194, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
        cnt ++;
    }

    return ret;
}

static int aw88194_i2c_write_bits(struct aw88194 *aw88194,
        unsigned char reg_addr, unsigned int mask, unsigned int reg_data)
{
    unsigned int reg_val = 0;

    aw88194_i2c_read(aw88194, reg_addr, &reg_val);
    reg_val &= mask;
    reg_val |= reg_data;
    aw88194_i2c_write(aw88194, reg_addr, reg_val);

    return 0;
}

/******************************************************
 *
 * aw88194 cali store
 *
 ******************************************************/
static int aw88194_get_cali_re(struct aw88194 *aw88194)
{
    /* get cali re from phone */

    /* user the default cali re */
    if(0 == aw88194->cali_re) {
        aw88194->cali_re = AW88194_DFT_CALI_RE;
        pr_debug("%s find no re, use default re=0x%x\n",
            __func__, aw88194->cali_re);
    }

    return 0;
}

static int aw88194_set_cali_re(struct aw88194 *aw88194)
{
    unsigned char reg_addr = 0;
    unsigned int reg_val = 0;

    /* set cali re to phone */

    /* set cali re to aw88194 */
    reg_addr = AW88194_REG_DSPMADD;
    reg_val = AW88194_DSP_REG_CFG_ADPZ_RE;
    aw88194_i2c_write(aw88194, reg_addr, reg_val);
    reg_addr = AW88194_REG_DSPMDAT;
    reg_val = aw88194->cali_re;
    aw88194_i2c_write(aw88194, reg_addr, reg_val);

    return 0;
}


static int aw88194_dsp_update_cali_re(struct aw88194 *aw88194)
{
    aw88194_get_cali_re(aw88194);
    aw88194_set_cali_re(aw88194);

    return 0;
}

/******************************************************
 *
 * aw88194 control
 *
 ******************************************************/
static void aw88194_run_mute(struct aw88194 *aw88194, bool mute)
{
    pr_debug("%s enter\n", __func__);

    if(mute) {
        aw88194_i2c_write_bits(aw88194, AW88194_REG_PWMCTRL,
                AW88194_BIT_PWMCTRL_HMUTE_MASK,
                AW88194_BIT_PWMCTRL_HMUTE_ENABLE);
    } else {
        aw88194_i2c_write_bits(aw88194, AW88194_REG_PWMCTRL,
                AW88194_BIT_PWMCTRL_HMUTE_MASK,
                AW88194_BIT_PWMCTRL_HMUTE_DISABLE);
    }
}

static void aw88194_run_pwd(struct aw88194 *aw88194, bool pwd)
{
    pr_debug("%s enter\n", __func__);

    if(pwd) {
        aw88194_i2c_write_bits(aw88194, AW88194_REG_SYSCTRL,
                AW88194_BIT_SYSCTRL_PW_MASK,
                AW88194_BIT_SYSCTRL_PW_PDN);
    } else {
        aw88194_i2c_write_bits(aw88194, AW88194_REG_SYSCTRL,
                AW88194_BIT_SYSCTRL_PW_MASK,
                AW88194_BIT_SYSCTRL_PW_ACTIVE);
    }
}

static void aw88194_dsp_enable(struct aw88194 *aw88194, bool dsp)
{
    pr_debug("%s enter\n", __func__);

    if(dsp) {
        aw88194_i2c_write_bits(aw88194, AW88194_REG_SYSCTRL,
                AW88194_BIT_SYSCTRL_DSP_MASK,
                AW88194_BIT_SYSCTRL_DSP_WORK);
    } else {
        aw88194_i2c_write_bits(aw88194, AW88194_REG_SYSCTRL,
                AW88194_BIT_SYSCTRL_DSP_MASK,
                AW88194_BIT_SYSCTRL_DSP_BYPASS);
    }
}
/*
static void aw88194_spk_rcv_mode(struct aw88194 *aw88194)
{
    pr_debug("%s spk_rcv=%d\n", __func__, aw88194->spk_rcv_mode);

    if(aw88194->spk_rcv_mode == AW88194_SPEAKER_MODE) {
        aw88194_i2c_write_bits(aw88194, AW88194_REG_SYSCTRL,
                AW88194_BIT_SYSCTRL_MODE_MASK,
                AW88194_BIT_SYSCTRL_SPK_MODE);
    } else if(aw88194->spk_rcv_mode == AW88194_RECEIVER_MODE){
        aw88194_i2c_write_bits(aw88194, AW88194_REG_SYSCTRL,
                AW88194_BIT_SYSCTRL_MODE_MASK,
                AW88194_BIT_SYSCTRL_RCV_MODE);
    } else {
        pr_err("%s: unknown mode=%d\n", __func__, aw88194->spk_rcv_mode);
    }
}
*/

static void aw88194_start(struct aw88194 *aw88194)
{
    pr_debug("%s enter\n", __func__);

    aw88194_run_pwd(aw88194, false);
    aw88194_run_mute(aw88194, false);

    if(aw88194->vbat_monitor_flag) {
        aw88194_vbat_monitor_start(aw88194);
    }
}

static void aw88194_stop(struct aw88194 *aw88194)
{
    pr_debug("%s enter\n", __func__);

    aw88194_run_mute(aw88194, true);
    aw88194_run_pwd(aw88194, true);

    if(aw88194->vbat_monitor_flag) {
        aw88194_vbat_monitor_stop(aw88194);
    }
}

static void aw88194_memclk_select(struct aw88194 *aw88194, unsigned char flag)
{
    pr_debug("%s enter\n", __func__);

    if(flag == AW88194_MEMCLK_PLL) {
        aw88194_i2c_write_bits(aw88194, AW88194_REG_SYSCTRL2,
                AW88194_BIT_SYSCTRL2_MEMCLK_MASK,
                AW88194_BIT_SYSCTRL2_MEMCLK_PLL);
    } else if (flag == AW88194_MEMCLK_OSC){
        aw88194_i2c_write_bits(aw88194, AW88194_REG_SYSCTRL2,
                AW88194_BIT_SYSCTRL2_MEMCLK_MASK,
                AW88194_BIT_SYSCTRL2_MEMCLK_OSC);
    } else {
        pr_err("%s unkown memclk config, flag=0x%x\n", __func__, flag);
    }
}

static int aw88194_get_iis_status(struct aw88194 *aw88194)
{
    int ret = -1;
    unsigned int reg_val = 0;

    pr_debug("%s enter\n", __func__);

    aw88194_i2c_read(aw88194, AW88194_REG_SYSST, &reg_val);
    if(reg_val & AW88194_BIT_SYSST_PLLS) {
        ret = 0;
    }

    return ret;
}

static int aw88194_get_dsp_status(struct aw88194 *aw88194)
{
    int ret = -1;
    unsigned int reg_val = 0;

    pr_debug("%s enter\n", __func__);

    aw88194_i2c_read(aw88194, AW88194_REG_WDT, &reg_val);
    if(reg_val) {
        ret = 0;
    }

    return ret;
}


static int aw88194_get_dsp_config(struct aw88194 *aw88194)
{
    int ret = -1;
    unsigned int reg_val = 0;

    pr_debug("%s enter\n", __func__);

    aw88194_i2c_read(aw88194, AW88194_REG_SYSCTRL, &reg_val);
    if(reg_val & AW88194_BIT_SYSCTRL_DSP_BYPASS) {
        aw88194->dsp_cfg = AW88194_DSP_BYPASS;
    } else {
        aw88194->dsp_cfg = AW88194_DSP_WORK;
    }

    return ret;
}

static int aw88194_get_hmute(struct aw88194 *aw88194)
{
    int ret = -1;
    unsigned int reg_val = 0;

    pr_debug("%s enter\n", __func__);

    aw88194_i2c_read(aw88194, AW88194_REG_PWMCTRL, &reg_val);
    if(reg_val & AW88194_BIT_PWMCTRL_HMUTE_ENABLE) {
        ret = 1;
    } else {
        ret = 0;
    }

    return ret;
}

/******************************************************
 *
 * aw88194 dsp
 *
 ******************************************************/
static int aw88194_dsp_check(struct aw88194 *aw88194)
{
    int ret = -1;
    unsigned int reg_val = 0;
    unsigned int iis_check_max = 5;
    unsigned int i = 0;

    pr_debug("%s enter\n", __func__);

    aw88194_run_pwd(aw88194, false);
    aw88194_memclk_select(aw88194, AW88194_MEMCLK_PLL);
    for(i=0; i<iis_check_max; i++) {
        ret = aw88194_get_iis_status(aw88194);
        if(ret < 0){
            pr_err("%s: iis signal check error, reg=0x%x\n", __func__, reg_val);
            msleep(2);
        } else {
            if(aw88194->dsp_cfg == AW88194_DSP_WORK) {
                aw88194_dsp_enable(aw88194, false);
                aw88194_dsp_enable(aw88194, true);
                msleep(1);
                ret = aw88194_get_dsp_status(aw88194);
                if(ret < 0) {
                    pr_err("%s: dsp wdt status error=%d\n", __func__, ret);
                } else {
                    return 0;
                }
            } else if(aw88194->dsp_cfg == AW88194_DSP_BYPASS){
                return 0;
            } else {
                pr_err("%s: unkown dsp cfg=%d\n", __func__, aw88194->dsp_cfg);
                return -1;
            }
        }
    }
    return -1;
}

static void aw88194_dsp_container_update(struct aw88194 *aw88194,
        struct aw88194_container *aw88194_cont, int base)
{
    int i = 0;
    unsigned int reg_val = 0;

    pr_debug("%s enter\n", __func__);

    mutex_lock(&aw88194->lock);
    aw88194_i2c_write(aw88194, AW88194_REG_DSPMADD, base);
    for(i=0; i<aw88194_cont->len; i+=2) {
        reg_val = (aw88194_cont->data[i+1]<<8) + aw88194_cont->data[i+0];
        aw88194_i2c_write(aw88194, AW88194_REG_DSPMDAT, reg_val);
    }
    mutex_unlock(&aw88194->lock);

    pr_debug("%s exit\n", __func__);
}

static void aw88194_dsp_cfg_loaded(const struct firmware *cont, void *context)
{
    struct aw88194 *aw88194 = context;
    struct aw88194_container *aw88194_cfg;
    int ret = -1;

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw88194_cfg_name[aw88194->cfg_num]);
        release_firmware(cont);
        return;
    }

    pr_info("%s: loaded %s - size: %zu\n", __func__, aw88194_cfg_name[aw88194->cfg_num],
            cont ? cont->size : 0);

    aw88194_cfg = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
    if (!aw88194_cfg) {
        release_firmware(cont);
        pr_err("%s: error allocating memory\n", __func__);
        return;
    }
    aw88194->dsp_cfg_len= cont->size;
    aw88194_cfg->len = cont->size;
    memcpy(aw88194_cfg->data, cont->data, cont->size);
    release_firmware(cont);

    aw88194_dsp_container_update(aw88194, aw88194_cfg, aw88194_base_addr[aw88194->cfg_num]);

    kfree(aw88194_cfg);

    aw88194_dsp_update_cali_re(aw88194);

    ret = aw88194_dsp_check(aw88194);
    if(ret < 0) {
        aw88194->init = AW88194_INIT_NG;
        aw88194_run_mute(aw88194, true);
        pr_info("%s: fw/cfg update error\n", __func__);
    } else {
        pr_info("%s: fw/cfg update complete\n", __func__);

        aw88194_get_bst_ipeak(aw88194);

        aw88194->init = AW88194_INIT_OK;
        aw88194_start(aw88194);
    }
}

static int aw88194_load_dsp_cfg(struct aw88194 *aw88194)
{
    pr_info("%s enter\n", __func__);

    aw88194->cfg_num ++;

    return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
            aw88194_cfg_name[aw88194->cfg_num], aw88194->dev, GFP_KERNEL,
            aw88194, aw88194_dsp_cfg_loaded);
}


static void aw88194_dsp_fw_loaded(const struct firmware *cont, void *context)
{
    struct aw88194 *aw88194 = context;
    struct aw88194_container *aw88194_cfg;
    int ret = -1;

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw88194_cfg_name[aw88194->cfg_num]);
        release_firmware(cont);
        return;
    }

    pr_info("%s: loaded %s - size: %zu\n", __func__, aw88194_cfg_name[aw88194->cfg_num],
            cont ? cont->size : 0);

    aw88194_cfg = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
    if (!aw88194_cfg) {
        release_firmware(cont);
        pr_err("%s: error allocating memory\n", __func__);
        return;
    }
    aw88194->dsp_fw_len= cont->size;
    aw88194_cfg->len = cont->size;
    memcpy(aw88194_cfg->data, cont->data, cont->size);
    release_firmware(cont);

    aw88194_dsp_container_update(aw88194, aw88194_cfg, aw88194_base_addr[aw88194->cfg_num]);

    kfree(aw88194_cfg);

    ret = aw88194_load_dsp_cfg(aw88194);
    if(ret) {
        pr_err("%s: cfg loading requested failed: %d\n", __func__, ret);
    }
}

static int aw88194_load_dsp_fw(struct aw88194 *aw88194)
{
    pr_info("%s enter\n", __func__);

    aw88194->cfg_num ++;

    aw88194_run_mute(aw88194, true);
    aw88194_dsp_enable(aw88194, false);
    aw88194_memclk_select(aw88194, AW88194_MEMCLK_PLL);

    return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
            aw88194_cfg_name[aw88194->cfg_num], aw88194->dev, GFP_KERNEL,
            aw88194, aw88194_dsp_fw_loaded);
}

static void aw88194_reg_container_update(struct aw88194 *aw88194,
        struct aw88194_container *aw88194_cont)
{
    int i = 0;
    int reg_addr = 0;
    int reg_val = 0;

    pr_debug("%s enter\n", __func__);

    mutex_lock(&aw88194->lock);
    for(i=0; i<aw88194_cont->len; i+=4) {
        reg_addr = (aw88194_cont->data[i+1]<<8) + aw88194_cont->data[i+0];
        reg_val = (aw88194_cont->data[i+3]<<8) + aw88194_cont->data[i+2];
        pr_debug("%s: reg=0x%04x, val = 0x%04x\n",
                __func__, reg_addr, reg_val);
        aw88194_i2c_write(aw88194, (unsigned char)reg_addr,
                (unsigned int)reg_val);
    }
    mutex_unlock(&aw88194->lock);

    pr_debug("%s exit\n", __func__);
}

static void aw88194_reg_loaded(const struct firmware *cont, void *context)
{
    struct aw88194 *aw88194 = context;
    struct aw88194_container *aw88194_cfg;
    int ret = -1;

    if (!cont) {
        pr_err("%s: failed to read %s\n", __func__, aw88194_cfg_name[aw88194->cfg_num]);
        release_firmware(cont);
        return;
    }

    pr_info("%s: loaded %s - size: %zu\n", __func__, aw88194_cfg_name[aw88194->cfg_num],
            cont ? cont->size : 0);

    aw88194_cfg = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
    if (!aw88194_cfg) {
        release_firmware(cont);
        pr_err("%s: error allocating memory\n", __func__);
        return;
    }
    aw88194_cfg->len = cont->size;
    memcpy(aw88194_cfg->data, cont->data, cont->size);
    release_firmware(cont);

    aw88194_reg_container_update(aw88194, aw88194_cfg);

    kfree(aw88194_cfg);

    aw88194_get_dsp_config(aw88194);

    ret = aw88194_get_iis_status(aw88194);
    if(ret < 0) {
        pr_err("%s: get no iis singal, ret=%d\n", __func__, ret);
    } else {
        ret = aw88194_load_dsp_fw(aw88194);
        if(ret) {
            pr_err("%s: cfg loading requested failed: %d\n", __func__, ret);
        }
    }
}

static int aw88194_load_reg(struct aw88194 *aw88194)
{
    pr_info("%s enter\n", __func__);

    return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
            aw88194_cfg_name[aw88194->cfg_num], aw88194->dev, GFP_KERNEL,
            aw88194, aw88194_reg_loaded);
}

static void aw88194_get_cfg_shift(struct aw88194 *aw88194)
{
    aw88194->cfg_num = aw88194_mode_cfg_shift[aw88194->spk_rcv_mode];
    pr_debug("%s cfg_num=%d\n", __func__, aw88194->cfg_num);
}

static void aw88194_cold_start(struct aw88194 *aw88194)
{
    int ret = -1;

    pr_info("%s enter\n", __func__);

    aw88194_get_cfg_shift(aw88194);

    ret = aw88194_load_reg(aw88194);
    if(ret) {
        pr_err("%s: cfg loading requested failed: %d\n", __func__, ret);
    }
}

static void aw88194_smartpa_cfg(struct aw88194 *aw88194, bool flag)
{
    pr_info("%s, flag = %d\n", __func__, flag);

    if(flag == true) {
        if((aw88194->init == AW88194_INIT_ST) || (aw88194->init == AW88194_INIT_NG)) {
            pr_info("%s, init = %d\n", __func__, aw88194->init);
            aw88194_cold_start(aw88194);
        } else {
            aw88194_start(aw88194);
        }
    } else {
        aw88194_stop(aw88194);
    }
}

/******************************************************
 *
 * kcontrol
 *
 ******************************************************/
 static const char *const spk_function[] = { "Off", "On" };
 static const char *const rcv_function[] = { "Off", "On" };
 static const DECLARE_TLV_DB_SCALE(digital_gain,0,50,0);

 struct soc_mixer_control aw88194_mixer ={
    .reg    = AW88194_REG_HAGCCFG7,
    .shift  = AW88194_VOL_REG_SHIFT,
    .max    = AW88194_VOLUME_MAX,
    .min    = AW88194_VOLUME_MIN,
 };

static int aw88194_volume_info(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_info *uinfo)
{
    struct soc_mixer_control *mc = (struct soc_mixer_control*)kcontrol->private_value;

    //set kcontrol info
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = mc->max - mc->min;
    return 0;
}

static int aw88194_volume_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);
    unsigned int reg_val = 0;
    unsigned int value = 0;
    struct soc_mixer_control *mc = (struct soc_mixer_control*) kcontrol->private_value;

    aw88194_i2c_read(aw88194, AW88194_REG_HAGCCFG7, &reg_val);
    ucontrol->value.integer.value[0] = (value >> mc->shift)\
            &(AW88194_BIT_HAGCCFG7_VOL_MASK);
    return 0;
}

static int aw88194_volume_put(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct soc_mixer_control *mc = (struct soc_mixer_control*) kcontrol->private_value;
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);
    unsigned int value = 0;
    unsigned int reg_value = 0;

    //value is right
    value = ucontrol->value.integer.value[0];
    if(value > (mc->max-mc->min)|| value <0){
      pr_err("%s:value over range \n",__func__);
      return -1;
    }

    //smartpa have clk
    aw88194_i2c_read(aw88194, AW88194_REG_SYSST, &reg_value);
    if(!(reg_value&AW88194_BIT_SYSST_PLLS)){
      pr_err("%s: NO I2S CLK ,cat not write reg \n",__func__);
      return 0;
    }
    //cal real value
    value = value << mc->shift&AW88194_BIT_HAGCCFG7_VOL_MASK;
    aw88194_i2c_read(aw88194, AW88194_REG_HAGCCFG7, &reg_value);
    value = value | (reg_value&0x00ff);

    //write value
    aw88194_i2c_write(aw88194, AW88194_REG_HAGCCFG7, value);

    return 0;
}

static struct snd_kcontrol_new aw88194_volume = {
    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
    .name  = "aw88194_rx_volume",
    .access= SNDRV_CTL_ELEM_ACCESS_TLV_READ|SNDRV_CTL_ELEM_ACCESS_READWRITE,
    .tlv.p  = (digital_gain),
    .info = aw88194_volume_info,
    .get =  aw88194_volume_get,
    .put =  aw88194_volume_put,
    .private_value = (unsigned long)&aw88194_mixer,
};

static int aw88194_spk_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    pr_debug("%s: aw88194_spk_control=%d\n", __func__, aw88194_spk_control);
    ucontrol->value.integer.value[0] = aw88194_spk_control;
    return 0;
}

static int aw88194_spk_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);

    pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n",
            __func__, ucontrol->value.integer.value[0]);
    if(ucontrol->value.integer.value[0] == aw88194_spk_control)
        return 1;

    aw88194_spk_control = ucontrol->value.integer.value[0];

    if(AW88194_SPEAKER_MODE == aw88194->spk_rcv_mode) {
    } else {
        aw88194->spk_rcv_mode = AW88194_SPEAKER_MODE;
        aw88194->init = AW88194_INIT_ST;
    }



    return 0;
}

static int aw88194_rcv_get(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    pr_debug("%s: aw88194_rcv_control=%d\n", __func__, aw88194_rcv_control);
    ucontrol->value.integer.value[0] = aw88194_rcv_control;
    return 0;
}

static int aw88194_rcv_set(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);
    pr_debug("%s: ucontrol->value.integer.value[0]=%ld\n ",
            __func__, ucontrol->value.integer.value[0]);
    if(ucontrol->value.integer.value[0] == aw88194_rcv_control)
        return 1;

    aw88194_rcv_control = ucontrol->value.integer.value[0];

    if(AW88194_RECEIVER_MODE == aw88194->spk_rcv_mode) {
    } else {
        aw88194->spk_rcv_mode = AW88194_RECEIVER_MODE;
        aw88194->init = AW88194_INIT_ST;
    }

    return 0;
}
static const struct soc_enum aw88194_snd_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_function), spk_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rcv_function), rcv_function),
};

static struct snd_kcontrol_new aw88194_controls[] = {
    SOC_ENUM_EXT("aw88194_speaker_switch", aw88194_snd_enum[0],
            aw88194_spk_get, aw88194_spk_set),
    SOC_ENUM_EXT("aw88194_receiver_switch", aw88194_snd_enum[1],
            aw88194_rcv_get, aw88194_rcv_set),
};

static void aw88194_add_codec_controls(struct aw88194 *aw88194)
{
    pr_info("%s enter\n", __func__);

    snd_soc_add_codec_controls(aw88194->codec, aw88194_controls,
            ARRAY_SIZE(aw88194_controls));

    snd_soc_add_codec_controls(aw88194->codec, &aw88194_volume,1);
}

/******************************************************
 *
 * Digital Audio Interface
 *
 ******************************************************/
static int aw88194_startup(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
    struct snd_soc_codec *codec = dai->codec;
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);

    pr_info("%s: enter\n", __func__);
    aw88194_run_pwd(aw88194, false);

    return 0;
}

static int aw88194_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
    //struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(dai->codec);
    struct snd_soc_codec *codec = dai->codec;

    pr_info("%s: fmt=0x%x\n", __func__, fmt);

    /* Supported mode: regular I2S, slave, or PDM */
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_I2S:
        if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
            dev_err(codec->dev, "%s: invalid codec master mode\n",
                    __func__);
            return -EINVAL;
        }
        break;
    default:
        dev_err(codec->dev, "%s: unsupported DAI format %d\n",
                __func__, fmt & SND_SOC_DAIFMT_FORMAT_MASK);
        return -EINVAL;
    }
    return 0;
}

static int aw88194_set_dai_sysclk(struct snd_soc_dai *codec_dai,
        int clk_id, unsigned int freq, int dir)
{
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec_dai->codec);

    pr_info("%s: freq=%d\n", __func__, freq);

    aw88194->sysclk = freq;
    return 0;
}

static int aw88194_hw_params(struct snd_pcm_substream *substream,
    struct snd_pcm_hw_params *params,
    struct snd_soc_dai *dai)
{
    struct snd_soc_codec *codec = dai->codec;
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);
    unsigned int rate = 0;
    int reg_value = 0;
    int width = 0;
    /* Supported */

    //get rate param
    aw88194->rate=rate = params_rate(params);
    pr_debug("%s: requested rate: %d, sample size: %d\n", __func__, rate,
            snd_pcm_format_width(params_format(params)));
    //match rate
    switch(rate)
    {
        case 8000:
            reg_value = AW88194_BIT_I2SCTRL_SR_8K;
            break;
        case 16000:
            reg_value = AW88194_BIT_I2SCTRL_SR_16K;
            break;
        case 32000:
            reg_value = AW88194_BIT_I2SCTRL_SR_32K;
            break;
        case 44100:
            reg_value = AW88194_BIT_I2SCTRL_SR_44P1K;
            break;
        case 48000:
            reg_value = AW88194_BIT_I2SCTRL_SR_48K;
            break;
        case 96000:
            reg_value = AW88194_BIT_I2SCTRL_SR_96K;
            break;
        case 192000:
            reg_value = AW88194_BIT_I2SCTRL_SR_192K;
            break;
        default:
            reg_value = AW88194_BIT_I2SCTRL_SR_48K;
            pr_err("%s: rate can not support\n", __func__);
            break;
    }
    //set chip rate
    if(-1 != reg_value){
        aw88194_i2c_write_bits(aw88194, AW88194_REG_I2SCTRL,
                AW88194_BIT_I2SCTRL_SR_MASK, reg_value);
    }

    //get bit width
    width = params_width(params);
    pr_debug("%s: width = %d \n",__func__,width);
    switch(width)
    {
        case 16:
            reg_value = AW88194_BIT_I2SCTRL_FMS_16BIT;
            break;
        case 20:
            reg_value = AW88194_BIT_I2SCTRL_FMS_20BIT;
            break;
        case 24:
            reg_value = AW88194_BIT_I2SCTRL_FMS_24BIT;
            break;
        case 32:
            reg_value = AW88194_BIT_I2SCTRL_FMS_32BIT;
            break;
        default:
            reg_value = AW88194_BIT_I2SCTRL_FMS_16BIT;
            pr_err("%s: width can not support\n", __func__);
            break;
    }
    //set width
    if(-1 != reg_value){
        aw88194_i2c_write_bits(aw88194, AW88194_REG_I2SCTRL,
                AW88194_BIT_I2SCTRL_FMS_MASK, reg_value);
    }

    return 0;
}

static int aw88194_mute(struct snd_soc_dai *dai, int mute, int stream)
{
    struct snd_soc_codec *codec = dai->codec;
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);

    pr_info("%s: mute state=%d\n", __func__, mute);

    if (!(aw88194->flags & AW88194_FLAG_START_ON_MUTE))
        return 0;

    if (mute) {
        if (stream == SNDRV_PCM_STREAM_PLAYBACK)
            aw88194->pstream = 0;
        else
            aw88194->cstream = 0;
        if (aw88194->pstream != 0 || aw88194->cstream != 0)
            return 0;

        aw88194_smartpa_cfg(aw88194, false);
    } else {
        if (stream == SNDRV_PCM_STREAM_PLAYBACK)
            aw88194->pstream = 1;
        else
            aw88194->cstream = 1;

        aw88194_smartpa_cfg(aw88194, true);
    }

    return 0;
}

static void aw88194_shutdown(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
    struct snd_soc_codec *codec = dai->codec;
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);

    aw88194->rate = 0;
    aw88194_run_pwd(aw88194, true);
}

static const struct snd_soc_dai_ops aw88194_dai_ops = {
    .startup = aw88194_startup,
    .set_fmt = aw88194_set_fmt,
    .set_sysclk = aw88194_set_dai_sysclk,
    .hw_params = aw88194_hw_params,
    .mute_stream = aw88194_mute,
    .shutdown = aw88194_shutdown,
};

static struct snd_soc_dai_driver aw88194_dai[] = {
    {
        .name = "aw88194-aif",
        .id = 1,
        .playback = {
            .stream_name = "Speaker_Playback",
            .channels_min = 1,
            .channels_max = 2,
            .rates = AW88194_RATES,
            .formats = AW88194_FORMATS,
        },
        .capture = {
            .stream_name = "Speaker_Capture",
            .channels_min = 1,
            .channels_max = 2,
            .rates = AW88194_RATES,
            .formats = AW88194_FORMATS,
         },
        .ops = &aw88194_dai_ops,
        .symmetric_rates = 1,
        .symmetric_channels = 1,
        .symmetric_samplebits = 1,
    },
};

/*****************************************************
 *
 * codec driver
 *
 *****************************************************/
static int aw88194_probe(struct snd_soc_codec *codec)
{
    struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);
    int ret = 0;

    pr_info("%s enter\n", __func__);

    aw88194->codec = codec;

    aw88194_add_codec_controls(aw88194);

    if (codec->dev->of_node)
        dev_set_name(codec->dev, "%s", "aw88194_smartpa");

    pr_info("%s exit\n", __func__);

    return ret;
}

static int aw88194_remove(struct snd_soc_codec *codec)
{
    //struct aw88194 *aw88194 = snd_soc_codec_get_drvdata(codec);
    pr_info("%s enter\n", __func__);

    return 0;
}


static unsigned int aw88194_codec_read(struct snd_soc_codec *codec,
        unsigned int reg)
{
    struct aw88194 *aw88194=snd_soc_codec_get_drvdata(codec);
    unsigned int value =0;
    int ret;
    pr_debug("%s:enter \n", __func__);

    if(aw88194_reg_access[reg]&REG_RD_ACCESS){
        ret=aw88194_i2c_read(aw88194,reg,&value);
    if(ret<0){
        pr_debug("%s: read register failed \n", __func__);
        return ret;
    }
    }else{
        pr_debug("%s:Register 0x%x NO read access\n", __func__, reg);
        return -1;
    }
    return value;
}

static int aw88194_codec_write(struct snd_soc_codec *codec,
        unsigned int reg,unsigned int value)

{
    int ret ;
    struct aw88194 *aw88194=snd_soc_codec_get_drvdata(codec);
    pr_debug("%s:enter ,reg is 0x%x value is 0x%x\n", __func__, reg, value);

    if(aw88194_reg_access[reg]&REG_WR_ACCESS){
        ret=aw88194_i2c_write(aw88194,reg,value);
        return ret;
    }else{
        pr_debug("%s: Register 0x%x NO write access \n", __func__, reg);
    }

    return -1;
}

static struct snd_soc_codec_driver soc_codec_dev_aw88194 = {
    .probe = aw88194_probe,
    .remove = aw88194_remove,
    .read = aw88194_codec_read,
    .write= aw88194_codec_write,
    .reg_cache_size= AW88194_REG_MAX,
    .reg_word_size=2,
};

/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw88194_interrupt_setup(struct aw88194 *aw88194)
{
    unsigned int reg_val;

    pr_info("%s enter\n", __func__);

    aw88194_i2c_read(aw88194, AW88194_REG_SYSINTM, &reg_val);
    reg_val &= (~AW88194_BIT_SYSINTM_PLLM);
    reg_val &= (~AW88194_BIT_SYSINTM_OTHM);
    reg_val &= (~AW88194_BIT_SYSINTM_OCDM);
    aw88194_i2c_write(aw88194, AW88194_REG_SYSINTM, reg_val);
}

static void aw88194_interrupt_clear(struct aw88194 *aw88194)
{
    unsigned int reg_val = 0;

    pr_info("%s enter\n", __func__);

    aw88194_i2c_read(aw88194, AW88194_REG_SYSST, &reg_val);
    pr_info("%s: reg SYSST=0x%x\n", __func__, reg_val);

    aw88194_i2c_read(aw88194, AW88194_REG_SYSINT, &reg_val);
    pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

    aw88194_i2c_read(aw88194, AW88194_REG_SYSINTM, &reg_val);
    pr_info("%s: reg SYSINTM=0x%x\n", __func__, reg_val);
}

static irqreturn_t aw88194_irq(int irq, void *data)
{
    struct aw88194 *aw88194 = data;

    pr_info("%s enter\n", __func__);

    aw88194_interrupt_clear(aw88194);

    pr_info("%s exit\n", __func__);

    return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw88194_parse_dt(struct device *dev, struct aw88194 *aw88194,
        struct device_node *np)
{
    int ret = -1;

    /* gpio */
    aw88194->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (aw88194->reset_gpio < 0) {
        dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n",
            __func__);
        return -1;
    } else {
        dev_info(dev, "%s: reset gpio provided ok\n", __func__);
    }
    aw88194->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
    if (aw88194->irq_gpio < 0) {
        dev_info(dev, "%s: no irq gpio provided.\n", __func__);
    } else {
        dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
    }

    /* vbat monitor */
    ret = of_property_read_u32(np, "vbat-monitor-flag",
        &aw88194->vbat_monitor_flag);
    if(ret) {
        dev_err(dev, "%s: find no vbat-monitor-flag, use default config\n",
            __func__);
        aw88194->vbat_monitor_flag = AW88194_VBAT_MONITOR_DFT_FLAG;
    } else {
        dev_info(dev, "%s: vbat-monitor-flag = %d\n",
            __func__, aw88194->vbat_monitor_flag);
    }
    ret = of_property_read_u32(np, "vbat-monitor-limit-min",
        &aw88194->vbat_monitor_limit_min);
    if(ret) {
        dev_err(dev, "%s: find no vbat-monitor-limit-min, use default config\n",
            __func__);
        aw88194->vbat_monitor_limit_min = AW88194_VBAT_MONITOR_DFT_LIMIT_MIN;
    } else {
        dev_info(dev, "%s: vbat-monitor-limit-min = %d\n",
            __func__, aw88194->vbat_monitor_limit_min);
    }
    ret = of_property_read_u32(np, "vbat-monitor-bst-ipeak-limit",
        &aw88194->bst_ipeak_limit);
    if(ret) {
        dev_err(dev, "%s: find no bst-ipeak-limit, use default config\n",
            __func__);
        aw88194->bst_ipeak_limit = AW88194_BST_IPEAK_DFT_LIMIT;
    } else {
        dev_info(dev, "%s: bst-ipeak-limit = %d\n",
            __func__, aw88194->bst_ipeak_limit);
    }

    return 0;
}

int aw88194_hw_reset(struct aw88194 *aw88194)
{
    pr_info("%s enter\n", __func__);

    if (aw88194 && gpio_is_valid(aw88194->reset_gpio)) {
        gpio_set_value_cansleep(aw88194->reset_gpio, 0);
        msleep(1);
        gpio_set_value_cansleep(aw88194->reset_gpio, 1);
        msleep(1);
    } else {
        dev_err(aw88194->dev, "%s:  failed\n", __func__);
    }
    return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
int aw88194_read_chipid(struct aw88194 *aw88194)
{
    int ret = -1;
    unsigned int cnt = 0;
    unsigned int reg = 0;

    while(cnt < AW_READ_CHIPID_RETRIES) {
        ret = aw88194_i2c_read(aw88194, AW88194_REG_ID, &reg);
        if (ret < 0) {
            dev_err(aw88194->dev, "%s: failed to read REG_ID: %d\n",
                    __func__, ret);
            return -EIO;
        }
        switch (reg) {
        case AW88194_ID:
            pr_info("%s aw88194 detected\n", __func__);
            aw88194->flags |= AW88194_FLAG_SKIP_INTERRUPTS;
            aw88194->flags |= AW88194_FLAG_START_ON_MUTE;
            aw88194->chipid = AW88194_ID;
            pr_info("%s aw88194->flags=0x%x\n", __func__, aw88194->flags);
            return 0;
        default:
            pr_info("%s unsupported device revision (0x%x)\n",
                    __func__, reg );
            break;
        }
        cnt ++;

        msleep(AW_READ_CHIPID_RETRY_DELAY);
    }

    return -EINVAL;
}

/*****************************************************
 *
 * vbat monitor
 *
 *****************************************************/
static int aw88194_vbat_monitor_stop(struct aw88194 *aw88194)
{
    pr_info("%s enter\n", __func__);

    if(hrtimer_active(&aw88194->vbat_monitor_timer)) {
        pr_info("%s: cancel vbat monitor\n", __func__);
        hrtimer_cancel(&aw88194->vbat_monitor_timer);
    }
    return 0;
}

static int aw88194_vbat_monitor_start(struct aw88194 *aw88194)
{
    int ram_timer_val = 30000;

    pr_info("%s enter\n", __func__);

    if(hrtimer_active(&aw88194->vbat_monitor_timer)) {
    } else {
        pr_info("%s: start vbat monitor\n", __func__);
        hrtimer_start(&aw88194->vbat_monitor_timer,
                ktime_set(ram_timer_val/1000, (ram_timer_val%1000)*1000000),
                HRTIMER_MODE_REL);
    }
    return 0;
}

static enum hrtimer_restart aw88194_vbat_monitor_timer_func(struct hrtimer *timer)
{
    struct aw88194 *aw88194 = container_of(timer, struct aw88194, vbat_monitor_timer);

    pr_debug("%s enter\n", __func__);

    schedule_work(&aw88194->vbat_monitor_work);

    return HRTIMER_NORESTART;
}

static unsigned int aw88194_get_vbat_vol(struct aw88194 *aw88194)
{
    unsigned int reg_val = 0;
    unsigned int ret = 0;

    aw88194_i2c_read(aw88194, AW88194_REG_VBAT, &reg_val);
    ret = reg_val * AW88194_VBAT_RANGE / 1023;

    return ret;
}

static int aw88194_set_bst_ipeak(struct aw88194 *aw88194, unsigned int bst_ipeak)
{
    int ret = 0;

    aw88194_i2c_write_bits(aw88194, AW88194_REG_SYSCTRL2,
        AW88194_BIT_SYSCTRL2_BST_IPEAK_MASK, bst_ipeak);

    return ret;
}

static int aw88194_get_bst_ipeak(struct aw88194 *aw88194)
{
    unsigned int reg_val = 0;
    int ret = 0;

    aw88194_i2c_read(aw88194, AW88194_REG_SYSCTRL2, &reg_val);
    aw88194->bst_ipeak = reg_val&(~AW88194_BIT_SYSCTRL2_BST_IPEAK_MASK);

    return ret;
}

static void aw88194_vbat_monitor_work_routine(struct work_struct *work)
{
    struct aw88194 *aw88194 = container_of(work, struct aw88194, vbat_monitor_work);
    unsigned int vbat_vol= 0;

    pr_debug("%s enter\n", __func__);

    mutex_lock(&aw88194->lock);
    if(aw88194_get_hmute(aw88194)) {
    } else {
        vbat_vol = aw88194_get_vbat_vol(aw88194);
        pr_info("%s: get battery = %dmV\n", __func__, vbat_vol);
        if(vbat_vol < aw88194->vbat_monitor_limit_min) {
            aw88194_set_bst_ipeak(aw88194, aw88194->bst_ipeak_limit);
        } else {
            aw88194_set_bst_ipeak(aw88194, aw88194->bst_ipeak);
        }
        aw88194_vbat_monitor_start(aw88194);
    }
    mutex_unlock(&aw88194->lock);
}

static int aw88194_vbat_monitor_init(struct aw88194 *aw88194)
{
    pr_info("%s enter\n", __func__);

    aw88194_get_bst_ipeak(aw88194);

    hrtimer_init(&aw88194->vbat_monitor_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aw88194->vbat_monitor_timer.function = aw88194_vbat_monitor_timer_func;
    INIT_WORK(&aw88194->vbat_monitor_work, aw88194_vbat_monitor_work_routine);
    return 0;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw88194_reg_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw88194_i2c_write(aw88194, databuf[0], databuf[1]);
    }

    return count;
}

static ssize_t aw88194_reg_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned int reg_val = 0;
    for(i = 0; i < AW88194_REG_MAX; i ++) {
        if(aw88194_reg_access[i]&REG_RD_ACCESS) {
            aw88194_i2c_read(aw88194, i, &reg_val);
            len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%04x \n", i, reg_val);
        }
    }
    return len;
}

static ssize_t aw88194_rw_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw88194->reg_addr = (unsigned char)databuf[0];
        aw88194_i2c_write(aw88194, databuf[0], databuf[1]);
        pr_debug("%s: get param: %x %x\n", __func__, databuf[0], databuf[1]);
    }else if(1 == sscanf(buf, "%x", &databuf[0])) {
        aw88194->reg_addr = (unsigned char)databuf[0];
        pr_debug("%s: get param: %x\n", __func__, databuf[0]);
    }

    return count;
}

static ssize_t aw88194_dsp_rw_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned int reg_val = 0;

    aw88194_i2c_read(aw88194, AW88194_REG_DSPMDAT, &reg_val);
    len += snprintf(buf+len, PAGE_SIZE-len,
            "dsp:0x%04x=0x%04x\n", aw88194->dsp_addr, reg_val);
    return len;
}

static ssize_t aw88194_dsp_rw_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw88194->dsp_addr = (unsigned int)databuf[0];
        aw88194_i2c_write(aw88194, AW88194_REG_DSPMADD, databuf[0]);
        aw88194_i2c_write(aw88194, AW88194_REG_DSPMDAT, databuf[1]);
        pr_debug("%s: get param: %x %x\n", __func__, databuf[0], databuf[1]);
    }else if(1 == sscanf(buf, "%x", &databuf[0])) {
        aw88194->dsp_addr = (unsigned int)databuf[0];
        aw88194_i2c_write(aw88194, AW88194_REG_DSPMADD, databuf[0]);
        pr_debug("%s: get param: %x\n", __func__, databuf[0]);
    }

    return count;
}

static ssize_t aw88194_rw_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned int reg_val = 0;

    if(aw88194_reg_access[aw88194->reg_addr] & REG_RD_ACCESS) {
        aw88194_i2c_read(aw88194, aw88194->reg_addr, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len,
                "reg:0x%02x=0x%04x\n", aw88194->reg_addr, reg_val);
    }
    return len;
}

static ssize_t aw88194_spk_rcv_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0};

    if(1 == sscanf(buf, "%d", &databuf[0])) {
        aw88194->spk_rcv_mode = databuf[0];
    }

    return count;
}

static ssize_t aw88194_spk_rcv_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);
    ssize_t len = 0;
    if(aw88194->spk_rcv_mode == AW88194_SPEAKER_MODE) {
        len += snprintf(buf+len, PAGE_SIZE-len, "aw88194 spk_rcv: %d, speaker mode\n", aw88194->spk_rcv_mode);
    } else if (aw88194->spk_rcv_mode == AW88194_RECEIVER_MODE) {
        len += snprintf(buf+len, PAGE_SIZE-len, "aw88194 spk_rcv: %d, receiver mode\n", aw88194->spk_rcv_mode);
    } else {
        len += snprintf(buf+len, PAGE_SIZE-len, "aw88194 spk_rcv: %d, unknown mode\n", aw88194->spk_rcv_mode);
    }

    return len;
}


static ssize_t aw88194_dsp_store(struct device *dev, struct device_attribute *attr,
                  const char *buf, size_t count)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);
    unsigned int val = 0;
    int ret = 0;

    ret = kstrtouint(buf, 0, &val);
    if (ret < 0)
        return ret;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    if(val) {
        aw88194->init = AW88194_INIT_ST;
        aw88194_smartpa_cfg(aw88194, false);
        aw88194_smartpa_cfg(aw88194, true);
    }
    return count;
}

static ssize_t aw88194_dsp_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned int i = 0;
    unsigned int reg_val = 0;
    int ret = -1;

    aw88194_get_dsp_config(aw88194);
    if(aw88194->dsp_cfg == AW88194_DSP_BYPASS) {
        len += snprintf((char *)(buf+len), PAGE_SIZE-len,
                "%s: aw88194 dsp bypass\n", __func__);
    } else {
        len += snprintf(buf+len, PAGE_SIZE-len, "aw88194 dsp working\n");
        ret = aw88194_get_iis_status(aw88194);
        if(ret < 0) {
            len += snprintf((char *)(buf+len), PAGE_SIZE-len,
                    "%s: aw88194 no iis signal\n", __func__);
        } else {
            pr_info("%s: aw88194_dsp_firmware:\n", __func__);
            aw88194_i2c_write(aw88194, AW88194_REG_DSPMADD, AW88194_DSP_FW_ADDR);
            for(i=0; i<aw88194->dsp_fw_len; i+=2) {
                aw88194_i2c_read(aw88194, AW88194_REG_DSPMDAT, &reg_val);
                pr_info("%s: dsp_fw[0x%04x]:0x%02x,0x%02x\n",
                        __func__, i, (reg_val>>0)&0xff, (reg_val>>8)&0xff);
            }
            pr_info("\n");

            pr_info("%s: aw88194_dsp_cfg:\n", __func__);
            aw88194_i2c_write(aw88194, AW88194_REG_DSPMADD, AW88194_DSP_FW_ADDR);
            len += snprintf(buf+len, PAGE_SIZE-len, "aw88194 dsp config:\n");
            aw88194_i2c_write(aw88194, AW88194_REG_DSPMADD, AW88194_DSP_CFG_ADDR);
            for(i=0; i<aw88194->dsp_cfg_len; i+=2) {
                aw88194_i2c_read(aw88194, AW88194_REG_DSPMDAT, &reg_val);
                len += snprintf(buf+len, PAGE_SIZE-len,
                        "0x%02x,0x%02x,", (reg_val>>0)&0xff, (reg_val>>8)&0xff);
                pr_info("%s: dsp_cfg[0x%04x]:0x%02x,0x%02x\n",
                        __func__, i, (reg_val>>0)&0xff, (reg_val>>8)&0xff);
                if((i/2+1)%8 == 0) {
                    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
                }
            }
            len += snprintf(buf+len, PAGE_SIZE-len, "\n");
        }
    }

    return len;
}

static ssize_t aw88194_cali_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0};

    if(1 == sscanf(buf, "%x", &databuf[0])) {
        aw88194->cali_re = databuf[0];
        aw88194_set_cali_re(aw88194);
    }

    return count;
}

static ssize_t aw88194_cali_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);
    ssize_t len = 0;

    len += snprintf(buf+len, PAGE_SIZE-len, "aw88194 cali re: 0x%04x\n", aw88194->cali_re);

    return len;
}


static ssize_t aw88194_bst_ipeak_limit_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0};

    if(1 == sscanf(buf, "%d", &databuf[0])) {
        aw88194->bst_ipeak_limit = databuf[0];
    }

    return count;
}

static ssize_t aw88194_bst_ipeak_limit_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    struct aw88194 *aw88194 = dev_get_drvdata(dev);
    ssize_t len = 0;

    len += snprintf(buf+len, PAGE_SIZE-len, "aw88194 bst_ipeak_limit=0x%02x\n", aw88194->bst_ipeak_limit);

    return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw88194_reg_show, aw88194_reg_store);
static DEVICE_ATTR(rw, S_IWUSR | S_IRUGO, aw88194_rw_show, aw88194_rw_store);
static DEVICE_ATTR(dsp_rw, S_IWUSR | S_IRUGO, aw88194_dsp_rw_show, aw88194_dsp_rw_store);
static DEVICE_ATTR(spk_rcv, S_IWUSR | S_IRUGO, aw88194_spk_rcv_show, aw88194_spk_rcv_store);
static DEVICE_ATTR(dsp, S_IWUSR | S_IRUGO, aw88194_dsp_show, aw88194_dsp_store);
static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO, aw88194_cali_show, aw88194_cali_store);
static DEVICE_ATTR(bst_ipeak_limit, S_IWUSR | S_IRUGO, aw88194_bst_ipeak_limit_show, aw88194_bst_ipeak_limit_store);

static struct attribute *aw88194_attributes[] = {
    &dev_attr_reg.attr,
    &dev_attr_rw.attr,
    &dev_attr_dsp_rw.attr,
    &dev_attr_spk_rcv.attr,
    &dev_attr_dsp.attr,
    &dev_attr_cali.attr,
    &dev_attr_bst_ipeak_limit.attr,
    NULL
};

static struct attribute_group aw88194_attribute_group = {
    .attrs = aw88194_attributes
};


/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw88194_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    struct snd_soc_dai_driver *dai;
    struct aw88194 *aw88194;
    struct device_node *np = i2c->dev.of_node;
    int irq_flags = 0;
    int ret = -1;

    pr_info("%s enter\n", __func__);

    if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
        dev_err(&i2c->dev, "check_functionality failed\n");
        return -EIO;
    }

    aw88194 = devm_kzalloc(&i2c->dev, sizeof(struct aw88194), GFP_KERNEL);
    if (aw88194 == NULL)
        return -ENOMEM;

    aw88194->dev = &i2c->dev;
    aw88194->i2c = i2c;

    i2c_set_clientdata(i2c, aw88194);
    mutex_init(&aw88194->lock);

    /* aw88194 rst & int */
    if (np) {
        ret = aw88194_parse_dt(&i2c->dev, aw88194, np);
        if (ret) {
              dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
              goto err_parse_dt;
        }
    } else {
        aw88194->reset_gpio = -1;
        aw88194->irq_gpio = -1;
    }

    if (gpio_is_valid(aw88194->reset_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw88194->reset_gpio,
              GPIOF_OUT_INIT_LOW, "aw88194_rst");
        if (ret){
              dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
              goto err_reset_gpio_request;
        }
    }

    if (gpio_is_valid(aw88194->irq_gpio)) {
        ret = devm_gpio_request_one(&i2c->dev, aw88194->irq_gpio,
              GPIOF_DIR_IN, "aw88194_int");
        if (ret){
              dev_err(&i2c->dev, "%s: int request failed\n", __func__);
              goto err_irq_gpio_request;
        }
    }

    /* hardware reset */
    aw88194_hw_reset(aw88194);

    /* aw88194 chip id */
    ret = aw88194_read_chipid(aw88194);
    if (ret < 0) {
        dev_err(&i2c->dev, "%s: aw88194_read_chipid failed ret=%d\n", __func__, ret);
        goto err_id;
    }

    /* aw88194 device name */
    if (i2c->dev.of_node) {
        dev_set_name(&i2c->dev, "%s", "aw88194_smartpa");
    } else {
        dev_err(&i2c->dev, "%s failed to set device name: %d\n", __func__, ret);
    }

    /* register codec */
    dai = devm_kzalloc(&i2c->dev, sizeof(aw88194_dai), GFP_KERNEL);
    if (!dai) {
        goto err_dai_kzalloc;
    }
    memcpy(dai, aw88194_dai, sizeof(aw88194_dai));
    pr_info("%s dai->name(%s)\n", __func__, dai->name);

    ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_aw88194,
            dai, ARRAY_SIZE(aw88194_dai));
    if (ret < 0) {
        dev_err(&i2c->dev, "%s failed to register aw88194: %d\n", __func__, ret);
        goto err_register_codec;
    }

    /* aw88194 irq */
    if (gpio_is_valid(aw88194->irq_gpio) &&
        !(aw88194->flags & AW88194_FLAG_SKIP_INTERRUPTS)) {
        aw88194_interrupt_setup(aw88194);
        /* register irq handler */
        irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
        ret = devm_request_threaded_irq(&i2c->dev,
                gpio_to_irq(aw88194->irq_gpio),
                NULL, aw88194_irq, irq_flags,
                "aw88194", aw88194);
        if (ret != 0) {
              dev_err(&i2c->dev, "failed to request IRQ %d: %d\n",
                          gpio_to_irq(aw88194->irq_gpio), ret);
              goto err_irq;
        }
    } else {
        dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
        /* disable feature support if gpio was invalid */
        aw88194->flags |= AW88194_FLAG_SKIP_INTERRUPTS;
    }

    dev_set_drvdata(&i2c->dev, aw88194);
    ret = sysfs_create_group(&i2c->dev.kobj, &aw88194_attribute_group);
    if (ret < 0) {
        dev_info(&i2c->dev, "%s error creating sysfs attr files\n", __func__);
        goto err_sysfs;
    }

    aw88194_vbat_monitor_init(aw88194);

    pr_info("%s probe completed successfully!\n", __func__);

    return 0;

err_sysfs:
    devm_free_irq(&i2c->dev, gpio_to_irq(aw88194->irq_gpio), aw88194);
err_irq:
    snd_soc_unregister_codec(&i2c->dev);
err_register_codec:
    devm_kfree(&i2c->dev, dai);
    dai = NULL;
err_dai_kzalloc:
err_id:
    if (gpio_is_valid(aw88194->irq_gpio))
        devm_gpio_free(&i2c->dev, aw88194->irq_gpio);
err_irq_gpio_request:
    if (gpio_is_valid(aw88194->reset_gpio))
        devm_gpio_free(&i2c->dev, aw88194->reset_gpio);
err_reset_gpio_request:
err_parse_dt:
    devm_kfree(&i2c->dev, aw88194);
    aw88194 = NULL;
    return ret;
}

static int aw88194_i2c_remove(struct i2c_client *i2c)
{
    struct aw88194 *aw88194 = i2c_get_clientdata(i2c);

    pr_info("%s enter\n", __func__);

    if(gpio_to_irq(aw88194->irq_gpio))
        devm_free_irq(&i2c->dev, gpio_to_irq(aw88194->irq_gpio), aw88194);

    snd_soc_unregister_codec(&i2c->dev);

    if (gpio_is_valid(aw88194->irq_gpio))
        devm_gpio_free(&i2c->dev, aw88194->irq_gpio);
    if (gpio_is_valid(aw88194->reset_gpio))
        devm_gpio_free(&i2c->dev, aw88194->reset_gpio);

    devm_kfree(&i2c->dev, aw88194);
    aw88194 = NULL;

    return 0;
}

static const struct i2c_device_id aw88194_i2c_id[] = {
    { AW88194_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, aw88194_i2c_id);

static struct of_device_id aw88194_dt_match[] = {
    { .compatible = "awinic,aw88194_smartpa" },
    { },
};

static struct i2c_driver aw88194_i2c_driver = {
    .driver = {
        .name = AW88194_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(aw88194_dt_match),
    },
    .probe = aw88194_i2c_probe,
    .remove = aw88194_i2c_remove,
    .id_table = aw88194_i2c_id,
};


static int __init aw88194_i2c_init(void)
{
    int ret = 0;

    pr_info("aw88194 driver version %s\n", AW88194_VERSION);

    ret = i2c_add_driver(&aw88194_i2c_driver);
    if(ret){
        pr_err("fail to add aw88194 device into i2c\n");
        return ret;
    }

    return 0;
}
module_init(aw88194_i2c_init);


static void __exit aw88194_i2c_exit(void)
{
    i2c_del_driver(&aw88194_i2c_driver);
}
module_exit(aw88194_i2c_exit);


MODULE_DESCRIPTION("ASoC AW88194 Smart PA Driver");
MODULE_LICENSE("GPL v2");
