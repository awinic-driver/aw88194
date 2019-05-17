#ifndef _AW88194_H_
#define _AW88194_H_


/*
 * i2c transaction on Linux limited to 64k
 * (See Linux kernel documentation: Documentation/i2c/writing-clients)
*/
#define MAX_I2C_BUFFER_SIZE 65536

#define AW88194_FLAG_START_ON_MUTE          (1 << 0)
#define AW88194_FLAG_SKIP_INTERRUPTS        (1 << 1)
#define AW88194_FLAG_SAAM_AVAILABLE         (1 << 2)
#define AW88194_FLAG_STEREO_DEVICE          (1 << 3)
#define AW88194_FLAG_MULTI_MIC_INPUTS       (1 << 4)

#define AW88194_NUM_RATES                   9

#define AW88194_DFT_CALI_RE                 0x8000

#define AW88194_VBAT_MONITOR_DFT_FLAG       0
#define AW88194_VBAT_MONITOR_DFT_LIMIT_MIN  3400
#define AW88194_BST_IPEAK_DFT_LIMIT         0
#define AW88194_VBAT_RANGE                  6025

enum aw88194_baseaddr{
    AW88194_REG_ADDR = 0x00,
    AW88194_DSP_FW_ADDR = 0x8c00,
    AW88194_DSP_CFG_ADDR = 0x8600,
    AW88194_BASE_ADDR_MAX = 3,
};

#define AW88194_MODE_SHIFT_MAX              2
enum aw88194_modeshift {
    AW88194_MODE_SPK_SHIFT = 0,
    AW88194_MODE_RCV_SHIFT = 3,
};

enum aw88194_memclk{
    AW88194_MEMCLK_OSC = 0,
    AW88194_MEMCLK_PLL = 1,
};


enum aw88194_init{
    AW88194_INIT_ST = 0,
    AW88194_INIT_OK = 1,
    AW88194_INIT_NG = 2,
};

enum aw88194_chipid{
    AW88194_ID = 0x1806,
};

enum aw88194_dsp_cfg{
    AW88194_DSP_WORK = 0,
    AW88194_DSP_BYPASS = 1,
};

enum aw88194_mode_spk_rcv{
    AW88194_SPEAKER_MODE = 0,
    AW88194_RECEIVER_MODE = 1,
};

struct aw88194 {
    struct regmap *regmap;
    struct i2c_client *i2c;
    struct snd_soc_codec *codec;
    struct device *dev;
    struct mutex lock;
    struct hrtimer vbat_monitor_timer;
    struct work_struct vbat_monitor_work;

    int sysclk;
    int rate;
    int pstream;
    int cstream;

    int reset_gpio;
    int irq_gpio;

    unsigned char reg_addr;
    unsigned int dsp_addr;

    unsigned int dsp_cfg;
    unsigned int dsp_fw_len;
    unsigned int dsp_cfg_len;

    unsigned int flags;
    unsigned int chipid;
    unsigned int init;
    unsigned int spk_rcv_mode;

    unsigned int cfg_num;

    unsigned int cali_re;

    unsigned int vbat_monitor_flag;
    unsigned int vbat_monitor_limit_min;
    unsigned int bst_ipeak_limit;
    unsigned int bst_ipeak;
};

struct aw88194_container{
    int len;
    unsigned char data[];
};


#endif
