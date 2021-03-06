#ifndef _AW88194_REG_H_
#define _AW88194_REG_H_

/********************************************
 * Register List
 *******************************************/
#define AW88194_REG_ID              (0x00)
#define AW88194_REG_SYSST           (0x01)
#define AW88194_REG_SYSINT          (0x02)
#define AW88194_REG_SYSINTM         (0x03)
#define AW88194_REG_SYSCTRL         (0x04)
#define AW88194_REG_I2SCTRL         (0x05)
#define AW88194_REG_I2SCFG1         (0x06)
#define AW88194_REG_PWMCTRL         (0x08)
#define AW88194_REG_HAGCCFG1        (0x09)
#define AW88194_REG_HAGCCFG2        (0x0A)
#define AW88194_REG_HAGCCFG3        (0x0B)
#define AW88194_REG_HAGCCFG4        (0x0C)
#define AW88194_REG_HAGCCFG5        (0x0D)
#define AW88194_REG_HAGCCFG6        (0x0E)
#define AW88194_REG_HAGCCFG7        (0x0F)
#define AW88194_REG_HAGCCFG8        (0x10)
#define AW88194_REG_SYSCTRL2        (0x11)
#define AW88194_REG_PRODID          (0x12)
#define AW88194_REG_DBGCTRL         (0x20)
#define AW88194_REG_I2SCFG2         (0x21)
#define AW88194_REG_I2SSTAT         (0x22)
#define AW88194_REG_I2SCAPCNT       (0x23)
#define AW88194_REG_TM              (0x34)
#define AW88194_REG_CRCIN           (0x38)
#define AW88194_REG_CRCOUT          (0x39)
#define AW88194_REG_DSPMADD         (0x40)
#define AW88194_REG_DSPMDAT         (0x41)
#define AW88194_REG_WDT             (0x42)
#define AW88194_REG_ACR1            (0x43)
#define AW88194_REG_ACR2            (0x44)
#define AW88194_REG_ASR1            (0x45)
#define AW88194_REG_ASR2            (0x46)
#define AW88194_REG_DSPCFG          (0x47)
#define AW88194_REG_ASR3            (0x48)
#define AW88194_REG_ASR4            (0x49)
#define AW88194_REG_BSTCTRL1        (0x60)
#define AW88194_REG_BSTCTRL2        (0x61)
#define AW88194_REG_BSTCTRL3        (0x62)
#define AW88194_REG_BSTDBG1         (0x63)
#define AW88194_REG_BSTDBG2         (0x64)
#define AW88194_REG_PLLCTRL1        (0x65)
#define AW88194_REG_PLLCTRL2        (0x66)
#define AW88194_REG_AMPDBG1         (0x67)
#define AW88194_REG_AMPDBG2         (0x68)
#define AW88194_REG_CDACTRL1        (0x69)
#define AW88194_REG_CDACTRL2        (0x6A)
#define AW88194_REG_ISECTRL1        (0x6B)
#define AW88194_REG_SADCCTRL        (0x6C)
#define AW88194_REG_VBAT            (0x6D)
#define AW88194_REG_TEMP            (0x6E)
#define AW88194_REG_TEST            (0x70)
#define AW88194_REG_TEST2           (0x71)
#define AW88194_REG_EFCTR1          (0x72)
#define AW88194_REG_EFCTR2          (0x73)
#define AW88194_REG_EFWH            (0x74)
#define AW88194_REG_EFWM            (0x75)
#define AW88194_REG_EFWL            (0x76)
#define AW88194_REG_EFRH            (0x77)
#define AW88194_REG_EFRM            (0x78)
#define AW88194_REG_EFRL            (0x79)
#define AW88194_REG_TESTDET         (0x7A)

#define AW88194_REG_MAX             (0x7F)

/********************************************
 * Register Access
 *******************************************/
#define REG_NONE_ACCESS             (0)
#define REG_RD_ACCESS               (1 << 0)
#define REG_WR_ACCESS               (1 << 1)

const unsigned char aw88194_reg_access[AW88194_REG_MAX] = {
    [AW88194_REG_ID        ] = REG_RD_ACCESS,
    [AW88194_REG_SYSST     ] = REG_RD_ACCESS,
    [AW88194_REG_SYSINT    ] = REG_RD_ACCESS,
    [AW88194_REG_SYSINTM   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_SYSCTRL   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_I2SCTRL   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_I2SCFG1   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_PWMCTRL   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_HAGCCFG1  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_HAGCCFG2  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_HAGCCFG3  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_HAGCCFG4  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_HAGCCFG5  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_HAGCCFG6  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_HAGCCFG7  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_HAGCCFG8  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_SYSCTRL2  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_PRODID    ] = REG_RD_ACCESS,
    [AW88194_REG_DBGCTRL   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_I2SCFG2   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_I2SSTAT   ] = REG_RD_ACCESS,
    [AW88194_REG_I2SCAPCNT ] = REG_RD_ACCESS,
    [AW88194_REG_TM        ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_CRCIN     ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_CRCOUT    ] = REG_RD_ACCESS,
    [AW88194_REG_DSPMADD   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_DSPMDAT   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_WDT       ] = REG_RD_ACCESS,
    [AW88194_REG_ACR1      ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_ACR2      ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_ASR1      ] = REG_RD_ACCESS,
    [AW88194_REG_ASR2      ] = REG_RD_ACCESS,
    [AW88194_REG_DSPCFG    ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_ASR3      ] = REG_RD_ACCESS,
    [AW88194_REG_ASR4      ] = REG_RD_ACCESS,
    [AW88194_REG_BSTCTRL1  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_BSTCTRL2  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_BSTCTRL3  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_BSTDBG1   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_BSTDBG2   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_PLLCTRL1  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_PLLCTRL2  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_AMPDBG1   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_AMPDBG2   ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_CDACTRL1  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_CDACTRL2  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_ISECTRL1  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_SADCCTRL  ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_VBAT      ] = REG_RD_ACCESS,
    [AW88194_REG_TEMP      ] = REG_RD_ACCESS,
    [AW88194_REG_TEST      ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_TEST2     ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_EFCTR1    ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_EFCTR2    ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_EFWH      ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_EFWM      ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_EFWL      ] = REG_RD_ACCESS|REG_WR_ACCESS,
    [AW88194_REG_EFRH      ] = REG_RD_ACCESS,
    [AW88194_REG_EFRM      ] = REG_RD_ACCESS,
    [AW88194_REG_EFRL      ] = REG_RD_ACCESS,
    [AW88194_REG_TESTDET   ] = REG_RD_ACCESS,
};

/******************************************************
 * Register Detail
 *****************************************************/
// SYSST
#define AW88194_BIT_SYSST_OVP2S                     (1<<15)
#define AW88194_BIT_SYSST_UVLOS                     (1<<14)
#define AW88194_BIT_SYSST_ADPS                      (1<<13)
#define AW88194_BIT_SYSST_DSPS                      (1<<12)
#define AW88194_BIT_SYSST_BSTOCS                    (1<<11)
#define AW88194_BIT_SYSST_OVPS                      (1<<10)
#define AW88194_BIT_SYSST_BSTS                      (1<< 9)
#define AW88194_BIT_SYSST_SWS                       (1<< 8)
#define AW88194_BIT_SYSST_CLIPS                     (1<< 7)
#define AW88194_BIT_SYSST_WDS                       (1<< 6)
#define AW88194_BIT_SYSST_NOCLKS                    (1<< 5)
#define AW88194_BIT_SYSST_CLKS                      (1<< 4)
#define AW88194_BIT_SYSST_OCDS                      (1<< 3)
#define AW88194_BIT_SYSST_CLIP_PRES                 (1<< 2)
#define AW88194_BIT_SYSST_OTHS                      (1<< 1)
#define AW88194_BIT_SYSST_PLLS                      (1<< 0)

// SYSINT
#define AW88194_BIT_SYSINT_OVP2I                    ( 1<<15)
#define AW88194_BIT_SYSINT_UVLOI                    ( 1<<14)
#define AW88194_BIT_SYSINT_ADPI                     ( 1<<13)
#define AW88194_BIT_SYSINT_DSPI                     ( 1<<12)
#define AW88194_BIT_SYSINT_BSTOCI                   ( 1<<11)
#define AW88194_BIT_SYSINT_OVPI                     ( 1<<10)
#define AW88194_BIT_SYSINT_BSTI                     ( 1<< 9)
#define AW88194_BIT_SYSINT_SWI                      ( 1<< 8)
#define AW88194_BIT_SYSINT_CLIPI                    ( 1<< 7)
#define AW88194_BIT_SYSINT_WDI                      ( 1<< 6)
#define AW88194_BIT_SYSINT_NOCLKI                   ( 1<< 5)
#define AW88194_BIT_SYSINT_CLKI                     ( 1<< 4)
#define AW88194_BIT_SYSINT_OCDI                     ( 1<< 3)
#define AW88194_BIT_SYSINT_CLIP_PREI                ( 1<< 2)
#define AW88194_BIT_SYSINT_OTHI                     ( 1<< 1)
#define AW88194_BIT_SYSINT_PLLI                     ( 1<< 0)

// SYSINTM
#define AW88194_BIT_SYSINTM_OVP2SM                  ( 1<<15)
#define AW88194_BIT_SYSINTM_UVLOM                   ( 1<<14)
#define AW88194_BIT_SYSINTM_ADPM                    ( 1<<13)
#define AW88194_BIT_SYSINTM_DSPM                    ( 1<<12)
#define AW88194_BIT_SYSINTM_BSTOCM                  ( 1<<11)
#define AW88194_BIT_SYSINTM_OVPM                    ( 1<<10)
#define AW88194_BIT_SYSINTM_BSTM                    ( 1<< 9)
#define AW88194_BIT_SYSINTM_SWM                     ( 1<< 8)
#define AW88194_BIT_SYSINTM_CLIPM                   ( 1<< 7)
#define AW88194_BIT_SYSINTM_WDM                     ( 1<< 6)
#define AW88194_BIT_SYSINTM_NOCLKM                  ( 1<< 5)
#define AW88194_BIT_SYSINTM_CLKM                    ( 1<< 4)
#define AW88194_BIT_SYSINTM_OCDM                    ( 1<< 3)
#define AW88194_BIT_SYSINTM_CLIP_PREM               ( 1<< 2)
#define AW88194_BIT_SYSINTM_OTHM                    ( 1<< 1)
#define AW88194_BIT_SYSINTM_PLLM                    ( 1<< 0)

// SYSCTRL
#define AW88194_BIT_SYSCTRL_MODE_MASK               (~( 1<< 7))
#define AW88194_BIT_SYSCTRL_RCV_MODE                ( 1<< 7)
#define AW88194_BIT_SYSCTRL_SPK_MODE                ( 0<< 7)
#define AW88194_BIT_SYSCTRL_DSP_MASK                (~( 1<< 2))
#define AW88194_BIT_SYSCTRL_DSP_BYPASS              ( 1<< 2)
#define AW88194_BIT_SYSCTRL_DSP_WORK                ( 0<< 2)
#define AW88194_BIT_SYSCTRL_PW_MASK                 (~( 1<< 0))
#define AW88194_BIT_SYSCTRL_PW_PDN                  ( 1<< 0)
#define AW88194_BIT_SYSCTRL_PW_ACTIVE               ( 0<< 0)

// I2SCTRL
#define AW88194_BIT_I2SCTRL_INPLEV_MASK             (~( 1<<13))
#define AW88194_BIT_I2SCTRL_INPLEV_0DB              ( 1<<13)
#define AW88194_BIT_I2SCTRL_INPLEV_NEG_6DB          ( 0<<13)
#define AW88194_BIT_I2SCTRL_STEREO_MASK             (~( 1<<12))
#define AW88194_BIT_I2SCTRL_STEREO_ENABLE           ( 1<<12)
#define AW88194_BIT_I2SCTRL_STEREO_DISABLE          ( 0<<12)
#define AW88194_BIT_I2SCTRL_CHS_MASK                (~( 3<<10))
#define AW88194_BIT_I2SCTRL_CHS_MONO                ( 3<<10)
#define AW88194_BIT_I2SCTRL_CHS_RIGHT               ( 2<<10)
#define AW88194_BIT_I2SCTRL_CHS_LEFT                ( 1<<10)
#define AW88194_BIT_I2SCTRL_MD_MASK                 (~( 3<< 8))
#define AW88194_BIT_I2SCTRL_MD_LSB                  ( 2<< 8)
#define AW88194_BIT_I2SCTRL_MD_MSB                  ( 1<< 8)
#define AW88194_BIT_I2SCTRL_MD_STD                  ( 0<< 8)
#define AW88194_BIT_I2SCTRL_FMS_MASK                (~( 3<< 6))
#define AW88194_BIT_I2SCTRL_FMS_32BIT               ( 3<< 6)
#define AW88194_BIT_I2SCTRL_FMS_24BIT               ( 2<< 6)
#define AW88194_BIT_I2SCTRL_FMS_20BIT               ( 1<< 6)
#define AW88194_BIT_I2SCTRL_FMS_16BIT               ( 0<< 6)
#define AW88194_BIT_I2SCTRL_BCK_MASK                (~( 3<< 4))
#define AW88194_BIT_I2SCTRL_BCK_64FS                ( 2<< 4)
#define AW88194_BIT_I2SCTRL_BCK_48FS                ( 1<< 4)
#define AW88194_BIT_I2SCTRL_BCK_32FS                ( 0<< 4)
#define AW88194_BIT_I2SCTRL_SR_MASK                 (~(15<< 0))
#define AW88194_BIT_I2SCTRL_SR_192K                 (10<< 0)
#define AW88194_BIT_I2SCTRL_SR_96K                  ( 9<< 0)
#define AW88194_BIT_I2SCTRL_SR_48K                  ( 8<< 0)
#define AW88194_BIT_I2SCTRL_SR_44P1K                ( 7<< 0)
#define AW88194_BIT_I2SCTRL_SR_32K                  ( 6<< 0)
#define AW88194_BIT_I2SCTRL_SR_24K                  ( 5<< 0)
#define AW88194_BIT_I2SCTRL_SR_22K                  ( 4<< 0)
#define AW88194_BIT_I2SCTRL_SR_16K                  ( 3<< 0)
#define AW88194_BIT_I2SCTRL_SR_12K                  ( 2<< 0)
#define AW88194_BIT_I2SCTRL_SR_11K                  ( 1<< 0)
#define AW88194_BIT_I2SCTRL_SR_8K                   ( 0<< 0)


// PWMCTRL
#define AW88194_BIT_PWMCTRL_HMUTE_MASK              (~( 1<< 0))
#define AW88194_BIT_PWMCTRL_HMUTE_ENABLE            ( 1<< 0)
#define AW88194_BIT_PWMCTRL_HMUTE_DISABLE           ( 0<< 0)



// SYSCTRL2
#define AW88194_BIT_SYSCTRL2_MEMCLK_MASK            (~( 1<<12))
#define AW88194_BIT_SYSCTRL2_MEMCLK_PLL             ( 1<<12)
#define AW88194_BIT_SYSCTRL2_MEMCLK_OSC             ( 0<<12)
#define AW88194_BIT_SYSCTRL2_BST_IPEAK_MASK         (~( 7<< 0))

// HAGCCFG7
#define AW88194_BIT_HAGCCFG7_VOL_MASK               (~(255<< 8))
#define AW88194_VOLUME_MAX                          (0)
#define AW88194_VOLUME_MIN                          (-255)
#define AW88194_VOL_REG_SHIFT                       (8)


// BSTCTRL3
#define AW88194_BIT_BSTCTRL3_BST_VOUT_MASK          (~(15<< 0))
#define AW88194_BIT_BSTCTRL3_BST_VOUT_10P25V        (15<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_10V           (14<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_9P75V         (13<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_9P5V          (12<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_9P25V         (11<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_9V            (10<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_8P75V         ( 9<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_8P5V          ( 8<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_8P25V         ( 7<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_8V            ( 6<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_7P75V         ( 5<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_7P5V          ( 4<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_7P25V         ( 3<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_7V            ( 2<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_6P75V         ( 1<< 0)
#define AW88194_BIT_BSTCTRL3_BST_VOUT_6P5V          ( 0<< 0)

/********************************************
 * Register List
 *******************************************/
#define AW88194_DSP_REG_CFG_ADPZ_RE                 0x8677


#endif
