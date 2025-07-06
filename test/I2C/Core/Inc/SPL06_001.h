#ifndef __SPL06_H
#define __SPL06_H

#include "stm32g4xx.h"
#include "i2c.h"
/**< \brief �Ĵ�����ַ */
#define SP06_PSR_B2     0x00
#define SP06_PSR_B1     0x01
#define SP06_PSR_B0     0x02
#define SP06_TMP_B2     0x03
#define SP06_TMP_B1     0x04
#define SP06_TMP_B0     0x05

#define SP06_PSR_CFG    0x06
#define SP06_TMP_CFG    0x07
#define SP06_MEAS_CFG   0x08

#define SP06_CFG_REG    0x09
#define SP06_INT_STS    0x0A
#define SP06_FIFO_STS   0x0B

#define SP06_RESET      0x0C
#define SP06_ID         0x0D

#define SP06_COEF       0x10
#define SP06_COEF_SRCE  0x28

/**< \brief ��ѹ��������(sample/sec),Background ģʽʹ�� */
#define PM_RATE_1           (0<<4)      /* 1 measurements pr. sec.   */
#define PM_RATE_2           (1<<4)      /* 2 measurements pr. sec.   */
#define PM_RATE_4           (2<<4)      /* 4 measurements pr. sec.   */      
#define PM_RATE_8           (3<<4)      /* 8 measurements pr. sec.   */
#define PM_RATE_16          (4<<4)      /* 16 measurements pr. sec.  */
#define PM_RATE_32          (5<<4)      /* 32 measurements pr. sec.  */
#define PM_RATE_64          (6<<4)      /* 64 measurements pr. sec.  */
#define PM_RATE_128         (7<<4)      /* 128 measurements pr. sec. */

/**< \brief ��ѹ�ز�������(times),Background ģʽʹ�� */
#define PM_PRC_1            0           /* Sigle        kP=524288,  3.6ms   */
#define PM_PRC_2            1           /* 2 times      kP=1572864, 5.2ms   */
#define PM_PRC_4            2           /* 4 times      kP=3670016, 8.4ms   */
#define PM_PRC_8            3           /* 8 times      kP=7864320, 14.8ms  */
#define PM_PRC_16           4           /* 16 times     kP=253952,  27.6ms  */
#define PM_PRC_32           5           /* 32 times     kP=516096,  53.2ms  */
#define PM_PRC_64           6           /* 64 times     kP=1040384, 104.4ms */
#define PM_PRC_128          7           /* 128 times    kP=2088960, 206.8ms */

/**< \brief �¶Ȳ�������(sample/sec),Background ģʽʹ�� */
#define TMP_RATE_1          (0<<4)      /* 1 measurements pr. sec.   */
#define TMP_RATE_2          (1<<4)      /* 2 measurements pr. sec.   */
#define TMP_RATE_4          (2<<4)      /* 4 measurements pr. sec.   */        
#define TMP_RATE_8          (3<<4)      /* 8 measurements pr. sec.   */
#define TMP_RATE_16         (4<<4)      /* 16 measurements pr. sec.  */
#define TMP_RATE_32         (5<<4)      /* 32 measurements pr. sec.  */
#define TMP_RATE_64         (6<<4)      /* 64 measurements pr. sec.  */
#define TMP_RATE_128        (7<<4)      /* 128 measurements pr. sec. */

/**< \brief �¶��ز�������(times),Background ģʽʹ�� */
#define TMP_PRC_1           0           /* Sigle     */
#define TMP_PRC_2           1           /* 2 times   */
#define TMP_PRC_4           2           /* 4 times   */
#define TMP_PRC_8           3           /* 8 times   */
#define TMP_PRC_16          4           /* 16 times  */
#define TMP_PRC_32          5           /* 32 times  */
#define TMP_PRC_64          6           /* 64 times  */
#define TMP_PRC_128         7           /* 128 times */

/**< \brief SPL06_MEAS_CFG */
#define MEAS_COEF_RDY                   0x80
#define MEAS_SENSOR_RDY                 0x40    /* ��������ʼ����� */
#define MEAS_TMP_RDY                    0x20    /* ���µ��¶�����   */
#define MEAS_PRS_RDY                    0x10    /* ���µ���ѹ����   */

#define MEAS_CTRL_Standby               0x00    /* ����ģʽ         */
#define MEAS_CTRL_PressMeasure          0x01    /* ������ѹ����     */
#define MEAS_CTRL_TempMeasure           0x02    /* �����¶Ȳ���     */
#define MEAS_CTRL_ContinuousPress       0x05    /* ������ѹ����     */
#define MEAS_CTRL_ContinuousTemp        0x06    /* �����¶Ȳ���     */
#define MEAS_CTRL_ContinuousPressTemp   0x07    /* ������ѹ�¶Ȳ��� */

/**< \brief FIFO_STS */
#define SPL06_FIFO_FULL                 0x02
#define SPL06_FIFO_EMPTY                0x01

/**< \brief INT_STS */
#define SPL06_INT_FIFO_FULL             0x04
#define SPL06_INT_TMP                   0x02
#define SPL06_INT_PRS                   0x01

/**< \brief CFG_REG */
#define SPL06_CFG_T_SHIFT               0x08    /* oversampling times>8ʱ����ʹ�� */
#define SPL06_CFG_P_SHIFT               0x04    /* oversampling times>8ʱ����ʹ�� */

#define SPL06_ADDR  0xee

typedef struct
{
    int32_t Praw;   /* ��ѹԭʼֵ */
    int32_t Traw;   /* �¶�ԭʼֵ */
    float   Pcomp;  /* ��������ѹֵ */
    float   Tcomp;  /* �������¶�ֵ */
} spl06_result_t;
extern spl06_result_t spl06_result;

int spl06_init(void);
void spl06_get_raw_data();
void spl06_get_result(float* press,float* temp);

#endif
