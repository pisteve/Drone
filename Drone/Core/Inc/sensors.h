/*
 * sensors.h
 *
 *  Created on: Feb 29, 2020
 *      Author: sp
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"
/* High Speed Filter Enabled */
void L3GD20_Init(uint8_t*, uint8_t*, uint8_t*);
void L3GD20_Calibrate();
void L3GD20_XYZrates (float*);


/******************************************************************************/
/*********************** L3GD20 START REGISTER MAPPING ************************/
/******************************************************************************/
#define L3GD20_WHO_AM_I_ADDR          (uint8_t*)0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         (uint8_t*)0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         (uint8_t*)0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         (uint8_t*)0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         (uint8_t*)0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         (uint8_t*)0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     (uint8_t*)0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          (uint8_t*)0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        (uint8_t*)0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           (uint8_t*)0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           (uint8_t*)0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           (uint8_t*)0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           (uint8_t*)0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           (uint8_t*)0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           (uint8_t*)0x2D  /* Output Register Z */
#define L3GD20_FIFO_CTRL_REG_ADDR     (uint8_t*)0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      (uint8_t*)0x2F  /* Fifo src Register */

#define L3GD20_INT1_CFG_ADDR          (uint8_t*)0x30  /* Interrupt 1 configuration Register */
#define L3GD20_INT1_SRC_ADDR          (uint8_t*)0x31  /* Interrupt 1 source Register */
#define L3GD20_INT1_TSH_XH_ADDR       (uint8_t*)0x32  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_XL_ADDR       (uint8_t*)0x33  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_YH_ADDR       (uint8_t*)0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_YL_ADDR       (uint8_t*)0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_ZH_ADDR       (uint8_t*)0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_TSH_ZL_ADDR       (uint8_t*)0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_DURATION_ADDR     (uint8_t*)0x38  /* Interrupt 1 DURATION register */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define I_AM_L3GD20                 ((uint8_t)0xD4)
#define I_AM_L3GD20_TR              ((uint8_t)0xD5)

/* Power_Mode_selection */
#define L3GD20_MODE_POWERDOWN       ((uint8_t)0x00)
#define L3GD20_MODE_ACTIVE          ((uint8_t)0x08)

/* OutPut_DataRate_Selection */
#define L3GD20_OUTPUT_DATARATE_1    ((uint8_t)0x00)
#define L3GD20_OUTPUT_DATARATE_2    ((uint8_t)0x40)
#define L3GD20_OUTPUT_DATARATE_3    ((uint8_t)0x80)
#define L3GD20_OUTPUT_DATARATE_4    ((uint8_t)0xC0)

/* Axes_Selection */
#define L3GD20_X_ENABLE            ((uint8_t)0x02)
#define L3GD20_Y_ENABLE            ((uint8_t)0x01)
#define L3GD20_Z_ENABLE            ((uint8_t)0x04)
#define L3GD20_AXES_ENABLE         ((uint8_t)0x07)
#define L3GD20_AXES_DISABLE        ((uint8_t)0x00)

/* Bandwidth_Selection */
#define L3GD20_BANDWIDTH_1         ((uint8_t)0x00)
#define L3GD20_BANDWIDTH_2         ((uint8_t)0x10)
#define L3GD20_BANDWIDTH_3         ((uint8_t)0x20)
#define L3GD20_BANDWIDTH_4         ((uint8_t)0x30)

/* Full_Scale_Selection */
#define L3GD20_FULLSCALE_250       ((uint8_t)0x00)
#define L3GD20_FULLSCALE_500       ((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000      ((uint8_t)0x20)
#define L3GD20_FULLSCALE_SELECTION ((uint8_t)0x30)

/* Full_Scale_Sensitivity */
#define L3GD20_SENSITIVITY_250DPS  ((float).00875f)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_500DPS  ((float).01750f)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_2000DPS ((float).07000f)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */


/* Block_Data_Update */
#define L3GD20_BlockDataUpdate_Continous   ((uint8_t)0x00)
#define L3GD20_BlockDataUpdate_Single      ((uint8_t)0x80)


/* Endian_Data_selection */
#define L3GD20_BLE_LSB                     ((uint8_t)0x00)
#define L3GD20_BLE_MSB	                   ((uint8_t)0x40)

/* High_Pass_Filter_status */
#define L3GD20_HIGHPASSFILTER_DISABLE      ((uint8_t)0x00)
#define L3GD20_HIGHPASSFILTER_ENABLE	     ((uint8_t)0x10)

/* INT1_INT2_selection */
#define L3GD20_INT1                        ((uint8_t)0x00)
#define L3GD20_INT2                        ((uint8_t)0x01)

/* INT1_Interrupt_status */
#define L3GD20_INT1INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT1INTERRUPT_ENABLE        ((uint8_t)0x80)

/* INT2_Interrupt_status */

#define L3GD20_INT2INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT2INTERRUPT_ENABLE        ((uint8_t)0x08)

/* INT1_Interrupt_ActiveEdge */
#define L3GD20_INT1INTERRUPT_LOW_EDGE      ((uint8_t)0x20)
#define L3GD20_INT1INTERRUPT_HIGH_EDGE     ((uint8_t)0x00)

/* Boot_Mode_selection */
#define L3GD20_BOOT_NORMALMODE             ((uint8_t)0x00)
#define L3GD20_BOOT_REBOOTMEMORY           ((uint8_t)0x80)

/* High_Pass_Filter_Mode */
#define L3GD20_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define L3GD20_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define L3GD20_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define L3GD20_HPM_AUTORESET_INT           ((uint8_t)0x30)

/* High_Pass_CUT OFF_Frequency */
#define L3GD20_HPFCF_0              0x00
#define L3GD20_HPFCF_1              0x01
#define L3GD20_HPFCF_2              0x02
#define L3GD20_HPFCF_3              0x03
#define L3GD20_HPFCF_4              0x04
#define L3GD20_HPFCF_5              0x05
#define L3GD20_HPFCF_6              0x06
#define L3GD20_HPFCF_7              0x07
#define L3GD20_HPFCF_8              0x08
#define L3GD20_HPFCF_9              0x09

#endif /* INC_SENSORS_H_ */
