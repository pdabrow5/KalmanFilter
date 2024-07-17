/*!
 * @file DFRobot_Bmx160.cpp
 * @brief define DFRobot_Bmx160 class infrastructure, the implementation of basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [luoyufeng] (yufeng.luo@dfrobot.com)
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-10-20
 * @url https://github.com/DFRobot/DFRobot_Bmx160
 */
#include "DFRobot_BMX160.h"
#include "../inc/common_porting.h"

#include "logger.h"

#define TRUE 1
#define FALSE 0
const float accelRange = Bmx160_ACCEL_MG_LSB_2G * 9.80665f;
const float gyroRange = Bmx160_GYRO_SENSITIVITY_250DPS;

sBmx160Dev_t Obmx160;

void Bmx160_init()
{
    Obmx160.delayMs = HAL_Delay;
    Bmx160_begin();
    DelayUs(100);
}

uint8_t Bmx160_begin()
{
    if (Bmx160_scan() == TRUE){
        Bmx160_softReset_();
        Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, 0x11);
        Obmx160.delayMs(50);
        /* Set gyro to normal mode */
        Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, 0x15);
        Obmx160.delayMs(100);
        /* Set mag to normal mode */
        Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, 0x19);
        Obmx160.delayMs(10);
        Bmx160_setMagnConf();
        return TRUE;
    }
    else
        return FALSE;
}

void Bmx160_setLowPower(){
    Bmx160_softReset_();
    Obmx160.delayMs(100);
    Bmx160_setMagnConf();
    Obmx160.delayMs(100);
    Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, 0x12);
    Obmx160.delayMs(100);
    /* Set gyro to normal mode */
    Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, 0x17);
    Obmx160.delayMs(100);
    /* Set mag to normal mode */
    Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, 0x1B);
    Obmx160.delayMs(100);
}

void Bmx160_wakeUp(){
    Bmx160_softReset_();
    Obmx160.delayMs(100);
    Bmx160_setMagnConf();
    Obmx160.delayMs(100);
    Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, 0x11);
    Obmx160.delayMs(100);
    /* Set gyro to normal mode */
    Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, 0x15);
    Obmx160.delayMs(100);
    /* Set mag to normal mode */
    Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, 0x19);
    Obmx160.delayMs(100);
}

uint8_t Bmx160_softReset_()
{
  int8_t rslt=Bmx160_OK;
  rslt = Bmx160_softReset(&Obmx160);
  if (rslt == 0)
    return TRUE;
  else
    return FALSE;
}

int8_t Bmx160_softReset(sBmx160Dev_t *dev)
{
  int8_t rslt=Bmx160_OK;
  uint8_t data = Bmx160_SOFT_RESET_CMD;
  if (dev==NULL){
    rslt = Bmx160_E_NULL_PTR;
  }
  Bmx160_writeBmxReg(Bmx160_COMMAND_REG_ADDR, data);
  Obmx160.delayMs(Bmx160_SOFT_RESET_DELAY_MS);
  if (rslt == Bmx160_OK){
    Bmx160_defaultParamSettg(dev);
  }
  return rslt;
}

void Bmx160_defaultParamSettg(sBmx160Dev_t *dev)
{
  // Initializing accel and gyro params with
  dev->gyroCfg.bw = Bmx160_GYRO_BW_NORMAL_MODE;
  dev->gyroCfg.odr = Bmx160_GYRO_ODR_200HZ;
  dev->gyroCfg.power = Bmx160_GYRO_SUSPEND_MODE;
  dev->gyroCfg.range = Bmx160_GYRO_RANGE_250_DPS;
  dev->accelCfg.bw = Bmx160_ACCEL_BW_NORMAL_AVG4;
  dev->accelCfg.odr = Bmx160_ACCEL_ODR_200HZ;
  dev->accelCfg.power = Bmx160_ACCEL_SUSPEND_MODE;
  dev->accelCfg.range = Bmx160_ACCEL_RANGE_2G;


  dev->prevMagnCfg = dev->magnCfg;
  dev->prevGyroCfg = dev->gyroCfg;
  dev->prevAccelCfg = dev->accelCfg;
}

void Bmx160_setMagnConf()
{
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_0_ADDR, 0x80);
    Obmx160.delayMs(50);
    // Sleep mode
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_3_ADDR, 0x01);
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_2_ADDR, 0x4B);
    // REPXY regular preset
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_3_ADDR, 0x04);
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_2_ADDR, 0x51);
    // REPZ regular preset
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_3_ADDR, 0x0E);
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_2_ADDR, 0x52);

    Bmx160_writeBmxReg(Bmx160_MAGN_IF_3_ADDR, 0x02);
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_2_ADDR, 0x4C);
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_1_ADDR, 0x42);
    Bmx160_writeBmxReg(Bmx160_MAGN_CONFIG_ADDR, 0x08);
    Bmx160_writeBmxReg(Bmx160_MAGN_IF_0_ADDR, 0x03);
    Obmx160.delayMs(50);
}

void Bmx160_getAllData(sBmx160SensorData_t *magn, sBmx160SensorData_t *gyro, sBmx160SensorData_t *accel){

    uint8_t data[23] = {0};
    int16_t x=0,y=0,z=0;
    uint32_t time = 0;
    float time_f;
    Bmx160_readReg(Bmx160_MAG_DATA_ADDR, data, 23);
    time = (uint32_t) ((uint32_t)(((uint16_t)data[22] << 8) | data[21]) << 8 | data[20]);
    time_f = ((float)(time)) * 0.039f;
    if(magn){
        x = (int16_t) (((uint16_t)data[1] << 8) | data[0]);
        y = (int16_t) (((uint16_t)data[3] << 8) | data[2]);
        z = (int16_t) (((uint16_t)data[5] << 8) | data[4]);
        //LOG("X: %d, Y: %d, Z: %d", x, y, z);
        magn->x = x * Bmx160_MAGN_UT_LSB_XY;
        magn->y = y * Bmx160_MAGN_UT_LSB_XY;
        magn->z = z * Bmx160_MAGN_UT_LSB_Z;
//        magn->x = x * Bmx160_MAGN_UT_LSB_XY;
//		magn->y = y * Bmx160_MAGN_UT_LSB_XY;
//		magn->z = z * Bmx160_MAGN_UT_LSB_XY;
        magn->sensortime = time_f;
    }
    if(gyro){
        x = (int16_t) (((uint16_t)data[9] << 8) | data[8]);
        y = (int16_t) (((uint16_t)data[11] << 8) | data[10]);
        z = (int16_t) (((uint16_t)data[13] << 8) | data[12]);
        gyro->x = x * gyroRange;
        gyro->y = y * gyroRange;
        gyro->z = z * gyroRange;
        gyro->sensortime = time_f;
    }
    if(accel){
        x = (int16_t) (((uint16_t)data[15] << 8) | data[14]);
        y = (int16_t) (((uint16_t)data[17] << 8) | data[16]);
        z = (int16_t) (((uint16_t)data[19] << 8) | data[18]);
        accel->x = x * accelRange;
        accel->y = y * accelRange;
        accel->z = z * accelRange;
        accel->sensortime = time_f;
    }
}

void Bmx160_writeBmxReg(uint8_t reg, uint8_t value)
{
    uint8_t buffer[1] = {value};
    Bmx160_writeReg(reg, buffer, 1);
}

void Bmx160_writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
    SensorAPI_I2Cx_Write(0, reg, pBuf, len);
}

void Bmx160_readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
    SensorAPI_I2Cx_Read(0, reg, pBuf, len);
}

uint8_t Bmx160_scan()
{
    return TRUE;
}
