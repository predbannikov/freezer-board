#ifndef __FLASH_H
#define __FLASH_H

struct Calibration
{
  float slope;    // склон
  float intercept;//кусок прямой
//	 uint8_t id;
};//calibration;

struct Calibration calibr_LoadCell;
void Sensor_SetCalibration2f(struct Calibration *calibr, float slope, float intercept,int id);


#endif