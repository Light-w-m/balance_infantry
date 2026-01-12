#ifndef ROBOT_H
#define ROBOT_H

#include "spi.h"

// extern SPI_HandleTypeDef hspi2;

void RobotInit(void);
void MotorTask(void);

#endif // ROBOT_H