#ifndef _MOTO_H_
#define _MOTO_H_

#include "USER_Moto.h"
#include "PID.h"
#include "super_cap.h"


typedef struct CHASSIS
{
	DJI_Motor_t M3508[4];
} ChassisMotor_t;


extern ChassisMotor_t chassis;
void Supercap_Update(SuperCap* this, uint8_t* rxdata);

#endif
