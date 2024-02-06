/**
 * Kondo B3M 3.0 Library (Header)
 *
 * Copyright 2016 - Yasuo Hayashibara (yasuo@hayashibara.net)
 * Chiba Institute of Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef B3M_H_
#define B3M_H_

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <sys/time.h>
#include "ftdi.h"

// some options
//#define B3M_BAUD 115200
#define B3M_USB_VID 0x165c
#define B3M_USB_PID 0x0009  // setting for RS485USB
#define B3M_RX_TIMEOUT 1000000
#define B3M_POS_TIMEOUT 2000
#define B3M_GET_TIMEOUT 2000
#define B3M_SET_TIMEOUT 2000
#define B3M_ID_TIMEOUT 2000

#define B3M_BAUD B38400 // custom bitrate

// b3m commands
#define B3M_CMD_LOAD					0x01
#define B3M_CMD_SAVE					0x02
#define B3M_CMD_READ					0x03
#define B3M_CMD_WRITE					0x04
#define B3M_CMD_POSITION				0x06
#define B3M_CMD_RESET					0x05

#define B3M_RETURN_ERROR_STATUS			0x00
#define B3M_RETURN_SYSTEM_STATUS		0x01
#define B3M_RETURN_MOTOR_STATUS			0x02
#define B3M_RETURN_UART_STATUS			0x03
#define B3M_RETURN_COMMAND_STATUS		0x04

#define B3M_SYSTEM_ID					0x00
#define B3M_SYSTEM_BAUDRATE				0x01
#define B3M_SYSTEM_POSITION_MIN			0x05
#define B3M_SYSTEM_POSITION_MAX			0x07
#define B3M_SYSTEM_POSITION_CENTER		0x09
#define B3M_SYSTEM_MCU_TEMP_LIMIT		0x0B
#define B3M_SYSTEM_MCU_TEMP_LIMIT_PR	0x0D
#define B3M_SYSTEM_MOTOR_TEMP_LIMIT		0x0E
#define B3M_SYSTEM_MOTOR_TEMP_LIMIT_PR	0x10
#define B3M_SYSTEM_CURRENT_LIMIT		0x11
#define B3M_SYSTEM_CURRENT_LIMIT_PR		0x13
#define B3M_SYSTEM_LOCKDETECT_TIME		0x14
#define B3M_SYSTEM_LOCKDETECT_OUTRATE	0x15
#define B3M_SYSTEM_LOCKDETECT_TIME_PR	0x16
#define B3M_SYSTEM_INPUT_VOLTAGE_MIN	0x17
#define B3M_SYSTEM_INPUT_VOLTAGE_MAX	0x19
#define B3M_SYSTEM_TORQUE_LIMIT			0x1B
#define B3M_SYSTEM_DEADBAND_WIDTH		0x1C
#define B3M_SYSTEM_MOTOR_CW_RATIO		0x22
#define B3M_SYSTEM_MOTOR_CCW_RATIO		0x23

#define B3M_SERVO_SERVO_OPTION			0x27
#define B3M_SERVO_SERVO_MODE			0x28
#define B3M_SERVO_TORQUE_ON				0x28
#define B3M_SERVO_RUN_MODE				0x29
#define B3M_SERVO_DESIRED_POSITION		0x2A
#define B3M_SERVO_CURRENT_POSITION		0x2C
#define B3M_SERVO_PREVIOUS_POSITION		0x2E
#define B3M_SERVO_DESIRED_VELOSITY		0x30
#define B3M_SERVO_CURRENT_VELOSITY		0x32
#define B3M_SERVO_PREVIOUS_VELOSITY		0x34
#define B3M_SERVO_DESIRED_TIME			0x36
#define B3M_SERVO_RUNNING_TIME			0x38
#define B3M_SERVO_WORKING_TIME			0x3A
#define B3M_SERVO_DESIRED_TORQUE		0x3C
#define B3M_SERVO_SYSTEM_CLOCK			0x3E
#define B3M_SERVO_SAMPLING_TIME			0x42
#define B3M_SERVO_MCU_TEMP				0x44
#define B3M_SERVO_MOTOR_TEMP			0x46
#define B3M_SERVO_CURRENT				0x48
#define B3M_SERVO_INPUT_VOLTAGE			0x4A
#define B3M_SERVO_PWM_DUTY				0x4C
#define B3M_SERVO_PWM_FREQUENCY			0x4E
#define B3M_SERVO_ENCODER_VALUE			0x50
#define B3M_SERVO_ENCODER_COUNT			0x52
#define B3M_SERVO_HALLIC_STATE			0x56

#define B3M_CONTROL_CONTROL_LOW			0x5C
#define B3M_CONTROL_GAIN_PRESETNO		0x5C
#define B3M_CONTROL_TYPE				0x5D
#define B3M_CONTROL_KP0					0x5E
#define B3M_CONTROL_KD0					0x62
#define B3M_CONTROL_KI0					0x66
#define B3M_CONTROL_STATIC_FRICTION0	0x6A
#define B3M_CONTROL_DYNAMIC_FRICTION0	0x6C
#define B3M_CONTROL_KP1					0x6E
#define B3M_CONTROL_KD1					0x72
#define B3M_CONTROL_KI1					0x76
#define B3M_CONTROL_STATIC_FRICTION1	0x7A
#define B3M_CONTROL_DYNAMIC_FRICTION1	0x7C
#define B3M_CONTROL_KP2					0x7E
#define B3M_CONTROL_KD2					0x82
#define B3M_CONTROL_KI2					0x86
#define B3M_CONTROL_STATIC_FRICTION2	0x8A
#define B3M_CONTROL_DYNAMIC_FRICTION2	0x8C

#define B3M_STATUS_BASE_ADDR			0x9D
#define B3M_STATUS_SYSTEM				0x9E
#define B3M_STATUS_MOTOR				0x9F
#define B3M_STATUS_UART					0xA0
#define B3M_STATUS_COMMAND				0xA1

#define B3M_CONFIG_MODEL_NUMBER			0xA2
#define B3M_CONFIG_MODEL_NUMBER_VOLTAGE_CLA	0xA2
#define B3M_CONFIG_MODEL_NUMBER_VERSION	0xA3
#define B3M_CONFIG_MODEL_NUMBER_TORQUE	0xA4
#define B3M_CONFIG_MODEL_NUMBER_CASE	0xA5
#define B3M_CONFIG_MODEL_TYPE			0xA6
#define B3M_CONFIG_MODEL_TYPE_MOTOR		0xA8
#define B3M_CONFIG_MODEL_TYPE_DEVICE	0xA9
#define B3M_CONFIG_FW_VERSION			0xAA
#define B3M_CONFIG_FW_BUID				0xAA
#define B3M_CONFIG_FW_REVISION			0xAB
#define B3M_CONFIG_FW_MINOR				0xAC
#define B3M_CONFIG_FW_MAJOR				0xAD
#define B3M_CONFIG_ENC_OFFSET_CENTER	0xAE
#define B3M_CONFIG_ENC_OFFSET			0xB0

#define B3M_OPTIONS_RUN_NORMAL			0x00
#define B3M_OPTIONS_RUN_FREE			0x02
#define B3M_OPTIONS_RUN_HOLD			0x03
#define B3M_OPTIONS_CONTROL_POSITION	0x00
#define B3M_OPTIONS_CONTROL_VELOCITY	0x04
#define B3M_OPTIONS_CONTROL_TORQUE		0x08
#define B3M_OPTIONS_CONTROL_FFORWARD	0x0C
#define B3M_OPTIONS_SERVO_NORMAL		0x00
#define B3M_OPTIONS_SERVO_CLONE			0x40
#define B3M_OPTIONS_SERVO_REVERSE		0x80

#define B3M_OPTIONS_TRAJECTORY_NORMAL	0x00
#define B3M_OPTIONS_TRAJECTORY_1		0x01
#define B3M_OPTIONS_TRAJECTORY_3		0x03
#define B3M_OPTIONS_TRAJECTORY_4		0x04
#define B3M_OPTIONS_TRAJECTORY_5		0x05


#define B3M_CMD_ID	0
#define B3M_CMD_GET	0
#define B3M_CMD_SET	0

#define B3M_SC_EEPROM  0
#define B3M_SC_STRETCH 1
#define B3M_SC_SPEED   2
#define B3M_SC_CURRENT 3
#define B3M_SC_TEMPERATURE 4
#define B3M_SC_READ 0
#define B3M_SC_WRITE 1



#ifndef __UCHAR__
#define __UCHAR__
typedef unsigned char UCHAR;
#endif 

#ifndef __UINT__
#define __UINT__
typedef unsigned int UINT;
#endif

// instance data for kondo library
typedef struct
{
	int fd;
	struct ftdi_context ftdic; // ftdi context
	UCHAR swap[128]; // swap space output
	char error[128]; // error messages
	UCHAR debug; // whether to print debug info
} B3MData;

// low level comms
int b3m_init(B3MData * r, const char* serial_port);
int b3m_close(B3MData * r);
int b3m_write(B3MData * r, int n);
int b3m_read(B3MData * r, int n);
int b3m_read_timeout(B3MData * r, int n, long timeout);
int b3m_purge(B3MData * r);
int b3m_trx(B3MData * r, UINT bytes_out, UINT bytes_in);
int b3m_trx_timeout(B3MData * r, UINT bytes_out, UINT bytes_in, long timeout);

// position commands
int b3m_set_angle(B3MData * r, UINT id, int pos);
int b3m_set_velocity(B3MData * r, UINT id, int vel100);
int b3m_set_trajectory_mode(B3MData * r, UINT id, int trajectory_mode);
int b3m_set_angle_period(B3MData * r, UINT id, int *deg100, int period_ms);
int b3m_set_angle_velocity(B3MData * r, UINT id, int *deg100, int velocity_deg100);
int b3m_servo_mode(B3MData * r, UINT id, UCHAR option);
int b3m_set_position(B3MData * r, UINT id, int deg100, int ms);

// servo setting commands
int b3m_get_angle(B3MData * r, UINT id, int *deg100);
int b3m_get_target_angle(B3MData * r, UINT id, int *deg100);
int b3m_get_velocity(B3MData * r, UINT id, int *deg100);
int b3m_get_pwm_duty_ratio(B3MData * r, UINT id, int *duty_ratio);

int b3m_get_stretch(B3MData * r, UINT id);
int b3m_get_current(B3MData * r, UINT id, int *current_mA);
int b3m_set_stretch(B3MData * r, UINT id, UCHAR stretch);
int b3m_set_speed(B3MData * r, UINT id, int speed);
int b3m_set_current_limit(B3MData * r, UINT id, int curlim);
int b3m_set_temperature_limit(B3MData * r, UINT id, int templim);
int b3m_set_torque(B3MData * r, UINT id, short smode);
int b3m_reset_error(B3MData * r, UINT id);

// set servo id (for use when 1 servo is connected)
int b3m_get_id(B3MData * r);
int b3m_set_id(B3MData * r, UINT id);

#endif /* B3M_H_ */
