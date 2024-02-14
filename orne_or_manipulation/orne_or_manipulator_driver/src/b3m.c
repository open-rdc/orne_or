/**
 * Kondo B3M 0.1 Library
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/serial.h>

#include "orne_or_manipulator_driver/b3m.h"

int target_deg100[256];

#define BAUDRATE 1500000

// Convenience macros
#define b3m_error(ki, err) { \
  snprintf(ki->error, 128, "ERROR: %s: %s\n", __func__, err); \
  fprintf(stderr,"%s\n", ki->error); \
  printf("%s\n", err); \
  return -1; }

/*!
 * @brief Iniitialize the B3M interface
 * 1500000 baud, 8 bits, even parity, 1 stop bit
 *
 * @return 0 if successful, < 0 otherwise
 */
int b3m_init(B3MData * r, const char* serial_port)
{
	int i;
	printf("b3m_init\n");
	assert(r);
	r->debug = 0;

	struct termios tio;

	r->fd = open(serial_port, O_RDWR | O_NOCTTY);
	if (ioctl(r->fd, TCGETS, &tio)){
		b3m_error(r, "Get serial port parameters");
	}

	tio.c_cflag &= ~CBAUD;          // clear mask for setting baud rate
    tio.c_cflag &= ~PARENB;         // set no parity
    tio.c_cflag &= ~CSTOPB;         // 1 stop bit
    tio.c_cflag &= ~CSIZE;          // clear mask for setting the data size
	tio.c_cflag |= B3M_BAUD;        // set B3M baud
    tio.c_cflag |= CS8;             // character size 8 bit
    tio.c_cflag |= CREAD;           // enable receiver
    tio.c_cflag |= CLOCAL;          // ignore modem status line
    tio.c_iflag = IGNBRK | IGNPAR;  // ignore break condition and characer with parity error
    tio.c_oflag = 0;                // raw mode
    tio.c_lflag = 0;                // noncanonical input
    tio.c_cc[VMIN] = 0;             // 0 return all else until n byte received
    tio.c_cc[VTIME] = 1;            // 0 block forever else until n tenth second
    tcflush(r->fd, TCIOFLUSH);      // flush current port setting

	if (ioctl(r->fd, TCSETS, &tio)){
		b3m_error(r, "Set serial port parameters");
	}

	struct serial_struct sstruct;

	if (ioctl(r->fd, TIOCGSERIAL, &sstruct) < 0) {
		printf("Error: could not get comm ioctl\n");
		exit(-1);
	}

	sstruct.custom_divisor = sstruct.baud_base / BAUDRATE;
	sstruct.flags |= ASYNC_SPD_CUST;

	if (ioctl(r->fd, TIOCSSERIAL, &sstruct) < 0) {
		printf("Error: could not set custom comm baud divisor\n");
		exit(-1);
	}
	for(i = 0; i < 256; i ++) target_deg100[i] = 100000;

	return 0;
}

/*!
 * @brief Close / Deinitialize the B3M Interface
 * 
 * @return 0 if successful, < 0 otherwise
 */
int b3m_close(B3MData * r)
{
	printf("b3m_close\n");
	assert(r);

	close(r->fd);

	return 0;
}

/*!
 * @brief Write n bytes from the swap to the Kondo
 * 
 * @return >0 number of bytes written, < 0 if error
 */
int b3m_write(B3MData * r, int n)
{
	assert(r);

	int i;
	if ((i = write(r->fd, r->swap, n)) < 0)
		b3m_error(r, "Send data");

	return i;
}

/*!
 * @brief Read n bytes from B3M. Reads immediately from the serial buffer
 * See b3m_read_timeout for a version that blocks waiting for the data.
 *
 * @return < 0: error, >= 0: number of bytes read
 */
int b3m_read(B3MData * r, int n)
{
	assert(r);
	
	int i;
	if ((i = read(r->fd, r->swap, n)) < 0)
		b3m_error(r, "Read data");
	
	return i;
}

/*!
 * @brief Read n bytes from the B3M, waiting for at most usecs for the n bytes
 * Performs this by continuously polling the serial buffer until either
 * all of the bytes are read or the timeout has been reached.
 *
 * @return < 0: error, >= 0: number of bytes read
 */
int b3m_read_timeout(B3MData * r, int n, long usecs)
{
	assert(r);

	static struct timeval tv, end;
	int i = 0, bytes_read = 0;
	gettimeofday(&tv, NULL);
	
	// determine end time
	end.tv_sec = tv.tv_sec + usecs / 1000000;
	end.tv_usec = tv.tv_usec + usecs % 1000000;
	if (end.tv_usec > 1000000) {
		end.tv_sec += 1;
		end.tv_usec -= 1000000;
	}
	// spam the read until data arrives
	do {
	    if ((i = read(r->fd, &(r->swap[bytes_read]), n - bytes_read)) < 0) {
	    //if ((i = read(r->fd, &(r->swap[bytes_read]), sizeof(&(r->swap[bytes_read])))) < 0) {
			b3m_error(r, "Read data");
	    }
	    bytes_read += i;
	    gettimeofday(&tv, NULL);
	} while (bytes_read < n && (tv.tv_sec <= end.tv_sec || tv.tv_usec < end.tv_usec));
	return bytes_read;
}

/*!
 * @brief Purge the serial buffers
 *
 * @return 0 if successful, < 0 if error
 */
int b3m_purge(B3MData * r)
{
	assert(r);
	tcflush(r->fd, TCIOFLUSH);

	return 0;
}

/*!
 * @brief Transaction template: Purge, then send out_bytes, then receive in_bytes
 * Transaction template: Purge, then send out_bytes, then receive in_bytes
 * Blocks for B3M_RX_TIMEOUT microseconds (default value)
 *
 * @return < 0: error, >= 0: number of bytes read
 */
int b3m_trx(B3MData * r, UINT bytes_out, UINT bytes_in)
{
	return b3m_trx_timeout(r, bytes_out, bytes_in, B3M_RX_TIMEOUT);
}

/*!
 * @brief Transaction template: Purge, then send out_bytes, then receive in_bytes
 * On RX, blocks for at most timeout microseconds before giving up.
 *
 * @return < 0: error, >= 0: number of bytes read
 */
int b3m_trx_timeout(B3MData * r, UINT bytes_out, UINT bytes_in, long timeout)
{
	assert(r);
	int i, j;

	if ((i = b3m_purge(r)) < 0)
		return i;

	if ((i = b3m_write(r, bytes_out)) < 0)
		return i;

	// debug printing
	if (r->debug) {
		printf("send %d bytes: ", bytes_out);
		for (j = 0; j < bytes_out; j++)
			printf("%x ", r->swap[j]);
		printf("\n");
	}

	// clear swap
	for (i = 0; i < bytes_in; i++)
		r->swap[i] = 0;

	// read the return data
	i = b3m_read_timeout(r, bytes_in, timeout);

	// debug printing
	if (r->debug) {
		printf("recv %d bytes: ", i);
		for (j = 0; j < i; j++)
			printf("%x ", r->swap[j]);
		printf("\n");
	}

	return i;
}


/*!
 * @brief send command to servo motors
 *
 * @param[in] id servo id, 0-255 (255: broadcast)
 * @param[in] address servo address
 * @param[in] data servo data
 * @param[in] byte byte of data
 * @return error status
 */
int b3m_com_send(B3MData * r, UINT id, UINT address, UCHAR *data, int byte)
{
	assert(r);

	int i, n = 0;
	int sum = 0, time = 0;

	// build command
	r->swap[n++] = 7 + byte;				// length
	r->swap[n++] = B3M_CMD_WRITE;			// command
	r->swap[n++] = B3M_RETURN_ERROR_STATUS;	// option
	r->swap[n++] = id;						// id
	for(i = 0; i < byte; i ++){
		r->swap[n++] = data[i];
	}
	r->swap[n++] = address;
	r->swap[n++] = 0x01;					// number of ID
	for(i = 0; i < n; i ++){
		sum += r->swap[i];
	}
	r->swap[n] = sum & 0xff;

	// synchronize with servo
	if ((i = b3m_trx_timeout(r, 7 + byte, 5, B3M_POS_TIMEOUT)) < 0)
		return i;

	// return error status
	return r->swap[2];
}

/*!
 * @brief servo torque on or off
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] 0 for off, 1 for on
 * @return error status.
 */
int b3m_set_torque(B3MData * r, UINT id, short smode)
{
	assert(r);

	short mode = (smode == 0) ? B3M_OPTIONS_RUN_FREE : B3M_OPTIONS_RUN_NORMAL;
	UCHAR data[1];
	data[0] = mode;

	return b3m_com_send(r, id, B3M_SERVO_TORQUE_ON, data, 1);
}

/*!
 * @brief get status from servo motors
 *
 * @param[in] id servo id, 0-255 (255: broadcast)
 * @param[in] address servo address
 * @param[out] data servo data
 * @param[in] byte byte of data
 * @return error status
 */
int b3m_get_status(B3MData * r, UINT id, UINT address, UCHAR *data, int byte)
{
	assert(r);

	int i, n = 0;
	int sum = 0, time = 0;

	// build command
	r->swap[n++] = 0x07;					// length
	r->swap[n++] = B3M_CMD_READ;			// command
	r->swap[n++] = B3M_RETURN_ERROR_STATUS;	// option
	r->swap[n++] = id;						// id
	r->swap[n++] = address;
	r->swap[n++] = byte;					// number of ID
	for(i = 0; i < n; i ++){
		sum += r->swap[i];
	}
	r->swap[n] = sum & 0xff;

	// synchronize with servo
	if ((i = b3m_trx_timeout(r, 7, 5 + byte, B3M_POS_TIMEOUT)) < 0){
		return i;
	}

	for(i = 0; i < byte; i ++){
		data[i] = r->swap[i + 4];
	}

	// return error status
	return r->swap[2];
}

/*!
 * @brief reset servo motors
 *
 * @param[in] id servo id, 0-255 (255: broadcast)
 */
int b3m_reset_error(B3MData * r, UINT id)
{
	assert(r);
	int i, n = 0;
	int sum = 0, time = 0;

	// build command
	r->swap[n++] = 6;                       // length
	r->swap[n++] = B3M_CMD_RESET;           // command
	r->swap[n++] = B3M_RETURN_ERROR_STATUS; // option
	r->swap[n++] = id;                      // id
	r->swap[n++] = time;                    //time
	for(i = 0; i < n; i ++){
		sum += r->swap[i];
	}
	r->swap[n] = sum & 0xff;

	// synchronize with servo
	if ((i = b3m_trx_timeout(r, 6, 5, B3M_POS_TIMEOUT)) < 0)
		return i;

	// return error status
	return r->swap[2];
}

/*!
 * @brief set servo position
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] pos the position to set (angle * 100)
 * @param[in] time it take to pos (ms) max speed for normal mode
 * @return error status.
 */
int b3m_set_position(B3MData * r, UINT id, int deg100, int ms)
{
	assert(r);

	int i, n = 0;
	int sum = 0, time = 0;

	// build command
	r->swap[n++] = 9;                      // length
	r->swap[n++] = B3M_CMD_POSITION;       // command
	r->swap[n++] = B3M_RETURN_ERROR_STATUS;// option
	r->swap[n++] = id;                     // id
	r->swap[n++] = deg100 & 0xff;          // angle
	r->swap[n++] = deg100 >> 8;
	r->swap[n++] = ms & 0xff;              // time
	r->swap[n++] = ms >> 8; 
	for(i = 0; i < n; i ++){
		sum += r->swap[i];
	}
	r->swap[n] = sum & 0xff;

	// synchronize with servo
	if ((i = b3m_trx_timeout(r, 9, 5, B3M_POS_TIMEOUT)) < 0)
		return i;

	// return error status
	return r->swap[2];
}

/*!
 * @brief set servo position
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] pos the position to set (angle * 100)
 * @return error status.
 */
int b3m_set_angle(B3MData * r, UINT id, int deg100)
{
	assert(r);

	UCHAR data[2];
	data[0] = deg100 & 0xff;
	data[1] = deg100 >> 8;

	return b3m_com_send(r, id, B3M_SERVO_DESIRED_POSITION, data, 2);
}


/*!
 * @brief set control angle and period
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in,out] deg100 angle (deg * 100)
 * @param[in] period_ms period (ms)
 * @return error status.
 */
int b3m_set_angle_period(B3MData * r, UINT id, int *deg100, int period_ms)
{
	if (*deg100 == target_deg100[id]) return 0;

	assert(r);

	int i, n = 0;
	int sum = 0, time = 0;

	// build command
	r->swap[n++] = 0x09;					// length
	r->swap[n++] = B3M_CMD_POSITION;		// command
	r->swap[n++] = B3M_RETURN_ERROR_STATUS;	// option
	r->swap[n++] = id;						// id
	r->swap[n++] = *deg100 & 0xff;			// lower byte of the angle data
	r->swap[n++] = *deg100 >> 8;			// higher byte of the angle data
	r->swap[n++] = period_ms & 0xff;		// lower byte of the angle data
	r->swap[n++] = period_ms >> 8;			// higher byte of the angle data
	for(i = 0; i < n; i ++){
		sum += r->swap[i];
	}
	r->swap[n] = sum & 0xff;

	// synchronize with servo
	if ((i = b3m_trx_timeout(r, 9, 7, B3M_POS_TIMEOUT)) < 0)
		return i;

	target_deg100[id] = *deg100;					// save previous desired angle for privending interference
	*deg100 = (int)((short)((r->swap[5] << 8) + r->swap[4]));

	printf("b3m_set_angle_period %d: %f, %f\n", id, (float)target_deg100[id] / 100.0f, (float)*deg100 / 100.0f);
	
	// return error status
	return r->swap[2];
}


/*!
 * @brief set control angle and velocity
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in,out] deg100 angle (deg * 100)
 * @param[in] velocity_deg100 angular velocity (deg/sec * 100)
 * @return error status.
 */
int b3m_set_angle_velocity(B3MData * r, UINT id, int *deg100, int velocity_deg100)
{
	int current_deg100;
	if (*deg100 == target_deg100[id]) return 0;
	
	if (b3m_get_angle(r, id, &current_deg100)){
		b3m_error(r, "get angle");
	}
	
	int period_ms = abs(*deg100 - current_deg100) * 1000 / velocity_deg100;
	return b3m_set_angle_period(r, id, deg100, period_ms);
}

/*!
 * @brief set servo velocity
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] velocity to set (vel * 100)
 * @return error status.
 */
int b3m_set_velocity(B3MData * r, UINT id, int vel100)
{
	assert(r);

	UCHAR data[2];
	data[0] = vel100 & 0xff;
	data[1] = vel100 >> 8;

	return b3m_com_send(r, id, B3M_SERVO_DESIRED_VELOSITY, data, 2);
}

/*!
 * @brief set trajectory mode
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] trajectory_mode trajectory mode
 * @return error status.
 */
int b3m_set_trajectory_mode(B3MData * r, UINT id, int trajectory_mode)
{
	assert(r);

	UCHAR data[1];
	data[0] = trajectory_mode;

	return b3m_com_send(r, id, B3M_SERVO_RUN_MODE, data, 1);
}


/*!
 * @brief Get servo position
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[out] deg100 the angle * 100
 * @return error status.
 */
int b3m_get_angle(B3MData * r, UINT id, int *deg100)
{
	assert(r);

	UCHAR data[2];
	if (b3m_get_status(r, id, B3M_SERVO_CURRENT_POSITION, data, 2)){
		b3m_error(r, "get status");
	}

	*deg100 = (int)((short)((data[1] << 8) + data[0]));
	return 0;
}

/*!
 * @brief get target servo position
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[out] deg100 the angle * 100
 * @return error status
 */
int b3m_get_target_angle(B3MData * r, UINT id, int *deg100){
	assert(r);

	UCHAR data[2];
	if (b3m_get_status(r, id, B3M_SERVO_DESIRED_POSITION, data, 2)){
		b3m_error(r, "get status");
	}
	
	*deg100 = (int)((short)((data[1] << 8) + data[0]));
	return 0;
}

/*!
 * @brief Get servo velocity
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[out] deg100 the angle/sec * 100
 * @return error status.
 */
int b3m_get_velocity(B3MData * r, UINT id, int *deg100)
{
	assert(r);

	int err;
	UCHAR data[2];
	if (err = b3m_get_status(r, id, B3M_SERVO_CURRENT_VELOSITY, data, 2)){
		b3m_error(r, "get status");
	}

	*deg100 = (int)((short)((data[1] << 8) + data[0]));

	return err;
}


/*!
 * @brief set servo mode
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] option B3M_OPTIONS_*
 * @return error status
 */
int b3m_servo_mode(B3MData * r, UINT id, UCHAR option)
{
	assert(r);

	UCHAR data[2];
	data[0] = option;						// mode
	data[1] = 0x00;							// interpolation

	return b3m_com_send(r, id, B3M_SERVO_SERVO_MODE, data, 2);
}


/*!
 * @brief get servo current
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[out] current_mA
 * @return error status.
 */
int b3m_get_current(B3MData * r, UINT id, int *current_mA)
{
	assert(r);

	int err;
	UCHAR data[2];
	if (err = b3m_get_status(r, id, B3M_SERVO_CURRENT, data, 2)){
		b3m_error(r, "get status\n");
	}

	*current_mA = (int)((short)((data[1] << 8) + data[0]));

	return err;
}


/*!
 * @brief get pwm duty ratio
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[out] duty_ratio duty ratio (unit?)
 * @return error status.
 */
int b3m_get_pwm_duty_ratio(B3MData * r, UINT id, int *duty_ratio)
{
	assert(r);

	int err;
	UCHAR data[2];
	if (err = b3m_get_status(r, id, B3M_SERVO_PWM_DUTY, data, 2)){
		b3m_error(r, "get status\n");
	}

	*duty_ratio = (int)((short)((data[1] << 8) + data[0]));

	return err;
}


/*-----------------------------------------------------------------------------
 * Set stretch parameter of the servo
 * id: the id of the servo 0-31
 * stretch: the desired stretch 1(2)-127(254)
 * Returns: the stretch as reported by the servo (>= 0), or < 0 if error
 */
int b3m_set_stretch(B3MData * r, UINT id, UCHAR stretch)
{
	return 0;
}

/*!
 * @brief Set speed parameter of the servo
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[out] current_mA
 * @return error status.
 */
int b3m_set_speed(B3MData * r, UINT id, int deg100)
{
	assert(r);

	UCHAR data[2];
	data[0] = deg100 & 0xff;
	data[1] = deg100 >> 8;

	return b3m_com_send(r, id, B3M_SERVO_DESIRED_VELOSITY, data, 2);
}


/*!
 * @brief Set current limit parameter of the servo
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[out] current_mA limit of current (mA)
 * @return error status.
 */
int b3m_set_current_limit(B3MData * r, UINT id, int curlim_mA)
{
	assert(r);

	UCHAR data[2];
	data[0] = curlim_mA & 0xff;
	data[1] = curlim_mA >> 8;

	return b3m_com_send(r, id, B3M_SYSTEM_CURRENT_LIMIT, data, 2);	
}

/*!
 * @brief Set temperature limit parameter of the servo
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[out] templim limit of temparature (deg * 100)
 * @return error status.
 */
int b3m_set_temperature_limit(B3MData * r, UINT id, int templim)
{
	assert(r);

	UCHAR data[2];
	data[0] = templim & 0xff;
	data[1] = templim >> 8;

	return b3m_com_send(r, id, B3M_SYSTEM_MOTOR_TEMP_LIMIT, data, 2);	
}

/*-----------------------------------------------------------------------------
 * Get the ID of the connected servo
 * This command should be used with only one servo attached to the bus.
 * Returns: ID (0-31), or < 0 if error
 */
int b3m_get_id(B3MData * r)
{
	assert(r);
	int i;

	// build command
	r->swap[0] = 0xFF; // command (0xFF for read)
	r->swap[1] = B3M_SC_READ; // subcommand (read)
	r->swap[2] = B3M_SC_READ; // subcommand (read)
	r->swap[3] = B3M_SC_READ; // subcommand (read)

	// synchronize
	if ((i = b3m_trx_timeout(r, 4, 5, B3M_ID_TIMEOUT)) < 0)
		return i;

	// return the ID
	return r->swap[4] & 0x1F;
}

/*-----------------------------------------------------------------------------
 * Set the ID of the connected servo
 * This command should be used with only one servo attached to the bus.
 * id: the desired ID of the servo 0-31
 * Returns: ID (0-31), or < 0 if error.
 */
int b3m_set_id(B3MData * r, UINT id)
{
	assert(r);
	int i;

	// check id
	if (id > 31)
	    b3m_error(r, "Invalid servo ID, > 31.");

	// build command
	r->swap[0] = id | B3M_CMD_ID;
	r->swap[1] = B3M_SC_WRITE;
	r->swap[2] = B3M_SC_WRITE;
	r->swap[3] = B3M_SC_WRITE;
	if (r->debug) {
	    int i;
	    for (i=0; i<4; i++) {
		printf("%x ", r->swap[i]);
	    }
	    printf("\n");
	}
	// This seems bug. set_id must wait some time before read.
#if 0	
	// synchronize
	if ((i = b3m_trx_timeout(r, 4, 5, B3M_ID_TIMEOUT)) < 0)
		return i;
#endif
	if ((i=b3m_write (r, 4)) < 0) {
	    return -1;
	}
	sleep (3);
	if ((i=b3m_read(r, 5)) < 0) {
	    return -1;
	}	    

	// return the ID
	return r->swap[4] & 0x1F;
}

int b3m_set_angles(B3MData * r, UINT *id, int *deg100, int len)
{
	assert(r);
	UCHAR size = (unsigned char)(6 + len * 3);
	int i, n = 0, sum = 0;

	// build command
	r->swap[n++] = size;					// length
	r->swap[n++] = B3M_CMD_WRITE;			// command
	r->swap[n++] = B3M_RETURN_ERROR_STATUS;	// option
	for(i = 0; i < len; i ++){
		r->swap[n++] = id[i];				// id
		r->swap[n++] = deg100[i] & 0xff;
		r->swap[n++] = deg100[i] >> 8;
	}
	r->swap[n++] = B3M_SERVO_DESIRED_POSITION;
	r->swap[n++] = len;				// number of ID
	for(i = 0; i < n; i ++){
		sum += r->swap[i];
	}
	r->swap[n] = sum & 0xff;

	return b3m_write(r, size);
}
