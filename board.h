
#ifndef __BOADS_H
#define __BOADS_H

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nordic_common.h"

#include "app_error.h"
#include "nrf_delay.h"

// log RTT
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// timer
#include "nrf_drv_timer.h"

// gpiote
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

// twi
#include "nrf_drv_twi.h"

// serial
#include "nrf_serial.h"


// eMPL
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
//#define _MPL
#ifdef _MPL
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#endif


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
/* TWI instance. */


static void gpiote_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);


/**
 * @brief Handler for timer events.
 */
void timer_ms_event_handler(nrf_timer_event_t event_type, void* p_context);
void get_ms(unsigned long *count);

ret_code_t system_init(void);


bool twi_write(unsigned char slave_addr, unsigned char reg_addr,
               unsigned char length, unsigned char const *data);
bool twi_read(unsigned char slave_addr, unsigned char reg_addr,
              unsigned char length, unsigned char *data);


void mpu6050_init(void);
void mpu6050_MPL_init(void);
void read_from_mpl(void);

static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(
     const signed char *mtx);


struct mpu6050_data
{
	bool data_in ;
	float accel[3] ;
	float gyro[3] ;
	float quat[4] ;
	float linear_accel[3] ;
	float gravity[3] ;
};
extern struct mpu6050_data mpu_data ;

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)


/** @} */

#endif
