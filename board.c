



#include "board.h"

//#define _DMP

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

const nrf_drv_timer_t TIMER_MS = NRF_DRV_TIMER_INSTANCE(0);

struct mpu6050_data mpu_data = {0};

static void gpiote_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_delay_us(10);
    if ( pin == 13 )
    {
        nrf_gpio_pin_toggle(16);
//              NRF_LOG_INFO("gyro data in\r\n");
//              NRF_LOG_FLUSH();

        unsigned long sensor_timestamp;
        short gyro[3], accel[3] ={0};
        long l_accel[3];
        unsigned char more=0;
		short sensors = INV_XYZ_GYRO|INV_XYZ_ACCEL ;
        long quat[4], temperature;

        mpu_data.data_in = true ;
#ifdef _DMP
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
        inv_build_quat(quat, 0, sensor_timestamp);
#else
//        mpu_get_accel_reg(accel, &sensor_timestamp);
//        mpu_get_gyro_reg(gyro, &sensor_timestamp);
//		mpu_read_fifo( gyro, accel, &sensor_timestamp, &sensors, &more);
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
#endif
        if ( more )
        {
            NRF_LOG_INFO("more = %d\r\n", more);
            NRF_LOG_FLUSH();
        }

        for ( uint8_t i = 0 ; i < 3 ; i++ )
        {
            mpu_data.accel[i] = accel[i] / 163.84 ;
            mpu_data.gyro[i] = gyro[i] / 16.4 ;

            l_accel[i] = (long)accel[i];
        }

#ifdef _MPL
        inv_build_gyro(gyro, sensor_timestamp);
        inv_build_accel(l_accel, 0, sensor_timestamp);

//              mpu_get_temperature(&temperature, &sensor_timestamp);
//              inv_build_temp(temperature, sensor_timestamp);

        if ( INV_SUCCESS != inv_execute_on_data() )
        {
            NRF_LOG_INFO("Execute data error.\n");
            NRF_LOG_FLUSH();
        }
#endif
    }
}

/**
 * @brief Handler for timer events.
 */
volatile uint32_t RTC_MS = 0 ;
void timer_ms_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
    case NRF_TIMER_EVENT_COMPARE0:
        RTC_MS ++ ;
//                      NRF_LOG_INFO("RTC_MS = %ld\r\n",RTC_MS);
//                      NRF_LOG_FLUSH();
        break;

    default:
        //Do nothing.
        break;
    }
}


// serial
NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                               25, 26,
                               NULL, NULL,
                               NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                               NRF_UART_BAUDRATE_1000000,
                               UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 2048
#define SERIAL_FIFO_RX_SIZE 32

NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 255
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, NULL, NULL);


NRF_SERIAL_UART_DEF(serial_uart, 0);
///


ret_code_t system_init(void)
{
    ret_code_t err_code;
    uint8_t buff[100] = {0} ;

    // RTT init
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\n project start\r\n");
    NRF_LOG_FLUSH();

    // gpio
    nrf_gpio_cfg_output( 16 );
    nrf_gpio_pin_clear( 16 );



    // gpiote
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_config_t gpiote_in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    gpiote_in_config.pull = NRF_GPIO_PIN_PULLUP ;
    err_code = nrf_drv_gpiote_in_init( 13 , &gpiote_in_config , gpiote_handler  );// 17 button , 28 mpu6050 INT pin
    APP_ERROR_CHECK(err_code);

//        nrf_drv_gpiote_in_event_enable( 28 , true );

    // twi
    nrf_drv_twi_config_t twi_config = {
        .scl                = 22,
        .sda                = 23,
        .frequency          = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

//      err_code = nrf_drv_twi_tx(&m_twi, 0x68, buff, 1 , false);
//    APP_ERROR_CHECK(err_code);


    // timer
    uint32_t time_ticks;

    //Configure TIMER for generating simple light effect .
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_MS, &timer_cfg, timer_ms_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_MS, 1000);

    nrf_drv_timer_extended_compare(
        &TIMER_MS, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_MS);


    // serial
    err_code = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
    APP_ERROR_CHECK(err_code);

    sprintf( (char *)buff , "Hello nrf_serial!\n\r" );
    err_code = nrf_serial_write(&serial_uart,
                                buff,
                                strlen((char *)buff),
                                NULL,
                                NRF_SERIAL_MAX_TIMEOUT);




    return err_code ;
}


uint8_t data_buff[256] = {0} ;
bool twi_write(unsigned char slave_addr, unsigned char reg_addr,
               unsigned char length, unsigned char const *data)
{
    ret_code_t err_code ;

    data_buff[0] = reg_addr ;
    for ( uint8_t i = 0 ; i < length ; i++ )
        data_buff[i + 1] = data[i] ;

    err_code = nrf_drv_twi_tx(&m_twi, slave_addr, &data_buff[0], length + 1, 0);

    APP_ERROR_CHECK(err_code);
    return err_code ;
}

bool twi_read(unsigned char slave_addr, unsigned char reg_addr,
              unsigned char length, unsigned char *data)
{
    ret_code_t err_code ;

    err_code = nrf_drv_twi_tx(&m_twi, slave_addr, &reg_addr, 1, 0);


    err_code = nrf_drv_twi_rx(&m_twi, slave_addr, data, length);
    APP_ERROR_CHECK(err_code);
    return err_code ;
}


void get_ms(unsigned long *count)
{
    *count = RTC_MS ;
}

///////////////////////////////////////////////

static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1
                                         };
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
#ifdef _MPL
    unsigned char lp_accel_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
#endif
};
static struct hal_s hal = {0};

#ifdef _MPL
/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};
static struct platform_data_s gyro_pdata = {
    .orientation = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    }
};
#endif

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";


#ifdef _DMP
#define F_SAMPLE 200
#else
#define F_SAMPLE 500
#endif
void mpu6050_init(void)
{
    ret_code_t err_code ;
    struct int_param_s int_param;

    /* Set up gyro.
    * Every function preceded by mpu_ is a driver function and can be found
    * in inv_mpu.h.
    */
    int_param.cb = NULL;
    int_param.pin = 13;
    err_code = mpu_init(&int_param);
    APP_ERROR_CHECK(err_code);

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(100);
    /* Read back configuration in case it was set improperly. */
//      mpu_get_sample_rate(&gyro_rate);
//    mpu_get_gyro_fsr(&gyro_fsr);
//    mpu_get_accel_fsr(&accel_fsr);

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));

    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(100);
    mpu_set_dmp_state(1);
}

#ifdef _MPL
void mpu6050_MPL_init(void)
{
    ret_code_t err_code ;
    struct int_param_s int_param;
    inv_error_t result;
    unsigned char accel_fsr,  new_temp = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;


    /* Set up gyro.
    * Every function preceded by mpu_ is a driver function and can be found
    * in inv_mpu.h.
    */
    int_param.cb = NULL;
    int_param.pin = 28;
    err_code = mpu_init(&int_param);
    APP_ERROR_CHECK(err_code);

    //////// init MPL
    result = inv_init_mpl();
    if (result) {
        NRF_LOG_INFO("Could not initialize MPL.\n");
        NRF_LOG_FLUSH();
    }

    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();

    /* Update gyro biases when not in motion.
    * WARNING: These algorithms are mutually exclusive.
    */
    inv_enable_fast_nomot();
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
            NRF_LOG_INFO("Not authorized.\n");
            NRF_LOG_FLUSH();
        }
    }
    if (result) {
        NRF_LOG_INFO("Could not start the MPL.\n");
        NRF_LOG_FLUSH();
    }
    /////////////////////////////////////

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(F_SAMPLE);

#ifndef _DMP
    mpu_set_accel_fsr(16);
    mpu_set_gyro_fsr(2000);
#endif

    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    /////////////
    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);


    /* Set chip-to-body orientation matrix.
    * Set hardware units to dps/g's/degrees scaling factor.
    */
    inv_set_gyro_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)gyro_fsr << 15);
    inv_set_accel_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)accel_fsr << 15);


    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;
    /////////////
#ifdef _DMP
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));
//      dmp_register_tap_cb(tap_cb);
//    dmp_register_android_orient_cb(android_orient_cb);

    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(F_SAMPLE);
    mpu_set_dmp_state(1);
#endif

}
#endif

static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

#ifdef _MPL
void read_from_mpl(void)
{
    long msg, data[9];
    int8_t accuracy = 0;
    unsigned long timestamp;
    float float_data[3] = {0};
    long long_data[3] = {0} ;
    uint8_t buff[100] = {0} ;


    static uint8_t counter = 0 ;

    counter++ ;

    if ( counter >= 1 )
    {
//          inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp);
//          sprintf( (char *)buff , "Linear Accel: %3.2f %3.2f %3.2f\r\n",float_data[0], float_data[1], float_data[2]);

//          inv_get_sensor_type_gravity(float_data, &accuracy,(inv_time_t*)&timestamp);
//          sprintf( (char *)buff , "Gravity Vector: %3.2f %3.2f %3.2f\r\n", float_data[0], float_data[1], float_data[2]);

        inv_get_sensor_type_euler(long_data, &accuracy, (inv_time_t*)&timestamp);
        sprintf( (char *)buff , "angle : %03.2f %03.2f %03.2f\r\n", long_data[0] / 65536.f, long_data[1] / 65536.f, long_data[2] / 65536.f);

//                  NRF_LOG_INFO("%s",buff);
//                  NRF_LOG_FLUSH();

        nrf_serial_write(&serial_uart, buff,
                         strlen((char *)buff),
                         NULL, NRF_SERIAL_MAX_TIMEOUT);

        counter = 0 ;
    }


}
#endif


/** @} */
