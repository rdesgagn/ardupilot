#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>
#include <AP_MSP/msp.h>
#include <AP_AHRS/AP_AHRS_config.h>
#include <AP_GPS/AP_GPS_config.h>

#ifndef AP_COMPASS_ENABLED
#define AP_COMPASS_ENABLED 1
#endif

#ifndef AP_COMPASS_DIAGONALS_ENABLED
#define AP_COMPASS_DIAGONALS_ENABLED 1
#endif

#ifndef COMPASS_CAL_ENABLED
#define COMPASS_CAL_ENABLED AP_COMPASS_ENABLED && AP_AHRS_DCM_ENABLED
#endif

#ifndef AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
#define AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED AP_COMPASS_ENABLED && AP_GPS_ENABLED && AP_AHRS_ENABLED
#endif

#define COMPASS_MAX_SCALE_FACTOR 1.5
#define COMPASS_MIN_SCALE_FACTOR (1.0/COMPASS_MAX_SCALE_FACTOR)

// Backend support
#ifndef AP_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED AP_COMPASS_ENABLED
#endif

#ifndef AP_COMPASS_EXTERNALAHRS_ENABLED
#define AP_COMPASS_EXTERNALAHRS_ENABLED (AP_COMPASS_BACKEND_DEFAULT_ENABLED && HAL_EXTERNAL_AHRS_ENABLED)
#endif

#ifndef AP_COMPASS_MSP_ENABLED
#define AP_COMPASS_MSP_ENABLED (AP_COMPASS_BACKEND_DEFAULT_ENABLED && HAL_MSP_SENSORS_ENABLED)
#endif

#ifndef AP_COMPASS_SITL_ENABLED
#define AP_COMPASS_SITL_ENABLED (AP_COMPASS_BACKEND_DEFAULT_ENABLED && AP_SIM_ENABLED)
#endif

#ifndef AP_COMPASS_DRONECAN_ENABLED
#define AP_COMPASS_DRONECAN_ENABLED (AP_COMPASS_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS)
#endif

#ifndef AP_COMPASS_DRONECAN_HIRES_ENABLED
#define AP_COMPASS_DRONECAN_HIRES_ENABLED 0
#endif

// i2c-based compasses:
#ifndef AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED AP_COMPASS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_AK09916_ENABLED
#define AP_COMPASS_AK09916_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_AK8963_ENABLED
#define AP_COMPASS_AK8963_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_BMM150_ENABLED
#define AP_COMPASS_BMM150_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

// this define dictates whether we iterate over the external i2c
// busses looking for BMM150.  Ordinarily this should be true, but
// SkyViper specifies its BMM150 and thus does not need to probe.
// This is an interim arrangement until PCNC1 is eliminated as an
// auto-detect board.
#ifndef AP_COMPASS_BMM150_DETECT_BACKENDS_ENABLED
#define AP_COMPASS_BMM150_DETECT_BACKENDS_ENABLED AP_COMPASS_BMM150_ENABLED
#endif

#ifndef AP_COMPASS_BMM350_ENABLED
#define AP_COMPASS_BMM350_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_HMC5843_ENABLED
#define AP_COMPASS_HMC5843_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_ICM20948_ENABLED
#define AP_COMPASS_ICM20948_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_IST8308_ENABLED
#define AP_COMPASS_IST8308_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_IST8310_ENABLED
#define AP_COMPASS_IST8310_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_LIS3MDL_ENABLED
#define AP_COMPASS_LIS3MDL_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_LSM303D_ENABLED
#define AP_COMPASS_LSM303D_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_LSM9DS1_ENABLED
#define AP_COMPASS_LSM9DS1_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_MAG3110_ENABLED
#define AP_COMPASS_MAG3110_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_MMC3416_ENABLED
#define AP_COMPASS_MMC3416_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_MMC5XX3_ENABLED
#define AP_COMPASS_MMC5XX3_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_QMC5883P_ENABLED
#define AP_COMPASS_QMC5883P_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

#ifndef AP_COMPASS_QMC5883L_ENABLED
#define AP_COMPASS_QMC5883L_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_COMPASS_RM3100_ENABLED
#define AP_COMPASS_RM3100_ENABLED AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#endif
