#pragma once

#define IMU_RATE_HZ             100
#define IMU_BETA_RUN            0.05f
#define IMU_BETA_ALIGN          2.0f
#define IMU_ALIGN_MS            1500
#define GYRO_CAL_MS             2000
#define GYRO_CAL_TRIES         5
#define GYRO_STILL_STD_RADS    0.08f
#define GB_WINDOW_S             0.5f
#define GB_STILL_STD_RADS       0.02f
#define GB_STILL_ACC_STD_G      0.01f
#define GB_MAX_BIAS_RADS        0.25f
#define GB_STILL_MS             1500
#define GB_TAU_S                2.0f
#define MAG_CAL_MS              20000
#define MAG_DISTURB_FRAC        0.20f
#define MAG_KF_Q                1e-3f
#define MAG_KF_R                4e-4f
#define MAG_KF_GATE             4.0f
#define MAG_KF_MAX_REJECTS      25
#define ACC_VALID_MIN_G         0.7f
#define ACC_VALID_MAX_G         1.3f
#define UPSIDE_DOWN_RECAL_G     (-0.7f)

#define IMU_AXIS_IDX            {0, 1, 2}
#define IMU_AXIS_SIGN           {1, -1, -1}
#define MAG_AXIS_IDX            {0, 1, 2}
#define MAG_AXIS_SIGN           {1, 1, 1}
#define SCREEN_UP_OFFSET_DEG    0.0f

#define STEP_LENGTH_M           0.7f
#define STEP_THRESHOLD_G        0.05f
#define STEP_MIN_GAP_MS         250

#define UWB_PERIOD_MS           100
#define UWB_SPI_MODE        2
#define UWB_SPI_SLOW_HZ     500000
#define UWB_SPI_FAST_HZ     4000000
#define UWB_CLOCK_CORRECT       1
#define UWB_ANT_DELAY           16436
#define RANGE_BIAS_M            (-0.3f)
#define RANGE_MAX_JUMP_M        1.5f
#define RANGE_EMA_ALPHA         0.5f
#define RANGE_MAX_M             30.0f
#define RANGE_LOST_MS           3000
#define RANGE_DIRECT_UNDER_M    3.0f
#define RANGE_TREND_WINDOW_MS   1500
#define RANGE_TREND_EPS_M       0.15f

#define PF_SIGMA_BASE_M         0.6f
#define PF_SIGMA_WALK_COEF      0.07f
#define PF_SIGMA_MAX_M          1.5f
#define PF_WINDOW_M             15.0f
#define PF_RESEED_SPREAD_M      1.5f
#define PF_MIN_MOVE_M           0.4f
#define PF_MAX_IDLE_MS          3000
#define PF_SIGMA_NOMAG_FACTOR   1.5f
#define PF_SPREAD_DRIFT_COEF    0.05f
#define PF_STALE_ERR_M          2.5f
#define PF_STALE_MS             4000

#define NAV_CLOSE_ENTER_M       2.0f
#define NAV_CLOSE_EXIT_M        2.4f
#define NAV_ARROW_MIN_M         1.0f
#define NAV_ARROW_HYST_M        0.2f
#define NAV_TRACK_ENTER_SPREAD  1.5f
#define NAV_TRACK_MIN_WALK_M    2.0f
#define NAV_SEARCH_ENTER_SPREAD 2.0f
#define NAV_ARROW_ALPHA         0.25f
#define NAV_TWO_HOLD_MS         2000
#define NAV_NEAR_M              0.4f
#define NAV_FAR_M               12.0f

#define UI_FPS                  15
#define LOG_CSV                 1
#define LOG_AXES                1
#define LOG_NAV                 1
#define LOG_UWB_RAW             1
