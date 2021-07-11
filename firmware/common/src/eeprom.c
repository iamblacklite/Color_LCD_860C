/*
 * Bafang LCD 850C firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stdio.h"
#include <string.h>
#include "eeprom.h"
#include "eeprom_hw.h"
#include "main.h"
#include "mainscreen.h"
//#include "lcd_configurations.h"

static eeprom_data_t m_eeprom_data;

// get rid of some copypasta with this little wrapper for copying arrays between structs
#define COPY_ARRAY(dest, src, field) memcpy((dest)->field, (src)->field, sizeof((dest)->field))

const eeprom_data_t m_eeprom_data_defaults = {
  .eeprom_version = EEPROM_VERSION,
  .ui8_bit_data_1 = DEFAULT_BIT_DATA_1,
  .ui8_bit_data_2 = DEFAULT_BIT_DATA_2,
  .ui8_bit_data_3 = DEFAULT_BIT_DATA_3,
  .ui8_riding_mode = DEFAULT_VALUE_RIDING_MODE,
  .ui8_assist_level = DEFAULT_VALUE_ASSIST_LEVEL,
  .ui16_wheel_perimeter = DEFAULT_VALUE_WHEEL_PERIMETER,
  .ui8_wheel_max_speed = DEFAULT_VALUE_WHEEL_MAX_SPEED,
  //.ui8_units_type = DEFAULT_VALUE_UNITS_TYPE,
  .ui32_wh_x10_offset = DEFAULT_VALUE_WH_X10_OFFSET,
  .ui32_wh_x10_100_percent = DEFAULT_VALUE_HW_X10_100_PERCENT,
  .ui8_battery_soc_enable = DEAFULT_VALUE_SHOW_NUMERIC_BATTERY_SOC,
  .ui8_battery_max_current = DEFAULT_VALUE_BATTERY_MAX_CURRENT,
  .ui8_target_max_battery_power_div25 = DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER,
  .ui8_motor_max_current = DEFAULT_VALUE_MOTOR_MAX_CURRENT,
  .ui8_motor_current_min_adc = DEFAULT_VALUE_CURRENT_MIN_ADC,
  //.ui8_field_weakening = DEFAULT_FIELD_WEAKENING,
  //.ui8_ramp_up_amps_per_second_x10 = DEFAULT_VALUE_RAMP_UP_AMPS_PER_SECOND_X10,
  .ui16_battery_low_voltage_cut_off_x10 = DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10,
  //.ui8_motor_type = DEFAULT_VALUE_MOTOR_TYPE,
  //.ui8_motor_current_control_mode = DEFAULT_VALUE_MOTOR_CURRENT_CONTROL_MODE,
  //.ui8_motor_assistance_startup_without_pedal_rotation = DEFAULT_VALUE_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION,
  .ui8_assist_level_factor = {
  {
	DEFAULT_VALUE_POWER_ASSIST_LEVEL_1,
	DEFAULT_VALUE_POWER_ASSIST_LEVEL_2,
	DEFAULT_VALUE_POWER_ASSIST_LEVEL_3,
	DEFAULT_VALUE_POWER_ASSIST_LEVEL_4,
	DEFAULT_VALUE_POWER_ASSIST_LEVEL_5,
	DEFAULT_VALUE_POWER_ASSIST_LEVEL_6,
	DEFAULT_VALUE_POWER_ASSIST_LEVEL_7,
	DEFAULT_VALUE_POWER_ASSIST_LEVEL_8,
	DEFAULT_VALUE_POWER_ASSIST_LEVEL_9
  }, {
	DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_1,
	DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_2,
	DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_3,
	DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_4,
	DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_5,
	DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_6,
	DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_7,
	DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_8,
	DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_9
  }, {
	DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_1,
	DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_2,
	DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_3,
	DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_4,
	DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_5,
	DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_6,
	DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_7,
	DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_8,
	DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_9
  }, {
	DEFAULT_VALUE_EMTB_ASSIST_LEVEL_1,
	DEFAULT_VALUE_EMTB_ASSIST_LEVEL_2,
	DEFAULT_VALUE_EMTB_ASSIST_LEVEL_3,
	DEFAULT_VALUE_EMTB_ASSIST_LEVEL_4,
	DEFAULT_VALUE_EMTB_ASSIST_LEVEL_5,
	DEFAULT_VALUE_EMTB_ASSIST_LEVEL_6,
	DEFAULT_VALUE_EMTB_ASSIST_LEVEL_7,
	DEFAULT_VALUE_EMTB_ASSIST_LEVEL_8,
	DEFAULT_VALUE_EMTB_ASSIST_LEVEL_9
  } },
/*
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_1,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_2,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_3,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_4,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_5,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_6,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_7,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_8,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_9,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_10,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_11,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_12,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_13,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_14,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_15,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_16,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_17,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_18,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_19,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_20,
  }, */
  .ui8_number_of_assist_levels = DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS,
  //.ui8_startup_motor_power_boost_feature_enabled = DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED,
  //.ui8_startup_motor_power_boost_always = DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ALWAYS,
  //.ui8_startup_motor_power_boost_limit_power = DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_LIMIT_POWER, // missing
  /*.ui8_startup_motor_power_boost_factor = {
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1, // bug?
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_5,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_6,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_7,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_8,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_9,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_10,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_11,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_12,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_13,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_14,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_15,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_16,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_17,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_18,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_19,
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_20,
  }, */
  //.ui8_startup_motor_power_boost_time =
  //DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_TIME,
  //.ui8_startup_motor_power_boost_fade_time =
  //DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_FADE_TIME,
  .ui8_optional_ADC_function =
  DEFAULT_VALUE_OPTIONAL_ADC_FUNCTION,
  .ui8_motor_temperature_min_value_to_limit =
  DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT,
  .ui8_motor_temperature_max_value_to_limit =
  DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT,
  .ui16_battery_voltage_reset_wh_counter_x10 =
  DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10,
  .ui8_lcd_power_off_time_minutes =
  DEFAULT_VALUE_LCD_POWER_OFF_TIME,
  .ui8_lcd_backlight_on_brightness =
  DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS,
  .ui8_lcd_backlight_off_brightness =
  DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS,
  .ui16_battery_pack_resistance_x1000 =
  DEFAULT_VALUE_BATTERY_PACK_RESISTANCE,
  //.ui8_offroad_feature_enabled =
  //DEFAULT_VALUE_OFFROAD_FEATURE_ENABLED,
  //.ui8_offroad_enabled_on_startup =
  //DEFAULT_VALUE_OFFROAD_MODE_ENABLED_ON_STARTUP,
  //.ui8_offroad_speed_limit = DEFAULT_VALUE_OFFROAD_SPEED_LIMIT,
  //.ui8_offroad_power_limit_enabled =
  //DEFAULT_VALUE_OFFROAD_POWER_LIMIT_ENABLED,
  //.ui8_offroad_power_limit_div25 =
  //DEFAULT_VALUE_OFFROAD_POWER_LIMIT_DIV25,
  .ui32_odometer_x10 =
  DEFAULT_VALUE_ODOMETER_X10,
  //.ui8_walk_assist_feature_enabled =
  //DEFAULT_VALUE_WALK_ASSIST_FEATURE_ENABLED,
  .ui8_walk_assist_level_factor = {
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_1,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_2,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_3,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_4,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_5,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_6,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_7,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_8,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_9 /*
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_10,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_11,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_12,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_13,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_14,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_15,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_16,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_17,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_18,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_19,
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_20, */
  },
/*
    upTimeField, // 0
    odoField, // 1
    tripADistanceField, // 2
    tripATimeField, // 3
    tripAAvgSpeedField, // 4
    tripAMaxSpeedField, // 5
    tripBDistanceField, // 6
    tripBTimeField, // 7
    tripBMaxSpeedField, // 8
    tripBAvgSpeedField, // 9
    wheelSpeedField, // 10
    cadenceField, // 11
	humanPowerField, // 12
	batteryPowerField, // 13
    batteryVoltageField, // 14
    batteryCurrentField, // 15
    motorCurrentField, // 16
    batterySOCField, // 17
	motorTempField, // 18
    motorErpsField, // 19
	pwmDutyField, // 20
	motorFOCField, // 21
	batteryPowerUsageField, // 22
	
	wheelSpeedGraph, // 0
	tripDistanceGraph, // 1
	cadenceGraph, // 2
	humanPowerGraph, // 3
	batteryPowerGraph, // 4
	batteryPowerUsageGraph, // 5
	batteryVoltageGraph, // 6
	batteryCurrentGraph, // 7
	motorCurrentGraph, // 8
	batterySOCGraph, // 9
	motorTempGraph, // 10
	motorErpsGraph, // 11
	pwmDutyGraph, // 12
	motorFOCGraph); // 13
*/
#ifdef SW102
  .field_selectors = {
    12, // human power
    13, // motor power

    0, // up time
    2, // trip distance

    13, // motor power
    20, // PWM
  },
#else
  .field_selectors = {
    1,  // odometer
    12, // human power
    0,  // up time
    13, // motor power

    2, // trip A distance
    4, // trip A avg speed
    3, // trip A time
    5, // trip A max speed

    20, // PWM
    15, // battery current
    19, // motor speed
    13, // motor power
  },

  .graphs_field_selectors = {
    0, // wheel speed
    3, // human power
    5, // battery voltage
  },
#endif

  .showNextScreenIndex = 0,
  .x_axis_scale = DEFAULT_VALUE_X_AXIS_SCALE,
  //.ui8_buttons_up_down_invert = DEFAULT_VALUE_BUTTONS_UP_DOWN_INVERT,

#ifndef SW102
  // enable automatic graph max min for every variable
  .graph_eeprom[VarsWheelSpeed].auto_max_min = GRAPH_AUTO_MAX_MIN_MANUAL,
  .graph_eeprom[VarsWheelSpeed].max = 350, // 35 km/h
  .graph_eeprom[VarsWheelSpeed].min = 0,

  .graph_eeprom[VarsTripDistance].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsCadence].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsHumanPower].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsBatteryPower].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsBatteryPowerUsage].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsBatteryVoltage].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsBatteryCurrent].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsMotorCurrent].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsBatterySOC].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsMotorTemp].auto_max_min = GRAPH_AUTO_MAX_MIN_SEMI_AUTO,
  .graph_eeprom[VarsMotorERPS].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,
  .graph_eeprom[VarsMotorPWM].auto_max_min = GRAPH_AUTO_MAX_MIN_SEMI_AUTO,
  .graph_eeprom[VarsMotorFOC].auto_max_min = GRAPH_AUTO_MAX_MIN_AUTO,

  .tripDistanceField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,

  .wheelSpeedField_auto_thresholds = FIELD_THRESHOLD_MANUAL,
  .wheelSpeedField_config_error_threshold = 350,
  .wheelSpeedField_config_warn_threshold = 300,
  .wheelSpeedField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,

  .cadenceField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .cadenceField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
  .batteryPowerField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .batteryPowerField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
  .batteryPowerUsageField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .batteryPowerUsageField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_15M,
  .batteryVoltageField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .batteryVoltageField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
  .batteryCurrentField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .batteryCurrentField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
  .motorCurrentField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .motorCurrentField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
  .motorTempField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .motorTempField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_15M,
  .motorErpsField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .motorErpsField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
  .pwmDutyField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .pwmDutyField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
  .motorFOCField_auto_thresholds = FIELD_THRESHOLD_AUTO,
  .motorFOCField_x_axis_scale_config = GRAPH_X_AXIS_SCALE_AUTO,
#endif

  //.ui8_torque_sensor_calibration_feature_enabled = DEFAULT_TORQUE_SENSOR_CALIBRATION_FEATURE_ENABLE,
  //.ui8_torque_sensor_calibration_pedal_ground = DEFAULT_TORQUE_SENSOR_CALIBRATION_PEDAL_GROUND,
/*  .ui16_torque_sensor_calibration_table_left[0][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_WEIGHT_1,
  .ui16_torque_sensor_calibration_table_left[0][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_ADC_1,
  .ui16_torque_sensor_calibration_table_left[1][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_WEIGHT_2,
  .ui16_torque_sensor_calibration_table_left[1][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_ADC_2,
  .ui16_torque_sensor_calibration_table_left[2][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_WEIGHT_3,
  .ui16_torque_sensor_calibration_table_left[2][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_ADC_3,
  .ui16_torque_sensor_calibration_table_left[3][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_WEIGHT_4,
  .ui16_torque_sensor_calibration_table_left[3][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_ADC_4,
  .ui16_torque_sensor_calibration_table_left[4][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_WEIGHT_5,
  .ui16_torque_sensor_calibration_table_left[4][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_ADC_5,
  .ui16_torque_sensor_calibration_table_left[5][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_WEIGHT_6,
  .ui16_torque_sensor_calibration_table_left[5][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_ADC_6,
  .ui16_torque_sensor_calibration_table_left[6][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_WEIGHT_7,
  .ui16_torque_sensor_calibration_table_left[6][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_ADC_7,
  .ui16_torque_sensor_calibration_table_left[7][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_WEIGHT_8,
  .ui16_torque_sensor_calibration_table_left[7][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_LEFT_ADC_8,

  .ui16_torque_sensor_calibration_table_right[0][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_WEIGHT_1,
  .ui16_torque_sensor_calibration_table_right[0][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_ADC_1,
  .ui16_torque_sensor_calibration_table_right[1][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_WEIGHT_2,
  .ui16_torque_sensor_calibration_table_right[1][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_ADC_2,
  .ui16_torque_sensor_calibration_table_right[2][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_WEIGHT_3,
  .ui16_torque_sensor_calibration_table_right[2][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_ADC_3,
  .ui16_torque_sensor_calibration_table_right[3][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_WEIGHT_4,
  .ui16_torque_sensor_calibration_table_right[3][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_ADC_4,
  .ui16_torque_sensor_calibration_table_right[4][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_WEIGHT_5,
  .ui16_torque_sensor_calibration_table_right[4][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_ADC_5,
  .ui16_torque_sensor_calibration_table_right[5][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_WEIGHT_6,
  .ui16_torque_sensor_calibration_table_right[5][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_ADC_6,
  .ui16_torque_sensor_calibration_table_right[6][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_WEIGHT_7,
  .ui16_torque_sensor_calibration_table_right[6][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_ADC_7,
  .ui16_torque_sensor_calibration_table_right[7][0] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_WEIGHT_8,
  .ui16_torque_sensor_calibration_table_right[7][1] = DEFAULT_TORQUE_SENSOR_CALIBRATION_RIGHT_ADC_8,
*/
  //.ui8_street_mode_function_enabled = DEFAULT_STREET_MODE_FUNCTION_ENABLE,
  //.ui8_street_mode_enabled_on_startup = DEFAULT_STREET_MODE_ENABLE_AT_STARTUP,
  //.ui8_street_mode_enabled = DEFAULT_STREET_MODE_ENABLE,
  .ui8_street_mode_speed_limit = DEFAULT_STREET_MODE_SPEED_LIMIT,
  .ui8_street_mode_power_limit_div25 = DEFAULT_STREET_MODE_POWER_LIMIT,
  //.ui8_street_mode_throttle_enabled = DEFAULT_STREET_MODE_THROTTLE_ENABLE,
  //.ui8_street_mode_hotkey_enabled = DEFAULT_STREET_MODE_HOTKEY_ENABLE,
  //.ui8_pedal_cadence_fast_stop = DEFAULT_PEDAL_CADENCE_FAST_STOP_ENABLE,
  .ui8_coast_brake_adc = DEFAULT_COAST_BRAKE_ADC,
  //.ui8_adc_lights_current_offset = DEFAULT_ADC_LIGHTS_CURRENT_OFFSET,
  .ui8_throttle_virtual_step = DEFAULT_THROTTLE_VIRTUAL_STEP,
  .ui8_torque_sensor_filter = DEFAULT_TORQUE_SENSOR_FILTER,
  .ui8_torque_sensor_adc_threshold = DEFAULT_TORQUE_SENSOR_ADC_THRESHOLD,
  //.ui8_coast_brake_enable = DEFAULT_COAST_BRAKE_ENABLE,

  .ui8_motor_acceleration_adjustment = DEFAULT_VALUE_MOTOR_ACCELERATION_ADJUSTMENT,
  .ui8_motor_deceleration_adjustment = DEFAULT_VALUE_MOTOR_DECELERATION_ADJUSTMENT,
  .ui8_time_field_enable = DEAFULT_VALUE_TIME_FIELD,
  .ui8_pedal_torque_per_10_bit_ADC_step_x100 = DEFAULT_VALUE_PEDAL_TORQUE_ADC_STEP_x100,
  .ui8_lights_configuration = DEFAULT_LIGHTS_CONFIGURATION,
  .ui16_startup_boost_torque_factor = DEFAULT_VALUE_STARTUP_BOOST_TORQUE_FACTOR,
  .ui8_startup_boost_cadence_step = DEFAULT_VALUE_STARTUP_BOOST_CADENCE_STEP,
  .ui16_adc_pedal_torque_offset = DEFAULT_TORQUE_SENSOR_ADC_OFFSET,
  .ui16_adc_pedal_torque_max = DEFAULT_TORQUE_SENSOR_ADC_MAX,
  .ui8_weight_on_pedal = DEFAULT_WEIGHT_ON_PEDAL_CALIBRATION,
  .ui16_adc_pedal_torque_calibration = DEFAULT_TORQUE_SENSOR_ADC_CALIBRATION,
  .ui8_motor_acceleration_after_braking = DEFAULT_VALUE_MOTOR_ACCELERATION_AFTER_BRAKING,
  .ui8_motor_acceleration_time_after_braking = DEFAULT_VALUE_MOTOR_ACCELERATION_TIME_AFTER_BRAKING,

#ifndef SW102
  .ui8_trip_a_auto_reset = DEFAULT_VALUE_TRIP_AUTO_RESET_ENABLE,
  .ui16_trip_a_auto_reset_hours = DEFAULT_VALUE_TRIP_A_AUTO_RESET_HOURS,
#endif

  .ui32_trip_a_distance_x1000 = DEFAULT_VALUE_TRIP_DISTANCE,
  .ui32_trip_a_time = DEFAULT_VALUE_TRIP_TIME,
  .ui16_trip_a_max_speed_x10 = DEFAULT_VALUE_TRIP_MAX_SPEED,

#ifndef SW102
  .ui8_trip_b_auto_reset = DEFAULT_VALUE_TRIP_AUTO_RESET_ENABLE,
  .ui16_trip_b_auto_reset_hours = DEFAULT_VALUE_TRIP_B_AUTO_RESET_HOURS,
#endif

  .ui32_trip_b_distance_x1000 = DEFAULT_VALUE_TRIP_DISTANCE,
  .ui32_trip_b_time = DEFAULT_VALUE_TRIP_TIME,
  .ui16_trip_b_max_speed_x10 = DEFAULT_VALUE_TRIP_MAX_SPEED,

};

void eeprom_init() {
	eeprom_hw_init();

	// read the values from EEPROM to array
	memset(&m_eeprom_data, 0, sizeof(m_eeprom_data));

	// if eeprom is blank use defaults
	// if eeprom version is less than the min required version, wipe and use defaults
	// if eeprom version is greater than the current app version, user must have downgraded - wipe and use defaults
	if (!flash_read_words(&m_eeprom_data,
			sizeof(m_eeprom_data)
					/ sizeof(uint32_t))
	    || m_eeprom_data.eeprom_version < EEPROM_MIN_COMPAT_VERSION
	    || m_eeprom_data.eeprom_version > EEPROM_VERSION
	    )
		// If we are using default data it doesn't get written to flash until someone calls write
		memcpy(&m_eeprom_data, &m_eeprom_data_defaults,
				sizeof(m_eeprom_data_defaults));

//	// Perform whatever migrations we need to update old eeprom formats
//	if (m_eeprom_data.eeprom_version < EEPROM_VERSION) {
//
//		m_eeprom_data.ui8_lcd_backlight_on_brightness =
//				m_eeprom_data_defaults.ui8_lcd_backlight_on_brightness;
//		m_eeprom_data.ui8_lcd_backlight_off_brightness =
//				m_eeprom_data_defaults.ui8_lcd_backlight_off_brightness;
//
//		m_eeprom_data.eeprom_version = EEPROM_VERSION;
//	}

	eeprom_init_variables();

	set_conversions();

	// prepare torque_sensor_calibration_table as it will be used at begin to init the motor
	prepare_torque_sensor_calibration_table();
}

void eeprom_init_variables(void) {
	ui_vars_t *ui_vars = get_ui_vars();
	rt_vars_t *rt_vars = get_rt_vars();

	// copy data final variables
	ui_vars->ui8_riding_mode = m_eeprom_data.ui8_riding_mode;
	ui_vars->ui8_assist_level = m_eeprom_data.ui8_assist_level;
	ui_vars->ui16_wheel_perimeter = m_eeprom_data.ui16_wheel_perimeter;
	ui_vars->ui16_wheel_max_speed_x10 =
			m_eeprom_data.ui8_wheel_max_speed * 10;
	//ui_vars->ui8_units_type = m_eeprom_data.ui8_units_type;
	ui_vars->ui8_units_type = (m_eeprom_data.ui8_bit_data_1 & 1);
	ui_vars->ui32_wh_x10_offset = m_eeprom_data.ui32_wh_x10_offset;
	ui_vars->ui32_wh_x10_100_percent =
			m_eeprom_data.ui32_wh_x10_100_percent;
	ui_vars->ui8_battery_soc_enable =
			m_eeprom_data.ui8_battery_soc_enable;
	ui_vars->ui8_time_field_enable = m_eeprom_data.ui8_time_field_enable;
	ui_vars->ui8_pedal_cadence_fast_stop = (m_eeprom_data.ui8_bit_data_1 & 32) >> 5;
	ui_vars->ui8_target_max_battery_power_div25 =
      m_eeprom_data.ui8_target_max_battery_power_div25;
	ui_vars->ui8_battery_max_current =
			m_eeprom_data.ui8_battery_max_current;
	ui_vars->ui8_motor_max_current =
      m_eeprom_data.ui8_motor_max_current;
	ui_vars->ui8_motor_current_min_adc =
      m_eeprom_data.ui8_motor_current_min_adc;
	//ui_vars->ui8_ramp_up_amps_per_second_x10 =
	//		m_eeprom_data.ui8_ramp_up_amps_per_second_x10;
	ui_vars->ui16_battery_low_voltage_cut_off_x10 =
			m_eeprom_data.ui16_battery_low_voltage_cut_off_x10;
	ui_vars->ui8_motor_type = (m_eeprom_data.ui8_bit_data_1 & 2) >> 1;
	//ui_vars->ui8_motor_current_control_mode = (m_eeprom_data.ui8_bit_data_1 & 4) >> 2;
	ui_vars->ui8_motor_assistance_startup_without_pedal_rotation =
			(m_eeprom_data.ui8_bit_data_1 & 8) >> 3;
	ui_vars->ui8_cruise_feature_enabled =
			(m_eeprom_data.ui8_bit_data_1 & 64) >> 6;
	//COPY_ARRAY(ui_vars, &m_eeprom_data, ui16_assist_level_factor);
	
	ui_vars->ui8_number_of_assist_levels =
			m_eeprom_data.ui8_number_of_assist_levels;
	ui_vars->ui8_startup_motor_power_boost_feature_enabled =
			(m_eeprom_data.ui8_bit_data_1 & 16) >> 4;
	//ui_vars->ui8_startup_motor_power_boost_limit_power =
	//		(m_eeprom_data.ui8_bit_data_1 & 64) >> 6;
	//ui_vars->ui8_startup_motor_power_boost_always =
	//		(m_eeprom_data.ui8_bit_data_1 & 32) >> 5;
	//COPY_ARRAY(ui_vars, &m_eeprom_data,
	//		ui8_startup_motor_power_boost_factor);
	//ui_vars->ui8_startup_motor_power_boost_time =
	//		m_eeprom_data.ui8_startup_motor_power_boost_time;
	//ui_vars->ui8_startup_motor_power_boost_fade_time =
	//		m_eeprom_data.ui8_startup_motor_power_boost_fade_time;
	ui_vars->ui8_optional_ADC_function = m_eeprom_data.ui8_optional_ADC_function;
	ui_vars->ui8_motor_temperature_min_value_to_limit =
			m_eeprom_data.ui8_motor_temperature_min_value_to_limit;
	ui_vars->ui8_motor_temperature_max_value_to_limit =
			m_eeprom_data.ui8_motor_temperature_max_value_to_limit;
	ui_vars->ui16_battery_voltage_reset_wh_counter_x10 =
			m_eeprom_data.ui16_battery_voltage_reset_wh_counter_x10;
	ui_vars->ui8_lcd_power_off_time_minutes =
			m_eeprom_data.ui8_lcd_power_off_time_minutes;
	ui_vars->ui8_lcd_backlight_on_brightness =
			m_eeprom_data.ui8_lcd_backlight_on_brightness;
	ui_vars->ui8_lcd_backlight_off_brightness =
			m_eeprom_data.ui8_lcd_backlight_off_brightness;
	ui_vars->ui16_battery_pack_resistance_x1000 =
			m_eeprom_data.ui16_battery_pack_resistance_x1000;
	//ui_vars->ui8_offroad_feature_enabled =
	//		m_eeprom_data.ui8_offroad_feature_enabled;
	//ui_vars->ui8_offroad_enabled_on_startup =
	//		m_eeprom_data.ui8_offroad_enabled_on_startup;
	//ui_vars->ui8_offroad_speed_limit =
	//		m_eeprom_data.ui8_offroad_speed_limit;
	//ui_vars->ui8_offroad_power_limit_enabled =
	//		m_eeprom_data.ui8_offroad_power_limit_enabled;
	//ui_vars->ui8_offroad_power_limit_div25 =
	//		m_eeprom_data.ui8_offroad_power_limit_div25;
	rt_vars->ui32_odometer_x10 = m_eeprom_data.ui32_odometer_x10; // odometer value should reside on RT vars
	ui_vars->ui8_walk_assist_feature_enabled =
			(m_eeprom_data.ui8_bit_data_1 & 128) >> 7;
	COPY_ARRAY(ui_vars, &m_eeprom_data, ui8_walk_assist_level_factor);
	COPY_ARRAY(ui_vars, &m_eeprom_data, field_selectors);
	COPY_ARRAY(ui_vars, &m_eeprom_data, graphs_field_selectors);
	ui_vars->ui8_buttons_up_down_invert = m_eeprom_data.ui8_bit_data_2 & 1;
	
	for (uint8_t i = 0; i < ASSIST_LEVEL_NUMBER; i++) {
		ui_vars->ui8_assist_level_factor[0][i] = m_eeprom_data.ui8_assist_level_factor[0][i];
		ui_vars->ui8_assist_level_factor[1][i] = m_eeprom_data.ui8_assist_level_factor[1][i];
		ui_vars->ui8_assist_level_factor[2][i] = m_eeprom_data.ui8_assist_level_factor[2][i];
		ui_vars->ui8_assist_level_factor[3][i] = m_eeprom_data.ui8_assist_level_factor[3][i];
	}
  
#ifndef SW102
  for (uint8_t i = 0; i < VARS_SIZE; i++) {
    g_graphVars[i].auto_max_min = m_eeprom_data.graph_eeprom[i].auto_max_min;
    g_graphVars[i].max = m_eeprom_data.graph_eeprom[i].max;
    g_graphVars[i].min = m_eeprom_data.graph_eeprom[i].min;
  }
  tripDistanceGraph.rw->graph.x_axis_scale_config = m_eeprom_data.tripDistanceField_x_axis_scale_config;
  graph_x_axis_scale_config_t temp = GRAPH_X_AXIS_SCALE_15M;
  if (tripDistanceGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = tripDistanceGraph.rw->graph.x_axis_scale_config;
  }
  tripDistanceGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsWheelSpeed].auto_thresholds = m_eeprom_data.wheelSpeedField_auto_thresholds;
  g_vars[VarsWheelSpeed].config_error_threshold = m_eeprom_data.wheelSpeedField_config_error_threshold;
  g_vars[VarsWheelSpeed].config_warn_threshold = m_eeprom_data.wheelSpeedField_config_warn_threshold;
  wheelSpeedGraph.rw->graph.x_axis_scale_config = m_eeprom_data.wheelSpeedField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (wheelSpeedGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = wheelSpeedGraph.rw->graph.x_axis_scale_config;
  }
  wheelSpeedGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsCadence].auto_thresholds = m_eeprom_data.cadenceField_auto_thresholds;
  g_vars[VarsCadence].config_error_threshold = m_eeprom_data.cadenceField_config_error_threshold;
  g_vars[VarsCadence].config_warn_threshold = m_eeprom_data.cadenceField_config_warn_threshold;
  cadenceGraph.rw->graph.x_axis_scale_config = m_eeprom_data.cadenceField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (cadenceGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = cadenceGraph.rw->graph.x_axis_scale_config;
  }
  cadenceGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsHumanPower].auto_thresholds = m_eeprom_data.humanPowerField_auto_thresholds;
  g_vars[VarsHumanPower].config_error_threshold = m_eeprom_data.humanPowerField_config_error_threshold;
  g_vars[VarsHumanPower].config_warn_threshold = m_eeprom_data.humanPowerField_config_warn_threshold;
  humanPowerGraph.rw->graph.x_axis_scale_config = m_eeprom_data.humanPowerField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (humanPowerGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = humanPowerGraph.rw->graph.x_axis_scale_config;
  }
  humanPowerGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsBatteryPower].auto_thresholds = m_eeprom_data.batteryPowerField_auto_thresholds;
  g_vars[VarsBatteryPower].config_error_threshold = m_eeprom_data.batteryPowerField_config_error_threshold;
  g_vars[VarsBatteryPower].config_warn_threshold = m_eeprom_data.batteryPowerField_config_warn_threshold;
  batteryPowerGraph.rw->graph.x_axis_scale_config = m_eeprom_data.batteryPowerField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (batteryPowerGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = batteryPowerGraph.rw->graph.x_axis_scale_config;
  }
  batteryPowerGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsBatteryPowerUsage].auto_thresholds = m_eeprom_data.batteryPowerUsageField_auto_thresholds;
  g_vars[VarsBatteryPowerUsage].config_error_threshold = m_eeprom_data.batteryPowerUsageField_config_error_threshold;
  g_vars[VarsBatteryPowerUsage].config_warn_threshold = m_eeprom_data.batteryPowerUsageField_config_warn_threshold;
  batteryPowerGraph.rw->graph.x_axis_scale_config = m_eeprom_data.batteryPowerUsageField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (batteryPowerUsageGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = batteryPowerUsageGraph.rw->graph.x_axis_scale_config;
  }
  batteryPowerUsageGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsBatteryVoltage].auto_thresholds = m_eeprom_data.batteryVoltageField_auto_thresholds;
  g_vars[VarsBatteryVoltage].config_error_threshold = m_eeprom_data.batteryVoltageField_config_error_threshold;
  g_vars[VarsBatteryVoltage].config_warn_threshold = m_eeprom_data.batteryVoltageField_config_warn_threshold;
  batteryVoltageGraph.rw->graph.x_axis_scale_config = m_eeprom_data.batteryVoltageField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (batteryVoltageGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = batteryVoltageGraph.rw->graph.x_axis_scale_config;
  }
  batteryVoltageGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsBatteryCurrent].auto_thresholds = m_eeprom_data.batteryCurrentField_auto_thresholds;
  g_vars[VarsBatteryCurrent].config_error_threshold = m_eeprom_data.batteryCurrentField_config_error_threshold;
  g_vars[VarsBatteryCurrent].config_warn_threshold = m_eeprom_data.batteryCurrentField_config_warn_threshold;
  batteryCurrentGraph.rw->graph.x_axis_scale_config = m_eeprom_data.batteryCurrentField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (batteryCurrentGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = batteryCurrentGraph.rw->graph.x_axis_scale_config;
  }
  batteryCurrentGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsMotorCurrent].auto_thresholds = m_eeprom_data.motorCurrentField_auto_thresholds;
  g_vars[VarsMotorCurrent].config_error_threshold = m_eeprom_data.motorCurrentField_config_error_threshold;
  g_vars[VarsMotorCurrent].config_warn_threshold = m_eeprom_data.motorCurrentField_config_warn_threshold;
  motorCurrentGraph.rw->graph.x_axis_scale_config = m_eeprom_data.motorCurrentField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (motorCurrentGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = motorCurrentGraph.rw->graph.x_axis_scale_config;
  }
  motorCurrentGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsMotorTemp].auto_thresholds = m_eeprom_data.motorTempField_auto_thresholds;
  g_vars[VarsMotorTemp].config_error_threshold = m_eeprom_data.motorTempField_config_error_threshold;
  g_vars[VarsMotorTemp].config_warn_threshold = m_eeprom_data.motorTempField_config_warn_threshold;
  motorTempGraph.rw->graph.x_axis_scale_config = m_eeprom_data.motorTempField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (motorTempGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = motorTempGraph.rw->graph.x_axis_scale_config;
  }
  motorTempGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsMotorERPS].auto_thresholds = m_eeprom_data.motorErpsField_auto_thresholds;
  g_vars[VarsMotorERPS].config_error_threshold = m_eeprom_data.motorErpsField_config_error_threshold;
  g_vars[VarsMotorERPS].config_warn_threshold = m_eeprom_data.motorErpsField_config_warn_threshold;
  motorErpsGraph.rw->graph.x_axis_scale_config = m_eeprom_data.motorErpsField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (motorErpsGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = motorErpsGraph.rw->graph.x_axis_scale_config;
  }
  motorErpsGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsMotorPWM].auto_thresholds = m_eeprom_data.pwmDutyField_auto_thresholds;
  g_vars[VarsMotorPWM].config_error_threshold = m_eeprom_data.pwmDutyField_config_error_threshold;
  g_vars[VarsMotorPWM].config_warn_threshold = m_eeprom_data.pwmDutyField_config_warn_threshold;
  pwmDutyGraph.rw->graph.x_axis_scale_config = m_eeprom_data.pwmDutyField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (pwmDutyGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = pwmDutyGraph.rw->graph.x_axis_scale_config;
  }
  pwmDutyGraph.rw->graph.x_axis_scale = temp;

  g_vars[VarsMotorFOC].auto_thresholds = m_eeprom_data.motorFOCField_auto_thresholds;
  g_vars[VarsMotorFOC].config_error_threshold = m_eeprom_data.motorFOCField_config_error_threshold;
  g_vars[VarsMotorFOC].config_warn_threshold = m_eeprom_data.motorFOCField_config_warn_threshold;
  motorFOCGraph.rw->graph.x_axis_scale_config = m_eeprom_data.motorFOCField_x_axis_scale_config;
  temp = GRAPH_X_AXIS_SCALE_15M;
  if (motorFOCGraph.rw->graph.x_axis_scale_config != GRAPH_X_AXIS_SCALE_AUTO) {
    temp = motorFOCGraph.rw->graph.x_axis_scale_config;
  }
  motorFOCGraph.rw->graph.x_axis_scale = temp;
#endif

  ui_vars->ui8_torque_sensor_calibration_feature_enabled = (m_eeprom_data.ui8_bit_data_2 & 2) >> 1;
  //ui_vars->ui8_torque_sensor_calibration_pedal_ground = (m_eeprom_data.ui8_bit_data_2 & 4) >> 2;
/*  for (uint8_t i = 0; i < 8; i++) {
    ui_vars->ui16_torque_sensor_calibration_table_left[i][0] = m_eeprom_data.ui16_torque_sensor_calibration_table_left[i][0];
    ui_vars->ui16_torque_sensor_calibration_table_left[i][1] = m_eeprom_data.ui16_torque_sensor_calibration_table_left[i][1];
    ui_vars->ui16_torque_sensor_calibration_table_right[i][0] = m_eeprom_data.ui16_torque_sensor_calibration_table_right[i][0];
    ui_vars->ui16_torque_sensor_calibration_table_right[i][1] = m_eeprom_data.ui16_torque_sensor_calibration_table_right[i][1];
  }
*/
  g_showNextScreenIndex = m_eeprom_data.showNextScreenIndex;

  ui_vars->ui8_street_mode_function_enabled =
      (m_eeprom_data.ui8_bit_data_2 & 8) >> 3;
  ui_vars->ui8_street_mode_enabled =
      (m_eeprom_data.ui8_bit_data_2 & 16) >> 4;
  ui_vars->ui8_street_mode_enabled_on_startup =
      (m_eeprom_data.ui8_bit_data_2 & 32) >> 5;
  ui_vars->ui8_street_mode_speed_limit =
      m_eeprom_data.ui8_street_mode_speed_limit;
  ui_vars->ui8_street_mode_power_limit_div25 =
      m_eeprom_data.ui8_street_mode_power_limit_div25;
  ui_vars->ui8_street_mode_throttle_enabled =
      (m_eeprom_data.ui8_bit_data_2 & 64) >> 6;
  ui_vars->ui8_street_mode_hotkey_enabled =
      (m_eeprom_data.ui8_bit_data_2 & 128) >> 7;
  ui_vars->ui8_street_mode_cruise_enabled =
      m_eeprom_data.ui8_bit_data_3 & 1;
  ui_vars->ui8_config_shortcut_key_enabled =
	  (m_eeprom_data.ui8_bit_data_3 & 2) >> 1;
  ui_vars->ui8_field_weakening =	  
	  (m_eeprom_data.ui8_bit_data_3 & 4) >> 2;
	  
  //ui_vars->ui8_pedal_cadence_fast_stop =
  //    m_eeprom_data.ui8_bit_data_3 & 1;
  ui_vars->ui8_coast_brake_adc =
      m_eeprom_data.ui8_coast_brake_adc;
  //ui_vars->ui8_adc_lights_current_offset =
  //    m_eeprom_data.ui8_adc_lights_current_offset;
  ui_vars->ui8_throttle_virtual_step =
      m_eeprom_data.ui8_throttle_virtual_step;
  ui_vars->ui8_torque_sensor_filter =
      m_eeprom_data.ui8_torque_sensor_filter;
  ui_vars->ui8_torque_sensor_adc_threshold =
      m_eeprom_data.ui8_torque_sensor_adc_threshold;
  ui_vars->ui8_coast_brake_enable =
      (m_eeprom_data.ui8_bit_data_1 & 4) >> 2;

  
  ui_vars->ui8_motor_acceleration_adjustment =
	  m_eeprom_data.ui8_motor_acceleration_adjustment;
	  ui_vars->ui8_motor_deceleration_adjustment =
	  m_eeprom_data.ui8_motor_deceleration_adjustment;
  ui_vars->ui8_pedal_torque_per_10_bit_ADC_step_x100 =
	  m_eeprom_data.ui8_pedal_torque_per_10_bit_ADC_step_x100;
  ui_vars->ui8_lights_configuration =
	  m_eeprom_data.ui8_lights_configuration;
  ui_vars->ui8_assist_whit_error_enabled =
	  (m_eeprom_data.ui8_bit_data_2 & 4) >> 2;
  ui_vars->ui16_startup_boost_torque_factor =
	  m_eeprom_data.ui16_startup_boost_torque_factor;
  ui_vars->ui8_startup_boost_cadence_step =
	  m_eeprom_data.ui8_startup_boost_cadence_step;
  ui_vars->ui16_adc_pedal_torque_offset =
	  m_eeprom_data.ui16_adc_pedal_torque_offset;
  ui_vars->ui16_adc_pedal_torque_max =
	  m_eeprom_data.ui16_adc_pedal_torque_max;
  ui_vars->ui8_weight_on_pedal =
	  m_eeprom_data.ui8_weight_on_pedal;
  ui_vars->ui16_adc_pedal_torque_calibration =
	  m_eeprom_data.ui16_adc_pedal_torque_calibration;
  ui_vars->ui8_motor_acceleration_after_braking =
	  m_eeprom_data.ui8_motor_acceleration_after_braking;
  ui_vars->ui8_motor_acceleration_time_after_braking =
	  m_eeprom_data.ui8_motor_acceleration_time_after_braking;
    
#ifndef SW102
  ui_vars->ui8_trip_a_auto_reset =
    m_eeprom_data.ui8_trip_a_auto_reset;
  ui_vars->ui16_trip_a_auto_reset_hours =
    m_eeprom_data.ui16_trip_a_auto_reset_hours;
  rt_vars->ui32_trip_a_last_update_time =
    m_eeprom_data.ui32_trip_a_last_update_time;

  ui_vars->ui8_trip_b_auto_reset =
    m_eeprom_data.ui8_trip_b_auto_reset;
  ui_vars->ui16_trip_b_auto_reset_hours =
    m_eeprom_data.ui16_trip_b_auto_reset_hours;
  rt_vars->ui32_trip_b_last_update_time =
    m_eeprom_data.ui32_trip_b_last_update_time;
#endif

  // trip A values should reside on RT vars
  rt_vars->ui32_trip_a_distance_x1000 =
      m_eeprom_data.ui32_trip_a_distance_x1000;
  rt_vars->ui32_trip_b_distance_x1000 =
      m_eeprom_data.ui32_trip_b_distance_x1000;
  rt_vars->ui32_trip_a_time =
      m_eeprom_data.ui32_trip_a_time;

  // trip B values should reside on RT vars
  rt_vars->ui32_trip_b_time =
      m_eeprom_data.ui32_trip_b_time;
  rt_vars->ui16_trip_a_max_speed_x10 =
      m_eeprom_data.ui16_trip_a_max_speed_x10;
  rt_vars->ui16_trip_b_max_speed_x10 =
      m_eeprom_data.ui16_trip_b_max_speed_x10;

}

void eeprom_write_variables(void) {
	ui_vars_t *ui_vars = get_ui_vars();
	
	// copy variables to m_eeprom_data
	m_eeprom_data.ui8_bit_data_1 = (ui_vars->ui8_units_type |
	  (ui_vars->ui8_motor_type << 1) |
	  (ui_vars->ui8_coast_brake_enable << 2) |
	  (ui_vars->ui8_motor_assistance_startup_without_pedal_rotation << 3) |
	  (ui_vars->ui8_startup_motor_power_boost_feature_enabled << 4) |
	  (ui_vars->ui8_pedal_cadence_fast_stop << 5) |
	  (ui_vars->ui8_cruise_feature_enabled << 6) |
	  (ui_vars->ui8_walk_assist_feature_enabled << 7));
	
	m_eeprom_data.ui8_bit_data_2 = (ui_vars->ui8_buttons_up_down_invert |
	  (ui_vars->ui8_torque_sensor_calibration_feature_enabled << 1) |
	  (ui_vars->ui8_assist_whit_error_enabled << 2) |
	  (ui_vars->ui8_street_mode_function_enabled << 3) |
	  (ui_vars->ui8_street_mode_enabled << 4) |
	  (ui_vars->ui8_street_mode_enabled_on_startup << 5) |
	  (ui_vars->ui8_street_mode_throttle_enabled << 6) |
	  (ui_vars->ui8_street_mode_hotkey_enabled << 7));
	
	m_eeprom_data.ui8_bit_data_3 = (ui_vars->ui8_street_mode_cruise_enabled |
	  (ui_vars->ui8_config_shortcut_key_enabled << 1) |
	  (ui_vars->ui8_field_weakening << 2));
	// bit free for future use
	
	m_eeprom_data.ui8_riding_mode = ui_vars->ui8_riding_mode;
	m_eeprom_data.ui8_assist_level = ui_vars->ui8_assist_level;
	m_eeprom_data.ui16_wheel_perimeter = ui_vars->ui16_wheel_perimeter;
	m_eeprom_data.ui8_wheel_max_speed = (uint8_t)
			(ui_vars->ui16_wheel_max_speed_x10 / 10);
	//m_eeprom_data.ui8_units_type = ui_vars->ui8_units_type;
	m_eeprom_data.ui32_wh_x10_offset = ui_vars->ui32_wh_x10_offset;
	m_eeprom_data.ui32_wh_x10_100_percent =
			ui_vars->ui32_wh_x10_100_percent;
	m_eeprom_data.ui8_battery_soc_enable =
			ui_vars->ui8_battery_soc_enable;
  m_eeprom_data.ui8_time_field_enable = ui_vars->ui8_time_field_enable;
  m_eeprom_data.ui8_target_max_battery_power_div25 =
      ui_vars->ui8_target_max_battery_power_div25;
  m_eeprom_data.ui8_battery_max_current =
      ui_vars->ui8_battery_max_current;
  m_eeprom_data.ui8_motor_max_current =
      ui_vars->ui8_motor_max_current;
  m_eeprom_data.ui8_motor_current_min_adc =
      ui_vars->ui8_motor_current_min_adc;
	//m_eeprom_data.ui8_field_weakening =
	//    ui_vars->ui8_field_weakening;
	//m_eeprom_data.ui8_ramp_up_amps_per_second_x10 =
	//	  ui_vars->ui8_ramp_up_amps_per_second_x10;
	m_eeprom_data.ui16_battery_low_voltage_cut_off_x10 =
		ui_vars->ui16_battery_low_voltage_cut_off_x10;
	//m_eeprom_data.ui8_motor_type = ui_vars->ui8_motor_type;
	//m_eeprom_data.ui8_motor_current_control_mode = ui_vars->ui8_motor_current_control_mode;
	//m_eeprom_data.ui8_motor_assistance_startup_without_pedal_rotation =
	//		ui_vars->ui8_motor_assistance_startup_without_pedal_rotation;
	m_eeprom_data.ui8_optional_ADC_function = ui_vars->ui8_optional_ADC_function;

	//COPY_ARRAY(&m_eeprom_data, ui_vars, ui16_assist_level_factor);
	for (uint8_t i = 0; i < ASSIST_LEVEL_NUMBER; i++) {
		m_eeprom_data.ui8_assist_level_factor[0][i] = ui_vars->ui8_assist_level_factor[0][i];
		m_eeprom_data.ui8_assist_level_factor[1][i] = ui_vars->ui8_assist_level_factor[1][i];
		m_eeprom_data.ui8_assist_level_factor[2][i] = ui_vars->ui8_assist_level_factor[2][i];
		m_eeprom_data.ui8_assist_level_factor[3][i] = ui_vars->ui8_assist_level_factor[3][i];
	}
	m_eeprom_data.ui8_number_of_assist_levels =
			ui_vars->ui8_number_of_assist_levels;
	//m_eeprom_data.ui8_startup_motor_power_boost_feature_enabled =
	//		ui_vars->ui8_startup_motor_power_boost_feature_enabled;
	//m_eeprom_data.ui8_startup_motor_power_boost_always =
	//		ui_vars->ui8_startup_motor_power_boost_always;
	//m_eeprom_data.ui8_startup_motor_power_boost_limit_power =
	//		ui_vars->ui8_startup_motor_power_boost_limit_power;
	//COPY_ARRAY(&m_eeprom_data, ui_vars,
	//		ui8_startup_motor_power_boost_factor);
	//m_eeprom_data.ui8_startup_motor_power_boost_time =
	//		ui_vars->ui8_startup_motor_power_boost_time;
	//m_eeprom_data.ui8_startup_motor_power_boost_fade_time =
	//		ui_vars->ui8_startup_motor_power_boost_fade_time;
	m_eeprom_data.ui8_motor_temperature_min_value_to_limit =
			ui_vars->ui8_motor_temperature_min_value_to_limit;
	m_eeprom_data.ui8_motor_temperature_max_value_to_limit =
			ui_vars->ui8_motor_temperature_max_value_to_limit;
	m_eeprom_data.ui16_battery_voltage_reset_wh_counter_x10 =
			ui_vars->ui16_battery_voltage_reset_wh_counter_x10;
	m_eeprom_data.ui8_lcd_power_off_time_minutes =
			ui_vars->ui8_lcd_power_off_time_minutes;
	m_eeprom_data.ui8_lcd_backlight_on_brightness =
			ui_vars->ui8_lcd_backlight_on_brightness;
	m_eeprom_data.ui8_lcd_backlight_off_brightness =
			ui_vars->ui8_lcd_backlight_off_brightness;
	m_eeprom_data.ui16_battery_pack_resistance_x1000 =
			ui_vars->ui16_battery_pack_resistance_x1000;
	//m_eeprom_data.ui8_offroad_feature_enabled =
	//		ui_vars->ui8_offroad_feature_enabled;
	//m_eeprom_data.ui8_offroad_enabled_on_startup =
	//		ui_vars->ui8_offroad_enabled_on_startup;
	//m_eeprom_data.ui8_offroad_speed_limit =
	//		ui_vars->ui8_offroad_speed_limit;
	//m_eeprom_data.ui8_offroad_power_limit_enabled =
	//		ui_vars->ui8_offroad_power_limit_enabled;
	//m_eeprom_data.ui8_offroad_power_limit_div25 =
	//		ui_vars->ui8_offroad_power_limit_div25;
	m_eeprom_data.ui32_odometer_x10 = ui_vars->ui32_odometer_x10;
	//m_eeprom_data.ui8_walk_assist_feature_enabled =
	//		ui_vars->ui8_walk_assist_feature_enabled;
	COPY_ARRAY(&m_eeprom_data, ui_vars, ui8_walk_assist_level_factor);
	COPY_ARRAY(&m_eeprom_data, ui_vars, field_selectors);
	COPY_ARRAY(&m_eeprom_data, ui_vars, graphs_field_selectors);
	//m_eeprom_data.ui8_buttons_up_down_invert = ui_vars->ui8_buttons_up_down_invert;

	//m_eeprom_data.ui8_torque_sensor_calibration_feature_enabled = ui_vars->ui8_torque_sensor_calibration_feature_enabled;
	//m_eeprom_data.ui8_torque_sensor_calibration_pedal_ground = ui_vars->ui8_torque_sensor_calibration_pedal_ground;
/*	for (uint8_t i = 0; i < 8; i++) {
		m_eeprom_data.ui16_torque_sensor_calibration_table_left[i][0] = ui_vars->ui16_torque_sensor_calibration_table_left[i][0];
		m_eeprom_data.ui16_torque_sensor_calibration_table_left[i][1] = ui_vars->ui16_torque_sensor_calibration_table_left[i][1];
		m_eeprom_data.ui16_torque_sensor_calibration_table_right[i][0] = ui_vars->ui16_torque_sensor_calibration_table_right[i][0];
		m_eeprom_data.ui16_torque_sensor_calibration_table_right[i][1] = ui_vars->ui16_torque_sensor_calibration_table_right[i][1];
	}
*/
#ifndef SW102
  for (uint8_t i = 0; i < VARS_SIZE; i++) {
    m_eeprom_data.graph_eeprom[i].auto_max_min = g_graphVars[i].auto_max_min;
    m_eeprom_data.graph_eeprom[i].max = g_graphVars[i].max;
    m_eeprom_data.graph_eeprom[i].min = g_graphVars[i].min;
  }
  m_eeprom_data.wheelSpeedField_auto_thresholds = g_vars[VarsWheelSpeed].auto_thresholds;
  m_eeprom_data.wheelSpeedField_config_error_threshold = g_vars[VarsWheelSpeed].config_error_threshold;
  m_eeprom_data.wheelSpeedField_config_warn_threshold = g_vars[VarsWheelSpeed].config_warn_threshold;
  m_eeprom_data.wheelSpeedField_x_axis_scale_config = wheelSpeedGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.cadenceField_auto_thresholds = g_vars[VarsCadence].auto_thresholds;
  m_eeprom_data.cadenceField_config_error_threshold = g_vars[VarsCadence].config_error_threshold;
  m_eeprom_data.cadenceField_config_warn_threshold = g_vars[VarsCadence].config_warn_threshold;
  m_eeprom_data.cadenceField_x_axis_scale_config = cadenceGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.humanPowerField_auto_thresholds = g_vars[VarsHumanPower].auto_thresholds;
  m_eeprom_data.humanPowerField_config_error_threshold = g_vars[VarsHumanPower].config_error_threshold;
  m_eeprom_data.humanPowerField_config_warn_threshold = g_vars[VarsHumanPower].config_warn_threshold;
  m_eeprom_data.humanPowerField_x_axis_scale_config = humanPowerGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.batteryPowerField_auto_thresholds = g_vars[VarsBatteryPower].auto_thresholds;
  m_eeprom_data.batteryPowerField_config_error_threshold = g_vars[VarsBatteryPower].config_error_threshold;
  m_eeprom_data.batteryPowerField_config_warn_threshold = g_vars[VarsBatteryPower].config_warn_threshold;
  m_eeprom_data.batteryPowerField_x_axis_scale_config = batteryPowerGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.batteryPowerUsageField_auto_thresholds = g_vars[VarsBatteryPowerUsage].auto_thresholds;
  m_eeprom_data.batteryPowerUsageField_config_error_threshold = g_vars[VarsBatteryPowerUsage].config_error_threshold;
  m_eeprom_data.batteryPowerUsageField_config_warn_threshold = g_vars[VarsBatteryPowerUsage].config_warn_threshold;
  m_eeprom_data.batteryPowerUsageField_x_axis_scale_config = batteryPowerUsageGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.batteryVoltageField_auto_thresholds = g_vars[VarsBatteryVoltage].auto_thresholds;
  m_eeprom_data.batteryVoltageField_config_error_threshold = g_vars[VarsBatteryVoltage].config_error_threshold;
  m_eeprom_data.batteryVoltageField_config_warn_threshold = g_vars[VarsBatteryVoltage].config_warn_threshold;
  m_eeprom_data.batteryVoltageField_x_axis_scale_config = batteryVoltageGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.batteryCurrentField_auto_thresholds = g_vars[VarsBatteryCurrent].auto_thresholds;
  m_eeprom_data.batteryCurrentField_config_error_threshold = g_vars[VarsBatteryCurrent].config_error_threshold;
  m_eeprom_data.batteryCurrentField_config_warn_threshold = g_vars[VarsBatteryCurrent].config_warn_threshold;
  m_eeprom_data.batteryCurrentField_x_axis_scale_config = batteryCurrentGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.motorCurrentField_auto_thresholds = g_vars[VarsMotorCurrent].auto_thresholds;
  m_eeprom_data.motorCurrentField_config_error_threshold = g_vars[VarsMotorCurrent].config_error_threshold;
  m_eeprom_data.motorCurrentField_config_warn_threshold = g_vars[VarsMotorCurrent].config_warn_threshold;
  m_eeprom_data.motorCurrentField_x_axis_scale_config = motorCurrentGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.motorTempField_auto_thresholds = g_vars[VarsMotorTemp].auto_thresholds;
  m_eeprom_data.motorTempField_config_error_threshold = g_vars[VarsMotorTemp].config_error_threshold;
  m_eeprom_data.motorTempField_config_warn_threshold = g_vars[VarsMotorTemp].config_warn_threshold;
  m_eeprom_data.motorTempField_x_axis_scale_config = motorTempGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.motorErpsField_auto_thresholds = g_vars[VarsMotorERPS].auto_thresholds;
  m_eeprom_data.motorErpsField_config_error_threshold = g_vars[VarsMotorERPS].config_error_threshold;
  m_eeprom_data.motorErpsField_config_warn_threshold = g_vars[VarsMotorERPS].config_warn_threshold;
  m_eeprom_data.motorErpsField_x_axis_scale_config = motorErpsGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.pwmDutyField_auto_thresholds = g_vars[VarsMotorPWM].auto_thresholds;
  m_eeprom_data.pwmDutyField_config_error_threshold = g_vars[VarsMotorPWM].config_error_threshold;
  m_eeprom_data.pwmDutyField_config_warn_threshold = g_vars[VarsMotorPWM].config_warn_threshold;
  m_eeprom_data.pwmDutyField_x_axis_scale_config = pwmDutyGraph.rw->graph.x_axis_scale_config;

  m_eeprom_data.motorFOCField_auto_thresholds = g_vars[VarsMotorFOC].auto_thresholds;
  m_eeprom_data.motorFOCField_config_error_threshold = g_vars[VarsMotorFOC].config_error_threshold;
  m_eeprom_data.motorFOCField_config_warn_threshold = g_vars[VarsMotorFOC].config_warn_threshold;
  m_eeprom_data.motorFOCField_x_axis_scale_config = motorFOCGraph.rw->graph.x_axis_scale_config;
#endif

  m_eeprom_data.showNextScreenIndex = g_showNextScreenPreviousIndex;

  //m_eeprom_data.ui8_street_mode_function_enabled =
  //    ui_vars->ui8_street_mode_function_enabled;
  //m_eeprom_data.ui8_street_mode_enabled =
  //    ui_vars->ui8_street_mode_enabled;
  //m_eeprom_data.ui8_street_mode_enabled_on_startup =
  //    ui_vars->ui8_street_mode_enabled_on_startup;
  m_eeprom_data.ui8_street_mode_speed_limit =
      ui_vars->ui8_street_mode_speed_limit;
  m_eeprom_data.ui8_street_mode_power_limit_div25 =
      ui_vars->ui8_street_mode_power_limit_div25;
  //m_eeprom_data.ui8_street_mode_throttle_enabled =
  //    ui_vars->ui8_street_mode_throttle_enabled;
  //m_eeprom_data.ui8_street_mode_hotkey_enabled =
  //    ui_vars->ui8_street_mode_hotkey_enabled;

  //m_eeprom_data.ui8_pedal_cadence_fast_stop =
  //    ui_vars->ui8_pedal_cadence_fast_stop;
  m_eeprom_data.ui8_coast_brake_adc =
      ui_vars->ui8_coast_brake_adc;
  //m_eeprom_data.ui8_adc_lights_current_offset =
  //    ui_vars->ui8_adc_lights_current_offset;
  m_eeprom_data.ui8_throttle_virtual_step =
      ui_vars->ui8_throttle_virtual_step;
  m_eeprom_data.ui8_torque_sensor_filter =
      ui_vars->ui8_torque_sensor_filter;
  m_eeprom_data.ui8_torque_sensor_adc_threshold =
      ui_vars->ui8_torque_sensor_adc_threshold;
  //m_eeprom_data.ui8_coast_brake_enable =
  //    ui_vars->ui8_coast_brake_enable;
			
#ifndef SW102
  m_eeprom_data.ui8_trip_a_auto_reset =
    ui_vars->ui8_trip_a_auto_reset;
  m_eeprom_data.ui16_trip_a_auto_reset_hours = 
    ui_vars->ui16_trip_a_auto_reset_hours;
  m_eeprom_data.ui32_trip_a_last_update_time =
    ui_vars->ui32_trip_a_last_update_time;
#endif

  m_eeprom_data.ui32_trip_a_distance_x1000 =
      ui_vars->ui32_trip_a_distance_x1000;
  m_eeprom_data.ui32_trip_a_time =
      ui_vars->ui32_trip_a_time;
  m_eeprom_data.ui16_trip_a_max_speed_x10 =
      ui_vars->ui16_trip_a_max_speed_x10;
  
#ifndef SW102
  m_eeprom_data.ui8_trip_b_auto_reset =
    ui_vars->ui8_trip_b_auto_reset;
  m_eeprom_data.ui16_trip_b_auto_reset_hours = 
    ui_vars->ui16_trip_b_auto_reset_hours;
  m_eeprom_data.ui32_trip_b_last_update_time =
    ui_vars->ui32_trip_b_last_update_time;
#endif

  m_eeprom_data.ui32_trip_b_distance_x1000 =
      ui_vars->ui32_trip_b_distance_x1000;
  m_eeprom_data.ui32_trip_b_time =
      ui_vars->ui32_trip_b_time;

  m_eeprom_data.ui16_trip_b_max_speed_x10 =
      ui_vars->ui16_trip_b_max_speed_x10;
	  
  m_eeprom_data.ui8_motor_acceleration_adjustment =
	  ui_vars->ui8_motor_acceleration_adjustment;
  m_eeprom_data.ui8_motor_deceleration_adjustment =
	  ui_vars->ui8_motor_deceleration_adjustment;
  m_eeprom_data.ui8_pedal_torque_per_10_bit_ADC_step_x100 =
	  ui_vars->ui8_pedal_torque_per_10_bit_ADC_step_x100;
  m_eeprom_data.ui8_lights_configuration =
	  ui_vars->ui8_lights_configuration;
  m_eeprom_data.ui16_startup_boost_torque_factor =
	  ui_vars->ui16_startup_boost_torque_factor;
  m_eeprom_data.ui8_startup_boost_cadence_step =
	  ui_vars->ui8_startup_boost_cadence_step;
  m_eeprom_data.ui16_adc_pedal_torque_offset =
	  ui_vars->ui16_adc_pedal_torque_offset;
  m_eeprom_data.ui16_adc_pedal_torque_max =
	  ui_vars->ui16_adc_pedal_torque_max;
  m_eeprom_data.ui8_weight_on_pedal =
	  ui_vars->ui8_weight_on_pedal;
  m_eeprom_data.ui16_adc_pedal_torque_calibration =
	  ui_vars->ui16_adc_pedal_torque_calibration;
  m_eeprom_data.ui8_motor_acceleration_after_braking =
	  ui_vars->ui8_motor_acceleration_after_braking;
  m_eeprom_data.ui8_motor_acceleration_time_after_braking =
	  ui_vars->ui8_motor_acceleration_time_after_braking;
	  
	flash_write_words(&m_eeprom_data, sizeof(m_eeprom_data) / sizeof(uint32_t));
}

void eeprom_init_defaults(void)
{
#ifdef SW102
  memset(&m_eeprom_data, 0, sizeof(m_eeprom_data));
  memcpy(&m_eeprom_data,
      &m_eeprom_data_defaults,
      sizeof(m_eeprom_data_defaults));

  eeprom_init_variables();
  set_conversions();
  // prepare torque_sensor_calibration_table as it will be used at begin to init the motor
  prepare_torque_sensor_calibration_table();

  flash_write_words(&m_eeprom_data, sizeof(m_eeprom_data) / sizeof(uint32_t));
#else
  // first force KEY value to 0
  eeprom_write(ADDRESS_KEY, 0);

  // eeprom_init() will read the default values now
  eeprom_init();
#endif
}
