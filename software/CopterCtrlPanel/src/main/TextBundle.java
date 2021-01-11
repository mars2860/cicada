package main;

import java.util.ListResourceBundle;

public class TextBundle extends ListResourceBundle
{
    protected Object[][] getContents()
    {
        return new Object[][]
        {
        // LOCALIZE THIS
        	{"ERROR", "Error"},
            {"SYSTEM_OK", "The system is ok"},
            {"INVALID_HOST", "Invalid host name"},
            {"SOCKET_NOT_OPEN", "Can't open socket"},

            // ALARMS
            {"COPTER_NOT_FOUND", "The copter is not found"},
            {"COPTER_SEND_ERROR", "Can't send data to copter"},
            {"COPTER_RECEIVE_ERROR", "Can't receive data from copter"},
            
            // UI
            {"APP_TITLE", "Control pult (C) Anton Sysoev (anton.sysoev.ru68@gmail.com)"},
            
            {"OK", "OK"},
            {"CANCEL", "Cancel"},
            
            {"STATUS", "Status"},
            {"BATTERY_STATE", "Battery voltage/percent"},
            {"WIFI_STATE", "Wifi RSSI (db)"},
            {"YAW_STATE", "Yaw (deg)/Target (deg)"},
            {"PITCH_STATE","Pitch (deg)/Target (deg)"},
            {"ROLL_STATE","Roll (deg)/Target (deg)"},
            {"HEADING_STATE","Magnetic azimuth (deg)"},
            {"ALTITUDE_STATE","Altitude (QNE meters)"},
            {"PRESSURE_STATE","Pressure (hPa)"},
            {"TEMPERATURE_STATE","Temperature (Celsius deg)"},
            {"LOOP_TIME_STATE","One loop time of thread (us)"},
            
            
            {"MOTORS", "Motors"},
            {"MOTORS_ENABLED", "Motors enabled"},
            {"STABILIZATION_ENABLED","Stabilization"},
            {"GAS", "Gas"},
            
            {"CONNECT", "Connect"},
            {"SENSORS", "Sensors"},
            {"CHARTS", "Charts"},
            {"SETTINGS", "Settings"},
            {"REMOTE_CONTROL", "Remote control"},
            
            {"IP_ADDRESS", "IP Address"},
            {"CMD_PORT", "Port 1"},
            {"TELEMETRY_PORT", "Port 2"},
            {"VIDEO_PORT", "Port 3"},
            {"LANGUAGE", "Lang"},
            {"START", "START"},
            {"STOP", "STOP"},
            {"NET", "Net"},
            {"CALIBRATION", "Calibration"},
            {"UPDATE_PERIODS", "Update Periods"},
            {"TELEMETRY", "Telemetry"},
            {"PERIOD", "Period"},
            {"PARAM", "Parameter"},
            {"NEW_VALUE", "New value"},
            {"CURRENT_VALUE", "Current value"},
            {"ACCEL", "Accelerometer"},
            {"GYRO", "Gyroscope"},
            {"MAGNETO", "Magnetometer"},
            {"OTHER", "Other"},
            
            {"REALTIME","Realtime"},
            {"SUBMIT","Submit"},
            {"SNAPSHOT","Snapshot"},
            {"DATA","Data"},
            {"SAVE_IMAGE","Save image"},
            {"EXPORT_SEL_RANGE","Export sel range"},
            {"EXPORT_FULL_RANGE","Export full range"},
            {"TIME_AXIS","time (us)"},
            
            {"ALARMS", "Alarms"},
            {"COLLECT_DATA", "Collect Data"},
            {"DATA_COUNT", "Data Count"},
            {"SHOW_ACCEL", "Show accel"},
            {"SHOW_MAGNET", "Show magnet"},
            {"CALIBRATE_ACCEL", "Calibrate A"},
            {"SELF_CALIBRATE_ACCEL", "Self Calib A"},
            {"SELF_CALIBRATE_GYRO", "Self Calib G"},
            {"CALIBRATE_GYRO", "Calibrate G"},
            {"CALIBRATE_MAGNET", "Calibrate M"},
            {"RESET_CALIBRATION", "Reset all sensors"},
            {"RESET_MAGNET_CALIBRATION", "Reset compass"},
            {"SAVE_CALIBRATION", "Save calibration"},
            {"CALIBRATE_INSTRUCTION_1", "Before put the copter on the flat (it would be better to put on the water level) and after set the collect data checkbox"},
            {"CALIBRATE_INSTRUCTION_2", "Before set the collect data checkbox and after rotate the copter in all directions"},
            {"WAIT_CALIBRATION", "The copter will be disconnected for one minute to process sensor calibration"},
            
            {"PID", "PID"},
            {"TARGET", "Target"},
            {"PID_ENABLED", "PID Enabled"},
            {"YAW_PID", "Yaw PID"},
            {"YAW_RATE_PID", "Yaw Rate PID"},
            {"PITCH_PID", "Pitch PID"},
            {"ROLL_PID", "Roll PID"},
            {"ALT_PID", "Alt PID"},
            {"YAW","Yaw"},
            {"PITCH","Pitch"},
            {"ROLL","Roll"},
            {"HEADING","Heading"},
            {"ALTITUDE","Alt"},
            {"YAW_OUTPUT","Y-out"},
            {"PITCH_OUTPUT","P-out"},
            {"ROLL_OUTPUT","R-out"},
            {"ALT_OUTPUT","A-out"},
            {"PLOT","Plot"},
            {"SET", "Set"},
            {"CONFIRM_SAVE_SETTINGS", "Do you want to save settings?"},
            {"RECEIVE","Receive"},
            {"SEND","Send"},
            {"LOAD","Load"},
            {"SAVE","Save"},
            {"SETTINGS_RECEIVED","Settings has been received from a drone"},
            {"SETTINGS_SENT","New settings has been sent to a drone"},
            {"SETTINGS_SAVED","Settings has been saved to a disk"},
            {"SETTINGS_LOADED","Settings has been loaded from a disk"},
            {"CANT_GRAB_SETTINGS","Can't grab new settings from a table"},
            {"INVALID_VALUE","Invalid value for"},
        // END OF MATERIAL TO LOCALIZE
        };
    }
}