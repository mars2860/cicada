package pdl.res;

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
            {"ALARM_DRONE_NOT_FOUND", "The drone is not found"},
            {"ALARM_SEND_ERROR", "Can't send data to the drone"},
            {"ALARM_SEND_SETUP_ERROR","Can't setup the drone. Try send setting or connect again"},
            {"ALARM_RECEIVE_ERROR", "Can't receive data from the drone"},
            {"ALARM_LOW_BATTERY", "Battery is low, go to home"},
            {"ALARM_LOW_BATTERY_CRITICAL", "Battery is very low, land right now"},
            {"ALARM_LOW_RADIO_SIGNAL", "Radio signal is low, you can lose control"},
            {"ALARM_ENABLE_VELOCITY_Z_PID", "Enable velocityZPid in settings"},
            {"ALARM_DISABLE_VELOCITY_Z_PID", "Disable velocityZPid in settings"},
            {"ALARM_ENABLE_ALT_PID", "Enable altPid in settings"},
            {"ALARM_GPS_NOT_FIXED","GPS is not fixed. Search for satellites"},
            {"ALARM_VERT_VELO_NOT_NULL","Vertical velocity is not null. Check your sensors. Drone can fly up at max speed if you turn on stabilization."},
            {"ALARM_DANGER_LEVEL","Pitch/Roll is dangerous. Try to send settings again"},
            {"ALARM_IMU_NOT_READY","No IMU data. Maybe IMU module is broken. Don't take off!"},
            {"ALARM_MAGNETO_NOT_READY","No magneto data. Maybe magnetometer is broken"},
            {"ALARM_CONNECTING","Connecting...Please wait"},
            {"ALARM_UNSUPPORTED_FIRMWARE","Unsupported firmware version"},
            {"ALARM_UDP_MODEM_CONNECTION_ERROR","Can't open UDP socket, check Net params in settings"},
        	{"ALARM_WIFI_BROADCAST_MODEM_NOT_FOUND","Wifi broadcast modem is not found"},
        	{"ALARM_CANT_OPEN_COM_PORT","Can't open com port to connect wifi broadcast modem"},
        	{"ALARM_MODEM_CONNECTION_LOST","Lost connection to the modem"},
            
            // UI
            //{"APP_TITLE", "Cicada drone pult (C) Anton Sysoev (anton.sysoev.ru68@gmail.com)"},
            {"APP_TITLE", "cicada drone pult v3 (C) www.liftelectronica.ru"},
            
            {"OK", "OK"},
            {"CANCEL", "Cancel"},
            {"UNKNOWN","Unknown"},
            
            {"STATUS", "Status"},
            {"BATTERY_STATE", "Battery voltage/percent"},
            {"WIFI_STATE", "Wifi RSSI (db)"},
            {"YAW_STATE", "Yaw (deg)/Target (deg)"},
            {"PITCH_STATE","Pitch (deg)/Target (deg)"},
            {"ROLL_STATE","Roll (deg)/Target (deg)"},
            {"HEADING_STATE","True azimuth (deg)"},
            {"ALTITUDE_STATE","Altitude (QNE meters)"},
            {"PRESSURE_STATE","Pressure (hPa)"},
            {"TEMPERATURE_STATE","Temperature (Celsius deg)"},
            {"LOOP_TIME_STATE","One loop time of thread (us)"},
            {"LIDAR_RANGE", "Range measured by lidar (meters)"},
            {"OPTICAL_FLOW_X", "Optical flow sensor delta X (tikcs)"},
            {"OPTICAL_FLOW_Y", "Optical flow sensor delta Y (ticks)"},
            {"POS_NORTH", "Position along North from start (meters)"},
            {"POS_EAST", "Position along East from start (meters)"},
            {"VELOCITY_X", "Velocity X (m/s)"},
            {"VELOCITY_Y", "Velocity Y (m/s)"},
            {"VELOCITY_Z", "Velocity Z (m/s)"},
            {"SYSTEM_TIME", "System time"},
            {"FLY_TIME", "Time of flight"},
            
            {"LOCATION","Location"},
            {"HOME_POS","Distance to home (meters)/Heading to home (deg)"},
            {"SATELL_COUNT","Num satellites in view"},
            {"LATITUDE","Latitude (deg)"},
            {"LONGITUDE","Longitude (deg)"},
            {"HOR_VELO","Horizontal velocity (m/s)"},
            {"VERT_VELO","Vertical velocity (m/s)"},
            
            {"MOTORS", "Motors"},
            {"MOTORS_ENABLED", "Motors enabled"},
            {"REVERSE","Reverse"},
            {"STABILIZATION_ENABLED","Stabilization"},
            {"HOLD_POS_ENABLED", "Hold position"},
            {"GAS", "Gas"},
            {"TRICK_MODE","Trick mode"},
            {"TRICK_MODE_GYRO","Trick Gyro"},
            {"TRICK_MODE_ACRO","Trick Acro"},
            
            {"CONNECT", "Connect"},
            {"CONNECTED", "Connected"},
            {"SENSORS", "Sensors"},
            {"CHARTS", "Charts"},
            {"SETTINGS", "Settings"},
            {"REMOTE_CONTROL", "Remote control"},
            {"LOG","Log"},
            {"INFO","Info"},
            {"RADIO_STATUS","Radio status"},
            
            {"IP_ADDRESS", "IP Address"},
            {"CMD_PORT", "Port 1"},
            {"TELEMETRY_PORT", "Port 2"},
            {"VIDEO_PORT", "Port 3"},
            {"LANGUAGE", "Lang"},
            {"START", "START"},
            {"STOP", "STOP"},
            {"NET", "Net"},
            {"MISC","Misc"},
            {"WIDGETS","Widgets"},
            {"SOUNDS","Sounds"},
            {"BATTERY", "Battery"},
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
            {"BARO", "Barometer"},
            {"OTHER", "Other"},
            {"USER_DATA1", "User data"},
            {"KALMAN_FILTER","Kalman filter"},
            {"GPS", "GPS"},
            {"LOAD1","Load1"},
            {"LOAD2","Load2"},
            {"LOAD3","Load3"},
            {"ANTITURTLE","Turtle"},
            {"CAMERA_ANGLE_0","Cam0"},
            {"CAMERA_ANGLE_45","Cam45"},
            {"CAMERA_ANGLE_90","Cam90"},
            {"VIDEO","Video"},
            {"PHOTO","Photo"},
            
            {"REALTIME","Realtime"},
            {"SUBMIT","Submit"},
            {"SNAPSHOT","Snapshot"},
            {"DATA","Data"},
            {"SAVE_IMAGE","Save image"},
            {"EXPORT_SEL_RANGE","Export sel range"},
            {"EXPORT_FULL_RANGE","Export full range"},
            {"TIME_AXIS","time (us)"},
            {"CLEAR_BLACK_BOX","Clear BlackBox"},
            {"DRAW_TRACK","Draw track"},
            
            {"ALARMS", "Alarms"},
            {"COLLECT_DATA", "Collect Data"},
            {"DATA_COUNT", "Data Count"},
            {"SHOW_MAGNET_RAW", "Show MagnetRaw"},
            {"SHOW_MAGNET_PURE", "Show MagnetPure"},
            {"CALIBRATE_ACCEL", "Calibrate A"},
            {"SELF_CALIBRATE_ACCEL", "Self Calib A"},
            {"SELF_CALIBRATE_GYRO", "Self Calib G"},
            {"CALIBRATE_GYRO", "Calibrate G"},
            {"CALIBRATE_MAGNET", "Calibrate M"},
            {"RESET_CALIBRATION", "Reset all sensors"},
            {"RESET_MAGNET_CALIBRATION", "Reset compass"},
            {"SAVE_CALIBRATION", "Save calibration"},
            
            {"CALIBRATE_INSTRUCTION_1", "Before put the drone on the flat (it would be better to put on the water level) and after set the collect data checkbox"},
            {"CALIBRATE_INSTRUCTION_2", "Before set the collect data checkbox and after rotate the drone in all directions"},
            {"WAIT_CALIBRATION", "The drone will be disconnected for one minute to process sensor calibration"},
            
            {"TURN_OFF_VELOZ_PID", "To control lift by Gas you have to disable velociyZPid in Settings"},
            {"TURN_ON_VELOZ_PID", "To control lift by Velocity/Alt you have to enable velociyZPid in Settings"},
            
            {"PID", "PID"},
            {"TARGET", "Target"},
            {"PID_ENABLED", "PID Enabled"},
            {"YAW_PID", "Yaw PID"},
            {"Z_RATE_PID", "Z Rate PID"},
            {"Y_RATE_PID", "Y Rate PID"},
            {"X_RATE_PID", "X Rate PID"},
            {"PITCH_PID", "Pitch PID"},
            {"ROLL_PID", "Roll PID"},
            {"ALT_PID", "Alt PID"},
            {"VELOCITY_X_PID", "Velocity-X PID"},
            {"VELOCITY_Y_PID", "Velocity-Y PID"},
            {"VELOCITY_Z_PID", "Velocity-Z PID"},
            {"HEADING_PID", "Heading PID"},
            {"POS_NORTH_PID", "PosNorth PID"},
            {"POS_EAST_PID", "PosEast PID"},
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
            {"APPLY","Apply"},
            {"SAVE_DEFAULT_CFG","Save Default"},
            {"LOAD_DEFAULT_CFG","Load Default"},
            
            {"ROTATE_BY","Rotate by"},
            {"YAW_RATE","Yaw Rate"},
            {"Z_RATE","Z Rate"},
            {"DELTA","\u0394"},
            {"MOVE_BY","Move by"},
            {"PITCH_ROLL_RATE","Pitch/Roll Rate"},
            {"X_Y_RATE","X/Y Rate"},
            {"PITCH_ROLL","Level"},
            {"VELOCITY","Velocity"},
            {"LIFT_BY","Lift by"},
            {"THROTTLE","Throttle"},
            {"FORSAGE","Forsage"},
            {"TRICK_ANG_RATE","Trick Ang.Rate (deg)"},
            {"ACCELERATION_TIME","Acceleration time"},
            {"CONTROL_SETTINGS","Keyboard/Gameapad Settings"},
            {"GAMEPAD","Gamepad"},
            {"GAMEPAD_NOT_FOUND","No gamepad found"},
            {"STICK","Stick"},
            {"STICK_TEST","Stick Test"},
            {"UP","Up"},
            {"DOWN","Down"},
            {"FORWARD","Forward"},
            {"BACK","Back"},
            {"LEFT","Left"},
            {"RIGHT","Right"},
            {"TURN_CW","Turn CW"},
            {"TURN_CCW","Turn CCW"},
            {"ARM_DISARM","Arm/Disarm"},
            {"TRICK_MODE","Trick on/off"},
            {"ALT","Alt"},
            {"FORSAGE_UP_GAS","Up gas"},
            {"FORSAGE_MIDDLE_GAS","Middle gas"},
            {"FORSAGE_MAX_MIDDLE_GAS","Max Mid gas"},
            {"FORSAGE_DOWN_GAS","Down gas"},
            {"LIFT","Lift"},
            {"MOVE","Move"},
            {"ROTATE","Rotate"},
            {"SECONDS","seconds"},
            {"ROTATE_BY_YAW_RATE","yaw rate pid"},
            {"ROTATE_BY_HEADING","heading pid"},
            {"MOVE_BY_PITCH_ROLL_RATE","pitch/roll rate pid"},
            {"MOVE_BY_PITCH_ROLL","pitch/roll level pid"},
            {"MOVE_BY_VELOCITY","xy-velocity pid"},
            {"LIFT_BY_GAS_DELTA","gas increment"},
            {"LIFT_BY_VELOCITY","z-velocity pid"},
            {"LIFT_BY_ALT","alt pid"},
            {"LIFT_BY_GAS_FORSAGE","gas forsage"},
            {"LIFT_BY_GAS","gas"},
            {"PITCH_AUTO_LEVEL","pitchAutoLevel"},
            {"ROLL_AUTO_LEVEL","rollAutoLevel"},
            
            {"SETTINGS_RECEIVED","Settings has been received from a drone"},
            {"SETTINGS_SENT","New settings has been sent to a drone"},
            {"SETTINGS_SAVED","Settings has been saved to a disk of PC"},
            {"SETTINGS_LOADED","Settings has been loaded from a disk of PC"},
            {"SETTINGS_SAVED_DEFAULT","Default settings has been saved to a flash-memory of a drone"},
            {"SETTINGS_LOADED_DEFAULT","Default settings has been loaded from a flash-memory of a drone"},
            {"CANT_GRAB_SETTINGS","Can't grab new settings from a table"},
            {"INVALID_VALUE","Invalid value for"},
            {"CANT_LOAD_PROFILE","Can't load a profile"},
            {"CANT_SAVE_PROFILE","Can't save a profile"},
            {"TYPE_PROFILE_NAME","Type a profile name"}
        // END OF MATERIAL TO LOCALIZE
        };
    }
}