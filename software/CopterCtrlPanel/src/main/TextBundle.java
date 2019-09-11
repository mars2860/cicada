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
            
            {"MOTORS", "Motors"},
            {"MOTORS_ENABLED", "Motors enabled"},
            
            {"SENSORS", "Sensors"},
            {"SETTINGS", "Settings"},
            {"IP_ADDRESS", "IP Address"},
            {"CMD_PORT", "Port 1"},
            {"TELEMETRY_PORT", "Port 2"},
            {"VIDEO_PORT", "Port 3"},
            {"LANGUAGE", "Lang"},
            {"START", "START"},
            {"STOP", "STOP"},
            
            {"ALARMS", "Alarms"},
        // END OF MATERIAL TO LOCALIZE
        };
    }
}