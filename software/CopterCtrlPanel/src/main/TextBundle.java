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
            {"COPTER_NOT_FOUND", "The copter is not found"},
            
            // UI
            {"APP_TITLE", "Control pult (C) Anton Sysoev (anton.sysoev.ru68@gmail.com)"},
            {"MOTORS", "Motors"},
            {"MOTORS_ENABLED", "Motors enabled"}
        // END OF MATERIAL TO LOCALIZE
        };
    }
}