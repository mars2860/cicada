package main;

import java.util.ListResourceBundle;

public class TextBundle_ru extends ListResourceBundle
{
    protected Object[][] getContents()
    {
        return new Object[][]
        {
        // LOCALIZE THIS
        	{"ERROR", "������"},
            {"SYSTEM_OK", "��� ������� � �����"},
            
        // UI
            {"APP_TITLE", "����� ���������� (C) ����� ������ (anton.sysoev.ru68@gmail.com)"},
            {"MOTORS", "���������"}
        // END OF MATERIAL TO LOCALIZE
        };
    }
}