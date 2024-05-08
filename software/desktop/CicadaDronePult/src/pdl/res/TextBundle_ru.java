package pdl.res;

import java.util.ListResourceBundle;

public class TextBundle_ru extends ListResourceBundle
{
    protected Object[][] getContents()
    {
        return new Object[][]
        {
        // LOCALIZE THIS
        	{"ERROR", "Ошибка"},
            {"SYSTEM_OK", "Все системы в норме"},
            
        // UI
            {"APP_TITLE", "Cicada - пульт управления (C) Антон Сысоев (anton.sysoev.ru68@gmail.com)"},
            {"MOTORS", "Двигатели"}
        // END OF MATERIAL TO LOCALIZE
        };
    }
}