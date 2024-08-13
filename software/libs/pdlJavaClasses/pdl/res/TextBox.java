package pdl.res;

import java.util.Locale;
import java.util.ResourceBundle;

public class TextBox
{
	private static ResourceBundle bundle;
	
	public static void load(Locale locale)
	{
		ResourceBundle.clearCache();
		if(locale.getLanguage().startsWith("ru"))
			bundle = ResourceBundle.getBundle("pdl.res.TextBundle_ru");
		else
			bundle = ResourceBundle.getBundle("pdl.res.TextBundle");
	}
	
	public static String get(String key)
	{
		return bundle.getString(key);
	}
}
