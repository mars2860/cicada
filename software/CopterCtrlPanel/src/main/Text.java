package main;

import java.util.ResourceBundle;

public class Text
{
	private static ResourceBundle bundle;
	
	public static void load()
	{
		ResourceBundle.clearCache();
		bundle = ResourceBundle.getBundle("main.TextBundle");
	}
	
	public static String get(String key)
	{
		return bundle.getString(key);
	}
}
