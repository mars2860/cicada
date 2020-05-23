package main;

import java.awt.Image;
import java.util.HashMap;
import java.util.Map;
import java.util.ResourceBundle;

import javax.swing.ImageIcon;

public class ResBox
{
	private static ResourceBundle bundle;
	private static Map<String, ImageIcon> icons;
	
	public static void load()
	{
		ResourceBundle.clearCache();
		icons = new HashMap<String, ImageIcon>();
		bundle = ResourceBundle.getBundle("main.TextBundle");
		loadImages();
	}
	
	public static String text(String key)
	{
		return bundle.getString(key);
	}
	
	public static ImageIcon icon(String key)
	{
		ImageIcon icon = icons.get(key);
		
		if(icon == null)
			icon = icons.get("ERROR");
		
		return icon;
	}
	
	private static ImageIcon loadIcon(String path)
	{
		return loadIcon(path,24,24);
	}
	
	private static ImageIcon loadIcon(String path, int w, int h)
	{
		java.net.URL url = ResBox.class.getResource(path);
		return new ImageIcon(new ImageIcon(url).getImage().getScaledInstance(w,h,Image.SCALE_SMOOTH));
	}
	
	private static void loadImages()
	{
		icons.put("OK", loadIcon("images/ok.png"));
		icons.put("ERROR", loadIcon("images/error.png"));
		icons.put("BATTERY", loadIcon("images/battery.png"));
		icons.put("WIFI", loadIcon("images/wifi.png"));
		icons.put("YAW", loadIcon("images/yaw.png"));
		icons.put("PITCH", loadIcon("images/pitch.png"));
		icons.put("ROLL", loadIcon("images/roll.png"));
		icons.put("HEADING", loadIcon("images/heading.png"));
		icons.put("CPUTIME", loadIcon("images/cputime.png"));
		icons.put("TEMPERATURE", loadIcon("images/temperature.png"));
		icons.put("PRESSURE", loadIcon("images/pressure.png"));
		icons.put("ALTITUDE", loadIcon("images/altitude.png"));
		icons.put("PROPELLER", loadIcon("images/propeller.png"));
		icons.put("KEY", loadIcon("images/key.png"));
		icons.put("CHARTS", loadIcon("images/charts.png"));
		icons.put("SENSORS", loadIcon("images/sensors.png"));
		icons.put("GAUGE", loadIcon("images/gauge.png"));
		icons.put("SETTINGS", loadIcon("images/settings.png"));
	}
}
