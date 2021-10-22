package main;

import java.awt.Image;
import java.util.HashMap;
import java.util.Locale;
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
		if(Locale.getDefault().getLanguage().startsWith("ru"))
			bundle = ResourceBundle.getBundle("main.TextBundle_ru");
		else
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
		return loadIcon(path,32,32);
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
		icons.put("REMOTE_CONTROL", loadIcon("images/remotecontrol.png"));
		icons.put("ARROW_UP", loadIcon("images/arrow_up.png"));
		icons.put("ARROW_DOWN", loadIcon("images/arrow_down.png"));
		icons.put("ARROW_LEFT", loadIcon("images/arrow_left.png"));
		icons.put("ARROW_RIGHT", loadIcon("images/arrow_right.png"));
		icons.put("ARROW_GET_UP", loadIcon("images/get_up.png"));
		icons.put("ARROW_GET_DOWN", loadIcon("images/get_down.png"));
		icons.put("ROTATE_CW", loadIcon("images/rotate_cw.png"));
		icons.put("ROTATE_CCW", loadIcon("images/rotate_ccw.png"));
		icons.put("STOP", loadIcon("images/stop.png"));
		icons.put("LIDAR", loadIcon("images/lidar.png"));
		icons.put("OPTICAL_FLOW_X", loadIcon("images/opticalFlowX.png"));
		icons.put("OPTICAL_FLOW_Y", loadIcon("images/opticalFlowY.png"));
	}
}
