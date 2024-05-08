package main;

import java.awt.Image;
import java.util.HashMap;
import java.util.Map;

import javax.swing.ImageIcon;

import pdl.res.TextBox;

public class ResBox
{
	private static Map<String, ImageIcon> icons;
	private static Map<String, Sound> sounds;
	
	public static void load()
	{
		icons = new HashMap<String, ImageIcon>();
		sounds = new HashMap<String, Sound>();
		loadImages();
		loadSounds();
	}
	
	public static String text(String key)
	{
		return TextBox.get(key);
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
	
	private static Sound loadSound(String path)
	{
		java.net.URL url = ResBox.class.getResource(path);
		return new Sound(url);
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
		icons.put("OPTICAL_FLOW_X", loadIcon("images/delta_x.png"));
		icons.put("OPTICAL_FLOW_Y", loadIcon("images/delta_y.png"));
		icons.put("CLOCK", loadIcon("images/clock.png"));
		icons.put("STOPWATCH", loadIcon("images/stopwatch.png"));
		icons.put("LOG", loadIcon("images/log.png"));
		icons.put("LOCATION",loadIcon("images/location.png"));
		icons.put("HOME",loadIcon("images/home.png"));
		icons.put("SATELLITE",loadIcon("images/satellite.png"));
		icons.put("LATITUDE",loadIcon("images/latitude.png"));
		icons.put("LONGITUDE",loadIcon("images/longitude.png"));
		icons.put("NORTH",loadIcon("images/north.png"));
		icons.put("EAST",loadIcon("images/east.png"));
		icons.put("HOR_VELO",loadIcon("images/horvelo.png"));
		icons.put("VERT_VELO",loadIcon("images/vertvelo.png"));
		icons.put("INFO",loadIcon("images/info.png"));
	}
	
	private static void loadSounds()
	{
		sounds.put("BAT5", loadSound("sounds/bat5.wav"));
		sounds.put("BAT10", loadSound("sounds/bat10.wav"));
		sounds.put("BAT15", loadSound("sounds/bat15.wav"));
		sounds.put("LOW_RADIO_SIGNAL", loadSound("sounds/low_radio_signal.wav"));
		sounds.put("ALT_1M", loadSound("sounds/alt1m.wav"));
		sounds.put("ALT_2M", loadSound("sounds/alt2m.wav"));
		sounds.put("ALT_3M", loadSound("sounds/alt3m.wav"));
		sounds.put("ALT_5M", loadSound("sounds/alt5m.wav"));
		sounds.put("ALT_7M", loadSound("sounds/alt7m.wav"));
		sounds.put("ALT_10M", loadSound("sounds/alt10m.wav"));
		sounds.put("ALT_15M", loadSound("sounds/alt15m.wav"));
		sounds.put("ALT_20M", loadSound("sounds/alt20m.wav"));
		sounds.put("ALT_30M", loadSound("sounds/alt30m.wav"));
		sounds.put("ALT_40M", loadSound("sounds/alt40m.wav"));
		sounds.put("ALT_50M", loadSound("sounds/alt50m.wav"));
		sounds.put("MOTORS_ENABLED", loadSound("sounds/motors_enabled.wav"));
		sounds.put("MOTORS_DISABLED", loadSound("sounds/motors_disabled.wav"));
		sounds.put("STABILIZATION_ENABLED", loadSound("sounds/stabilization_enabled.wav"));
		sounds.put("STABILIZATION_DISABLED", loadSound("sounds/stabilization_disabled.wav"));
		sounds.put("TRICK_MODE_ENABLED", loadSound("sounds/trick_mode_enabled.wav"));
		sounds.put("TRICK_MODE_DISABLED", loadSound("sounds/trick_mode_disabled.wav"));
		sounds.put("SYSTEM_OK", loadSound("sounds/system_ok.wav"));
		sounds.put("SYSTEM_BAD", loadSound("sounds/system_bad.wav"));
		sounds.put("VIDEO_STARTED", loadSound("sounds/video_started.wav"));
		sounds.put("VIDEO_STOPED", loadSound("sounds/video_stoped.wav"));
		sounds.put("PHOTO_TAKEN", loadSound("sounds/photo_taken.wav"));
	}
	
	public static Sound sound(String key)
	{
		Sound snd = sounds.get(key);
		return snd;
	}
}
