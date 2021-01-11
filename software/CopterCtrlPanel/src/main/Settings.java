package main;

import java.io.FileOutputStream;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Locale;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.annotations.Expose;

import copter.DroneState;

public class Settings
{
	private static Settings mSingleton;
	
	public static Settings instance()
	{
		if(mSingleton == null)
			mSingleton = new Settings();
		
		return mSingleton;
	}
	
	public static final String SETTINGS_FILENAME = "settings.json";
	public static final String LANG = "Language";
	public static final String DRONE_SETTINGS = "droneSettings";
	
	public static class WndState
	{
		@Expose
		int x;
		@Expose
		int y;
		@Expose
		int w;
		@Expose
		int h;
	}
	
	@Expose
	private DroneState droneSettings;
	@Expose
	private String lang;
	@Expose
	private WndState mainWnd;
	@Expose
	private WndState chartsWnd;
	@Expose
	private WndState motorsWnd;
	@Expose
	private WndState sensorsWnd;
	@Expose
	private WndState statusWnd;
	@Expose
	private WndState settingsWnd;
	
	private Settings()
	{
		setDefault();
	}
	
	public void setMainWnd(WndState ws)
	{
		mainWnd = ws;
	}
	
	public WndState getMainWnd()
	{
		return mainWnd;
	}
	
	public void setChartsWnd(WndState ws)
	{
		chartsWnd = ws;
	}
	
	public WndState getChartsWnd()
	{
		return chartsWnd;
	}
	
	public void setMotorsWnd(WndState ws)
	{
		motorsWnd = ws;
	}
	
	public WndState getMotorsWnd()
	{
		return motorsWnd;
	}
	
	public void setSensorsWnd(WndState ws)
	{
		sensorsWnd = ws;
	}
	
	public WndState getSensorsWnd()
	{
		return sensorsWnd;
	}
	
	public void setStatusWnd(WndState ws)
	{
		statusWnd = ws;
	}
	
	public WndState getStatusWnd()
	{
		return statusWnd;
	}
	
	public void setSettingsWnd(WndState ws)
	{
		settingsWnd = ws;
	}
	
	public WndState getSettingsWnd()
	{
		return settingsWnd;
	}
	
	public void setLang(String lang)
	{
		this.lang = lang;
	}
	
	public String getLang()
	{
		return lang;
	}
	
	public void setDroneSettings(DroneState ds)
	{
		// TODO после получения ds от CopterTelemetry стираются настройки сети (DroneState.Net)
		droneSettings = ds;
	}
	
	public DroneState getDroneSettings()
	{
		DroneState ds = new DroneState();
		
		try
		{
			ds = (DroneState)droneSettings.clone(); 
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
		
		return ds; 
	}

	public void save()
	{
		Gson gson = new GsonBuilder()
				 .excludeFieldsWithModifiers(Modifier.STATIC)
				 .excludeFieldsWithoutExposeAnnotation().create();
		
		String json = gson.toJson(this);
		
		try(FileOutputStream fos = new FileOutputStream(SETTINGS_FILENAME))
		{
			fos.write(json.getBytes());
			fos.close();
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}
	
	public void setDefault()
	{
		Locale.setDefault(Locale.ENGLISH);
		
		droneSettings = new DroneState();
		lang = Locale.ENGLISH.getLanguage();
		mainWnd = new WndState();
		chartsWnd = new WndState();
		motorsWnd = new WndState();
		sensorsWnd = new WndState();
		statusWnd = new WndState();
		settingsWnd = new WndState();
	}
	
	public void load()
	{
		try
		{
			String content = new String(Files.readAllBytes(Paths.get(SETTINGS_FILENAME)));
			
			Gson gson = new GsonBuilder()
					 .excludeFieldsWithModifiers(Modifier.STATIC)
					 .excludeFieldsWithoutExposeAnnotation().create();

			mSingleton = gson.fromJson(content, Settings.class);
			
			Locale.setDefault(new Locale(this.getLang()));
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}
}
