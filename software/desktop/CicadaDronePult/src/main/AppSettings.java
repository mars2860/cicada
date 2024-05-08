package main;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.annotations.Expose;

import pdl.res.Profile;
import pdl.res.TextBox;

public class AppSettings
{
	private static AppSettings mSingleton;
	
	public static AppSettings instance()
	{
		if(mSingleton == null)
			mSingleton = new AppSettings();
		
		return mSingleton;
	}
	
	public static final String SETTINGS_FILENAME = "settings.json";
	public static final String PROFILES_DIR = "./profiles";
	
	public static class InputCtrl
	{
		@Expose
		public String btn1;
		@Expose
		public String btn2;
		@Expose
		public String stick;
		
		public InputCtrl()
		{
			btn1 = "";
			btn2 = "";
			stick = "";
		}
	}
	
	public static class InputMap
	{
		public static final int UP = 0;
		public static final int DOWN = 1;
		public static final int FWD = 2;
		public static final int BACK = 3;
		public static final int LEFT = 4;
		public static final int RIGHT = 5;
		public static final int TURN_CW = 6;
		public static final int TURN_CCW = 7;
		public static final int ARM_DISARM = 8;
		public static final int TRICK_MODE = 9;
		public static final int LOAD1 = 10;
		public static final int LOAD2 = 11;
		public static final int ANTITURTLE = 12;
		public static final int CAMERA_ANGLE_0 = 13;
		public static final int CAMERA_ANGLE_45 = 14;
		public static final int CAMERA_ANGLE_90 = 15;
		public static final int PHOTO = 16;
		public static final int VIDEO = 17;
				
		public static final int INPUT_CONTROL_COUNT = 18;
		
		@Expose
		public String gamepad = "";
		@Expose
		public InputCtrl controls[] = new InputCtrl[INPUT_CONTROL_COUNT];
		
		public InputMap()
		{
			setDefault();
		}
		
		public void setDefault()
		{
			for(int i = 0; i < INPUT_CONTROL_COUNT; i++)
			{
				controls[i] = new InputCtrl();
				
				switch(i)
				{
				case UP:
					controls[i].btn1 = "Z";
					controls[i].btn2 = "G_UP";
					controls[i].stick = "y";
					break;
				case DOWN:
					controls[i].btn1 = "X";
					controls[i].btn2 = "G_DOWN";
					controls[i].stick = "y";
					break;
				case FWD:
					controls[i].btn1 = "W";
					controls[i].btn2 = "G_3";
					controls[i].stick = "ry";
					break;
				case BACK:
					controls[i].btn1 = "S";
					controls[i].btn2 = "G_0";
					controls[i].stick = "ry";
					break;
				case LEFT:
					controls[i].btn1 = "A";
					controls[i].btn2 = "G_2";
					controls[i].stick = "rx";
					break;
				case RIGHT:
					controls[i].btn1 = "D";
					controls[i].btn2 = "G_1";
					controls[i].stick = "rx";
					break;
				case TURN_CW:
					controls[i].btn1 = "E";
					controls[i].btn2 = "G_RIGHT";
					controls[i].stick = "x";
					break;
				case TURN_CCW:
					controls[i].btn1 = "Q";
					controls[i].btn2 = "G_LEFT";
					controls[i].stick = "x";
					break;
				case ARM_DISARM:
					controls[i].btn1 = "Escape";
					controls[i].btn2 = "G_7";
					controls[i].stick = "no stick";
					break;
				case TRICK_MODE:
					controls[i].btn1 = " ";
					controls[i].btn2 = "G_4";
					controls[i].stick = "no stick";
					break;
				case LOAD1:
					controls[i].btn1 = "M";
					controls[i].btn2 = "G_6";
					controls[i].stick = "no stick";
					break;
				case LOAD2:
					controls[i].btn1 = "N";
					controls[i].btn2 = "G_5";
					controls[i].stick = "no stick";
					break;
				case ANTITURTLE:
					controls[i].btn1 = "B";
					controls[i].btn2 = "G_8";
					controls[i].stick = "no stick";
					break;
				}
			}
		}
	}
	
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
	private WndState locationWnd;
	@Expose
	private WndState settingsWnd;
	@Expose
	private WndState rcWnd;
	@Expose
	private WndState logWnd;
	@Expose
	private WndState rcSettingsWnd;
	@Expose
	private String lang;
	@Expose
	private String profile;
	@Expose
	private InputMap inputMap;
	
	private AppSettings()
	{
		setDefault();
	}
	
	public void setLang(String lang)
	{
		this.lang = lang;
	}
	
	public String getLang()
	{
		return lang;
	}
	
	public InputMap getInputMap()
	{
		return inputMap;
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
	
	public void setLocationWnd(WndState ws)
	{
		locationWnd = ws;
	}
	
	public WndState getLocationWnd()
	{
		return locationWnd;
	}
	
	public void setSettingsWnd(WndState ws)
	{
		settingsWnd = ws;
	}
	
	public WndState getSettingsWnd()
	{
		return settingsWnd;
	}
	
	public void setRcWnd(WndState ws)
	{
		rcWnd = ws;
	}
	
	public WndState getRcWnd()
	{
		return rcWnd;
	}
	
	public void setRcSettingsWnd(WndState ws)
	{
		rcSettingsWnd = ws;
	}
	
	public WndState getRcSettingsWnd()
	{
		return rcSettingsWnd;
	}
	
	public void setLogWnd(WndState ws)
	{
		logWnd = ws;
	}
	
	public WndState getLogWnd()
	{
		return logWnd;
	}

	public void save()
	{
		Gson gson = new GsonBuilder()
				 .excludeFieldsWithModifiers(java.lang.reflect.Modifier.TRANSIENT)
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
		lang = Locale.ENGLISH.getLanguage();
        TextBox.load(new Locale(lang));
        
		mainWnd = new WndState();
		chartsWnd = new WndState();
		motorsWnd = new WndState();
		sensorsWnd = new WndState();
		locationWnd = new WndState();
		statusWnd = new WndState();
		settingsWnd = new WndState();
		rcWnd = new WndState();
		rcSettingsWnd = new WndState();
		logWnd = new WndState();
		profile = "default";
		inputMap = new InputMap();
	}
	
	public String getCurProfileName()
	{
		return profile;
	}
	
	public void setCurProfileName(String profileName)
	{
		profile = profileName;
	}
	
	public File getProfileFile(String profileName)
	{
		File file = new File(AppSettings.PROFILES_DIR,profileName + Profile.EXT);
		return file;
	}
	
	public File getCurProfileFile()
	{
		return getProfileFile(getCurProfileName());
	}
	
	public List<String> listProfiles()
	{
		List<String> lstProfiles = new ArrayList<String>();
		
		File file = new File(PROFILES_DIR);
		
        String arrProfiles[] = file.list(new FilenameFilter()
		{
            @Override
            public boolean accept(File dir, String filename)
            {
                return filename.toLowerCase().endsWith(Profile.EXT);
            }
		});
		

        for(int i = 0; i < arrProfiles.length; i++)
        {
        	// remove .json from profile name
            arrProfiles[i] = arrProfiles[i].split("[.]")[0];
            lstProfiles.add(arrProfiles[i]);
        }

		
		return lstProfiles;
	}
	
	public static void load()
	{
		try
		{
			String content = new String(Files.readAllBytes(Paths.get(SETTINGS_FILENAME)));
			
			Gson gson = new GsonBuilder()
					 .excludeFieldsWithModifiers(java.lang.reflect.Modifier.TRANSIENT)
					 .excludeFieldsWithoutExposeAnnotation().create();

			mSingleton = gson.fromJson(content, AppSettings.class);
			
			TextBox.load(new Locale(mSingleton.getLang()));
			
			// if we add new control to new software there is ArrayOutOfIndex when we load old settings
			if(mSingleton.inputMap.controls.length != InputMap.INPUT_CONTROL_COUNT)
			{
				InputCtrl ctrl[] = Arrays.copyOf(mSingleton.inputMap.controls,InputMap.INPUT_CONTROL_COUNT);
				mSingleton.inputMap.controls = new InputCtrl[InputMap.INPUT_CONTROL_COUNT];
				mSingleton.inputMap.setDefault();
				for(int i = 0; i < InputMap.INPUT_CONTROL_COUNT; i++)
				{
					if(ctrl[i] != null)
					{
						mSingleton.inputMap.controls[i] = ctrl[i];
					}
				}
			}
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}
}
