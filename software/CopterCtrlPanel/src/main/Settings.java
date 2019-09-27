package main;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.Properties;

public class Settings
{
	private static Settings mSingleton;
	
	public static Settings instance()
	{
		if(mSingleton == null)
			mSingleton = new Settings();
		
		return mSingleton;
	}
	
	public static final String COPTER_DEFAULT_IP = "192.168.1.33";
	public static final int COPTER_DEFAULT_CMD_PORT = 4210;
	public static final int COPTER_DEFAULT_TELEMETRY_PORT = 4211;
	public static final int COPTER_DEFAULT_VIDEO_PORT = 4212;
	public static final String SETTINGS_FILENAME = "settings.properties";
	
	private String mCopterIp;
	private int mCopterCmdPort;
	private int mCopterTelemetryPort;
	private int mCopterVideoPort;
	private int mAccelXOffset;
	private int mAccelYOffset;
	private int mAccelZOffset;
	private int mGyroXOffset;
	private int mGyroYOffset;
	private int mGyroZOffset;
	private int mMagnetXOffset;
	private int mMagnetYOffset;
	private int mMagnetZOffset;
	private float mMagnetXScale;
	private float mMagnetYScale;
	private float mMagnetZScale;
	private boolean mYawPidEnabled;
	private float mYawPidKp;
	private float mYawPidKi;
	private float mYawPidKd;
	private boolean mPitchPidEnabled;
	private float mPitchPidKp;
	private float mPitchPidKi;
	private float mPitchPidKd;
	private boolean mRollPidEnabled;
	private float mRollPidKp;
	private float mRollPidKi;
	private float mRollPidKd;
	private boolean mAltPidEnabled;
	private float mAltPidKp;
	private float mAltPidKi;
	private float mAltPidKd;
	
	
	private Settings()
	{
		setDefault();
	}
	
	public void save()
	{
		Properties prop = new Properties();
		
		prop.setProperty("Language", Locale.getDefault().getLanguage());
		prop.setProperty("CopterIp", mCopterIp);
		prop.setProperty("CopterCmdPort", Integer.toString(mCopterCmdPort));
		prop.setProperty("CopterTelemetryPort", Integer.toString(mCopterTelemetryPort));
		prop.setProperty("CopterVideoPort", Integer.toString(mCopterVideoPort));
		prop.setProperty("AccelXOffset", Integer.toString(mAccelXOffset));
		prop.setProperty("AccelYOffset", Integer.toString(mAccelYOffset));
		prop.setProperty("AccelZOffset", Integer.toString(mAccelZOffset));
		prop.setProperty("GyroXOffset", Integer.toString(mGyroXOffset));
		prop.setProperty("GyroYOffset", Integer.toString(mGyroYOffset));
		prop.setProperty("GyroZOffset", Integer.toString(mGyroZOffset));
		prop.setProperty("MagnetXOffset", Integer.toString(mMagnetXOffset));
		prop.setProperty("MagnetYOffset", Integer.toString(mMagnetYOffset));
		prop.setProperty("MagnetZOffset", Integer.toString(mMagnetZOffset));
		prop.setProperty("MagnetXScale", Double.toString(mMagnetXScale));
		prop.setProperty("MagnetYScale", Double.toString(mMagnetYScale));
		prop.setProperty("MagnetZScale", Double.toString(mMagnetZScale));
		prop.setProperty("YawPidEnabled", Boolean.toString(mYawPidEnabled));
		prop.setProperty("PitchPidEnabled", Boolean.toString(mPitchPidEnabled));
		prop.setProperty("RollPidEnabled", Boolean.toString(mRollPidEnabled));
		prop.setProperty("AltPidEnabled", Boolean.toString(mAltPidEnabled));
		prop.setProperty("YawPidKp", Float.toString(mYawPidKp));
		prop.setProperty("PitchPidKp", Float.toString(mPitchPidKp));
		prop.setProperty("RollPidKp", Float.toString(mRollPidKp));
		prop.setProperty("AltPidKp", Float.toString(mAltPidKp));
		prop.setProperty("YawPidKi", Float.toString(mYawPidKi));
		prop.setProperty("PitchPidKi", Float.toString(mPitchPidKi));
		prop.setProperty("RollPidKi", Float.toString(mRollPidKi));
		prop.setProperty("AltPidKi", Float.toString(mAltPidKi));
		prop.setProperty("YawPidKd", Float.toString(mYawPidKd));
		prop.setProperty("PitchPidKd", Float.toString(mPitchPidKd));
		prop.setProperty("RollPidKd", Float.toString(mRollPidKd));
		prop.setProperty("AltPidKd", Float.toString(mAltPidKd));
		
		try
		{
			prop.store(new FileOutputStream(SETTINGS_FILENAME), null);
		}
		catch(FileNotFoundException e)
		{
			e.printStackTrace();
		}
		catch(IOException e)
		{
			e.printStackTrace();
		}
	}
	
	public void setDefault()
	{
		mCopterIp = COPTER_DEFAULT_IP;
		mCopterCmdPort = COPTER_DEFAULT_CMD_PORT;
		mCopterTelemetryPort = COPTER_DEFAULT_TELEMETRY_PORT;
		mCopterVideoPort = COPTER_DEFAULT_VIDEO_PORT;
	}
	
	public void load()
	{
		setDefault();
		
		Properties prop = new Properties();
		
		try
		{
			prop.load(new FileInputStream(SETTINGS_FILENAME));
			
			String lang = prop.getProperty("Language","en");
			Locale locale = new Locale(lang);
			Locale.setDefault(locale);
			
			mCopterIp = prop.getProperty("CopterIp", mCopterIp);
			
			String value = prop.getProperty("CopterCmdPort", Integer.toString(mCopterCmdPort));
			mCopterCmdPort = Integer.parseInt(value);
			
			value = prop.getProperty("CopterTelemetryPort", Integer.toString(mCopterTelemetryPort));
			mCopterTelemetryPort = Integer.parseInt(value);
			
			value = prop.getProperty("CopterVideoPort", Integer.toString(mCopterVideoPort));
			mCopterVideoPort = Integer.parseInt(value);
			
			value = prop.getProperty("AccelXOffset", Integer.toString(0));
			mAccelXOffset = Integer.parseInt(value);

			value = prop.getProperty("AccelYOffset", Integer.toString(0));
			mAccelYOffset = Integer.parseInt(value);
			
			value = prop.getProperty("AccelZOffset", Integer.toString(0));
			mAccelZOffset = Integer.parseInt(value);
			
			value = prop.getProperty("GyroXOffset", Integer.toString(0));
			mGyroXOffset = Integer.parseInt(value);
			
			value = prop.getProperty("GyroYOffset", Integer.toString(0));
			mGyroYOffset = Integer.parseInt(value);
			
			value = prop.getProperty("GyroZOffset", Integer.toString(0));
			mGyroZOffset = Integer.parseInt(value);
			
			value = prop.getProperty("MagnetXOffset", Integer.toString(0));
			mMagnetXOffset = Integer.parseInt(value);
			
			value = prop.getProperty("MagnetYOffset", Integer.toString(0));
			mMagnetYOffset = Integer.parseInt(value);
			
			value = prop.getProperty("MagnetZOffset", Integer.toString(0));
			mMagnetZOffset = Integer.parseInt(value);
			
			value = prop.getProperty("MagnetXScale", Float.toString(1.f));
			mMagnetXScale = Float.parseFloat(value);
			
			value = prop.getProperty("MagnetYScale", Float.toString(1.f));
			mMagnetYScale = Float.parseFloat(value);
			
			value = prop.getProperty("MagnetZScale", Double.toString(1.f));
			mMagnetZScale = Float.parseFloat(value);
			
			value = prop.getProperty("YawPidEnabled", Boolean.toString(false));
			mYawPidEnabled = Boolean.parseBoolean(value);
			
			value = prop.getProperty("PitchPidEnabled", Boolean.toString(false));
			mPitchPidEnabled = Boolean.parseBoolean(value);
			
			value = prop.getProperty("RollPidEnabled", Boolean.toString(false));
			mRollPidEnabled = Boolean.parseBoolean(value);
			
			value = prop.getProperty("AltPidEnabled", Boolean.toString(false));
			mAltPidEnabled = Boolean.parseBoolean(value);
			
			value = prop.getProperty("YawPidKp", Float.toString(0));
			mYawPidKp = Float.parseFloat(value);
			
			value = prop.getProperty("PitchPidKp", Float.toString(0));
			mPitchPidKp = Float.parseFloat(value);
			
			value = prop.getProperty("RollPidKp", Float.toString(0));
			mRollPidKp = Float.parseFloat(value);
			
			value = prop.getProperty("AltPidKp", Float.toString(0));
			mAltPidKp = Float.parseFloat(value);
			
			value = prop.getProperty("YawPidKi", Float.toString(0));
			mYawPidKi = Float.parseFloat(value);
			
			value = prop.getProperty("PitchPidKi", Float.toString(0));
			mPitchPidKi = Float.parseFloat(value);
			
			value = prop.getProperty("RollPidKi", Float.toString(0));
			mRollPidKi = Float.parseFloat(value);
			
			value = prop.getProperty("AltPidKi", Float.toString(0));
			mAltPidKi = Float.parseFloat(value);
			
			value = prop.getProperty("YawPidKd", Float.toString(0));
			mYawPidKd = Float.parseFloat(value);
			
			value = prop.getProperty("PitchPidKd", Float.toString(0));
			mPitchPidKd = Float.parseFloat(value);
			
			value = prop.getProperty("RollPidKd", Float.toString(0));
			mRollPidKd = Float.parseFloat(value);
			
			value = prop.getProperty("AltPidKd", Float.toString(0));
			mAltPidKd = Float.parseFloat(value);
		}
		catch (FileNotFoundException e)
		{
			e.printStackTrace();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}		
	}

	public String getCopterIp()
	{
		return mCopterIp;
	}

	public void setCopterIp(String ip)
	{
		this.mCopterIp = ip;
	}

	public int getCopterCmdPort()
	{
		return mCopterCmdPort;
	}

	public void setCopterCmdPort(int port)
	{
		this.mCopterCmdPort = port;
	}

	public int getCopterTelemetryPort()
	{
		return mCopterTelemetryPort;
	}

	public void setCopterTelemetryPort(int port)
	{
		this.mCopterTelemetryPort = port;
	}

	public int getCopterVideoPort()
	{
		return mCopterVideoPort;
	}

	public void setCopterVideoPort(int port)
	{
		this.mCopterVideoPort = port;
	}

	public int getAccelXOffset()
	{
		return mAccelXOffset;
	}

	public void setAccelXOffset(int offset)
	{
		this.mAccelXOffset = offset;
	}

	public int getAccelYOffset()
	{
		return mAccelYOffset;
	}

	public void setAccelYOffset(int offset)
	{
		this.mAccelYOffset = offset;
	}

	public int getAccelZOffset()
	{
		return mAccelZOffset;
	}

	public void setAccelZOffset(int offset)
	{
		this.mAccelZOffset = offset;
	}

	public int getGyroXOffset()
	{
		return mGyroXOffset;
	}

	public void setGyroXOffset(int offset)
	{
		this.mGyroXOffset = offset;
	}

	public int getGyroYOffset()
	{
		return mGyroYOffset;
	}

	public void setGyroYOffset(int offset)
	{
		this.mGyroYOffset = offset;
	}

	public int getGyroZOffset()
	{
		return mGyroZOffset;
	}

	public void setGyroZOffset(int offset)
	{
		this.mGyroZOffset = offset;
	}

	public int getMagnetXOffset()
	{
		return mMagnetXOffset;
	}

	public void setMagnetXOffset(int offset)
	{
		this.mMagnetXOffset = offset;
	}

	public int getMagnetYOffset()
	{
		return mMagnetYOffset;
	}

	public void setMagnetYOffset(int offset)
	{
		this.mMagnetYOffset = offset;
	}

	public int getMagnetZOffset()
	{
		return mMagnetZOffset;
	}

	public void setMagnetZOffset(int offset)
	{
		this.mMagnetZOffset = offset;
	}

	public float getMagnetXScale()
	{
		return mMagnetXScale;
	}

	public void setMagnetXScale(float scale)
	{
		this.mMagnetXScale = scale;
	}

	public float getMagnetYScale()
	{
		return mMagnetYScale;
	}

	public void setMagnetYScale(float scale)
	{
		this.mMagnetYScale = scale;
	}

	public float getMagnetZScale()
	{
		return mMagnetZScale;
	}

	public void setMagnetZScale(float scale)
	{
		this.mMagnetZScale = scale;
	}

	public boolean getYawPidEnabled()
	{
		return mYawPidEnabled;
	}

	public void setYawPidEnabled(boolean enabled)
	{
		this.mYawPidEnabled = enabled;
	}

	public float getYawPidKp()
	{
		return mYawPidKp;
	}

	public void setYawPidKp(float kp)
	{
		this.mYawPidKp = kp;
	}

	public float getYawPidKi()
	{
		return mYawPidKi;
	}

	public void setYawPidKi(float ki)
	{
		this.mYawPidKi = ki;
	}

	public float getYawPidKd()
	{
		return mYawPidKd;
	}

	public void setYawPidKd(float kd)
	{
		this.mYawPidKd = kd;
	}

	public boolean getPitchPidEnabled()
	{
		return mPitchPidEnabled;
	}

	public void setPitchPidEnabled(boolean enabled)
	{
		this.mPitchPidEnabled = enabled;
	}

	public float getPitchPidKp()
	{
		return mPitchPidKp;
	}

	public void setPitchPidKp(float kp)
	{
		this.mPitchPidKp = kp;
	}

	public float getPitchPidKi()
	{
		return mPitchPidKi;
	}

	public void setPitchPidKi(float ki)
	{
		this.mPitchPidKi = ki;
	}

	public float getPitchPidKd()
	{
		return mPitchPidKd;
	}

	public void setPitchPidKd(float kd)
	{
		this.mPitchPidKd = kd;
	}

	public boolean getRollPidEnabled()
	{
		return mRollPidEnabled;
	}

	public void setRollPidEnabled(boolean enabled)
	{
		this.mRollPidEnabled = enabled;
	}

	public float getRollPidKp()
	{
		return mRollPidKp;
	}

	public void setRollPidKp(float kp)
	{
		this.mRollPidKp = kp;
	}

	public float getRollPidKi()
	{
		return mRollPidKi;
	}

	public void setRollPidKi(float ki)
	{
		this.mRollPidKi = ki;
	}

	public float getRollPidKd()
	{
		return mRollPidKd;
	}

	public void setRollPidKd(float kd)
	{
		this.mRollPidKd = kd;
	}

	public boolean getAltPidEnabled()
	{
		return mAltPidEnabled;
	}

	public void setAltPidEnabled(boolean enabled)
	{
		this.mAltPidEnabled = enabled;
	}

	public float getAltPidKp()
	{
		return mAltPidKp;
	}

	public void setAltPidKp(float kp)
	{
		this.mAltPidKp = kp;
	}

	public float getAltPidKi()
	{
		return mAltPidKi;
	}

	public void setAltPidKi(float ki)
	{
		this.mAltPidKi = ki;
	}

	public float getAltPidKd()
	{
		return mAltPidKd;
	}

	public void setAltPidKd(float kd)
	{
		this.mAltPidKd = kd;
	}
}
