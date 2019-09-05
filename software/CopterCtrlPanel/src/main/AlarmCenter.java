package main;

import java.util.Observable;
import java.util.SortedSet;
import java.util.TreeSet;


public class AlarmCenter extends Observable
{
	private static AlarmCenter mSingleton;
	
	public static AlarmCenter instance()
	{
		if(mSingleton == null)
			mSingleton = new AlarmCenter();
		
		return mSingleton;
	}
	
	private SortedSet<Alarm> mAlarms;
	private Object mMapMutex;
	
	private AlarmCenter()
	{
		mAlarms = new TreeSet<Alarm>();
		mMapMutex = new Object();
	}
	
	public void setAlarm(Alarm alarm)
	{
		synchronized(mMapMutex)
		{
			if(mAlarms.add(alarm))
			{
				this.setChanged();
				this.notifyObservers();
				this.clearChanged();
			}
		}
	}
	
	public void clearAlarm(Alarm alarm)
	{
		synchronized(mMapMutex)
		{
			if(mAlarms.remove(alarm))
			{
				this.setChanged();
				this.notifyObservers();
				this.clearChanged();
			}
		}
	}
	
	public String getAlarmText()
	{
		Alarm alarm = null;
		String text = Text.get("SYSTEM_OK");
		
		synchronized(mMapMutex)
		{
			try
			{
				alarm = mAlarms.first();
			}
			catch(Exception e)
			{
				// nothing
			}
		}
		
		if(alarm != null)
			text = Text.get(alarm.name());
		
		return text;
	}
}
