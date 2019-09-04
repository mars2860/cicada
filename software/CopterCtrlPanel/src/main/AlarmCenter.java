package main;

import java.util.SortedSet;
import java.util.TreeSet;

public class AlarmCenter
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
			mAlarms.add(alarm);
		}
	}
	
	public void clearAlarm(Alarm alarm)
	{
		synchronized(mMapMutex)
		{
			mAlarms.remove(alarm);
		}
	}
	
	public String getAlarmText()
	{
		Alarm alarm;
		String text = Text.get("SYSTEM_OK");
		
		synchronized(mMapMutex)
		{
			alarm = mAlarms.first();
		}
		
		if(alarm != null)
			text = Text.get(alarm.name());
		
		return text;
	}

}
