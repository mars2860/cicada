package copter;

import java.util.Observable;
import java.util.SortedSet;
import java.util.TreeSet;

import main.ResBox;


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
	private Object mMutex;
	
	private AlarmCenter()
	{
		mAlarms = new TreeSet<Alarm>();
		mMutex = new Object();
	}
	
	public void setAlarm(Alarm alarm)
	{
		synchronized(mMutex)
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
		synchronized(mMutex)
		{
			if(mAlarms.remove(alarm))
			{
				this.setChanged();
				this.notifyObservers();
				this.clearChanged();
			}
		}
	}
	
	public int getAlarmLevel()
	{
		int level = 0;
		
		synchronized(mMutex)
		{
			if(mAlarms.isEmpty() == false)
				level = 1;
		}
		
		return level;
	}
	
	public String getAlarmText()
	{
		Alarm alarm = null;
		String text = ResBox.text("SYSTEM_OK");
		
		synchronized(mMutex)
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
			text = ResBox.text(alarm.name());
		
		return text;
	}
}
