package pdl;

import java.util.Observable;
import java.util.SortedSet;
import java.util.TreeSet;

public class DroneAlarmCenter extends Observable
{
	private static DroneAlarmCenter mSingleton;
	
	public static final double WIFI_LOW_LEVEL = -82.0;
	public static final double BATTERY_LOW_LEVEL = 15.0;
	public static final double BATTERY_CRITICAL_LOW_LEVEL = 5;
	
	public static DroneAlarmCenter instance()
	{
		if(mSingleton == null)
			mSingleton = new DroneAlarmCenter();
		
		return mSingleton;
	}
	
	private SortedSet<Alarm> mAlarms;
	private Object mMutex;
	
	private DroneAlarmCenter()
	{
		mAlarms = new TreeSet<Alarm>();
		mMutex = new Object();
	}
	
	public void setAlarm(Alarm alarm)
	{
		synchronized(mMutex)
		{
			//if(mAlarms.add(alarm))
			// notify always to play alarm sounds in loop
			mAlarms.add(alarm);
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
	
	public Alarm getAlarm()
	{
		Alarm alarm = null;

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

		return alarm;
	}

	public boolean getAlarm(Alarm alarm)
	{
		boolean result = false;
		synchronized(mMutex)
		{
			try
			{
				result = mAlarms.contains(alarm);
			}
			catch(Exception e)
			{
				// nothing
			}
		}

		return result;
	}

	void clearAll()
	{
		synchronized(mMutex)
		{
			mAlarms.clear();
			this.setChanged();
			this.notifyObservers();
			this.clearChanged();
		}
	}
}
