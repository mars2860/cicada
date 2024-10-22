package pdl;

import java.util.Observable;
import java.util.SortedSet;
import java.util.TreeSet;

public class DroneAlarmCenter extends Observable
{
	private static DroneAlarmCenter mSingleton;
	
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
		boolean result;
		synchronized(mMutex)
		{
			result = mAlarms.add(alarm);
		}
		if(result)
		{
			this.setChanged();
			this.notifyObservers();
			this.clearChanged();
		}
	}
	
	public void clearAlarm(Alarm alarm)
	{
		boolean result;
		synchronized(mMutex)
		{
			result = mAlarms.remove(alarm);
		}
		if(result)
		{
			this.setChanged();
			this.notifyObservers();
			this.clearChanged();
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

	public void clearAll()
	{
		synchronized(mMutex)
		{
			mAlarms.clear();
			this.setChanged();
			this.notifyObservers();
			this.clearChanged();
		}
	}
	
	public void clearModemAlarms()
	{
		clearAlarm(Alarm.ALARM_CANT_OPEN_COM_PORT);
		clearAlarm(Alarm.ALARM_RECEIVE_ERROR);
		clearAlarm(Alarm.ALARM_SEND_ERROR);
		clearAlarm(Alarm.ALARM_SEND_SETUP_ERROR);
		clearAlarm(Alarm.ALARM_UDP_MODEM_CONNECTION_ERROR);
		clearAlarm(Alarm.ALARM_WIFI_BROADCAST_MODEM_NOT_FOUND);
	}
}
