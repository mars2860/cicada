package pdl;

import java.util.Deque;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedDeque;

import pdl.res.SoundProvider;
import pdl.wlan.WlanTelemetryPacket;

public class DroneTelemetry extends java.util.Observable
{
	private static DroneTelemetry mSingleton;
	
	public static DroneTelemetry instance()
	{
		if(mSingleton == null)
			mSingleton = new DroneTelemetry();
		
		return mSingleton;
	}
		
	private DroneState mDroneState = new DroneState();
	private int mBlackBoxSize;
	private Deque<DroneState> mBlackBox;
	//private static final int MAX_BLACK_BOX_SIZE = 64000;	// about 18 Mb
	private double mFlyTime;
	private double oldTimestamp;
	private Object mBlackBoxLock;

	private DroneTelemetry()
	{
		mBlackBox = new ConcurrentLinkedDeque<DroneState>();
		mBlackBoxSize = 0;
		mBlackBoxLock = new Object();
	}
	
	public double getFlyTime()
	{
		return mFlyTime;
	}
	
	public void resetFlyTime()
	{
		mFlyTime = 0;
	}
	
	public boolean isDroneConnected()
	{
		return DroneCommander.instance().isDroneConnected();
	}
	
	/** @return drone connection time in millis */
	public long getDroneConnectionTime()
	{
		return DroneCommander.instance().getDroneConnectionTime();
	}
	
	public DroneState getDroneState()
	{
		DroneState state = new DroneState();
		
		synchronized(mBlackBoxLock)
		{
			try
			{
				state = (DroneState)mDroneState.clone();
			}
			catch (CloneNotSupportedException e)
			{
				e.printStackTrace();
			}
		}
		
		return state;
	}
	
	public long droneStateSize;
	
	public void append(WlanTelemetryPacket packet)
	{
		synchronized(mBlackBoxLock)
		{
			droneStateSize = packet.getDroneStateSize();
			mDroneState = packet.getDroneState();
			mBlackBox.offer(mDroneState);
			mBlackBoxSize++;
			if(mBlackBoxSize >= DroneState.misc.blackBoxSize)
			{
				mBlackBox.poll();
				mBlackBoxSize--;
				if(mBlackBoxSize < 0)
					mBlackBoxSize = 0;
			}
			
			checkDroneStateForAlarms(mDroneState);
			updateFlyTime();
		}

		this.setChanged();
		this.notifyObservers();
		this.clearChanged();
	}
	
	protected void updateFlyTime()
	{
		if(mDroneState.baseGas > (DroneState.Motors.maxGas - DroneState.Motors.minGas) / 8)
		{
			mFlyTime += mDroneState.timestamp - oldTimestamp;
		}
		
		oldTimestamp = mDroneState.timestamp;
	}
	
	public void checkDroneStateForAlarms(DroneState ds)
	{
		int errors = (int)ds.errors;
		if((errors & 0x1) > 0)
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_IMU_NOT_READY);
		}
		else
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_IMU_NOT_READY);
		}
		if(ds.gps.enabled && (errors & 0x2) > 0)
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_MAGNETO_NOT_READY);
		}
		else
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_MAGNETO_NOT_READY);
		}
		if(ds.battery.percent < DroneAlarmCenter.BATTERY_CRITICAL_LOW_LEVEL)
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_LOW_BATTERY);
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_LOW_BATTERY_CRITICAL);
		}
		else if(ds.battery.percent < DroneAlarmCenter.BATTERY_LOW_LEVEL)
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_LOW_BATTERY);
		}
		else 
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_LOW_BATTERY);
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_LOW_BATTERY_CRITICAL);
		}
		
		if(ds.rssi <= DroneState.net.rssiAlarmLevel)
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_LOW_RADIO_SIGNAL);
		}
		else if(ds.rssi >= DroneState.net.rssiAlarmLevel + 4.0)
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_LOW_RADIO_SIGNAL);
		}

		if(ds.velocityZPid.enabled)
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_ENABLE_VELOCITY_Z_PID);
			if(ds.motorsEnabled && !ds.stabilizationEnabled)
			{
				if(Math.abs(ds.velUp) > 0.3)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_VERT_VELO_NOT_NULL);
				}
				else
				{
					DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_VERT_VELO_NOT_NULL);
				}
			}
			else
			{
				DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_VERT_VELO_NOT_NULL);
			}
		}
		else
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_DISABLE_VELOCITY_Z_PID);
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_VERT_VELO_NOT_NULL);
		}
		
		if(ds.altPid.enabled)
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_ENABLE_ALT_PID);
		}
		
		if(ds.gps.enabled)
		{
			if((int)ds.gps.fixType != 3 || (int)ds.gps.numSV < 6)
			{
				DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_GPS_NOT_FIXED);
			}
			else
			{
				DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_GPS_NOT_FIXED);
			}
		}
		
		if(ds.baseGas == 0 && (Math.abs(ds.pitchDeg) > 60 || Math.abs(ds.rollDeg) > 60))
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DANGER_LEVEL);
		}
		else
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_DANGER_LEVEL);
		}
	}
	
	/**
	 * @param range microseconds from now to (now minus range). if 0 then return all
	 * */
	public DroneState[] getBlackBox(int range)
	{
		DroneState[] result = new DroneState[0];
		int telemetryPeriod = this.getDroneState().telemetry.period;
		
		synchronized(mBlackBoxLock)
		{
			if(range > 0 && telemetryPeriod > 0)
			{
				int count = Math.min(range/telemetryPeriod, mBlackBoxSize);

				result = new DroneState[count];
				Iterator<DroneState> iter = mBlackBox.descendingIterator();
			
				while(iter.hasNext() && count > 0)
				{
					result[count - 1] = iter.next();
					count--;
				}
			}
			else
			{
				result = mBlackBox.toArray(new DroneState[0]); 
			}
		}
		
		return result; 
	}
	
	public void clearBlackBox()
	{
		synchronized(mBlackBoxLock)
		{
			mBlackBox.clear();
			mBlackBoxSize = 0;
		}
		
		this.setChanged();
		this.notifyObservers();
		this.clearChanged();
	}
	
	private int pronouncedAlt;
	
	public void speakAltitude(DroneState ds, SoundProvider sp)
	{
		if(ds.altitude < 1.5)
		{
			pronouncedAlt = 0;
		}
		// it annoys when flying indoor
		/*if(ds.altitude < 0.5)
		{
			pronouncedAlt = 0;
		}
			
		if(ds.altitude >= 1.0 && ds.altitude <= 1.5 && pronouncedAlt != 1)
		{
			pronouncedAlt = 1;
			sp.play("ALT_1M");
		}
		if(ds.altitude >= 2.0 && ds.altitude <= 2.5 && pronouncedAlt != 2)
		{
			pronouncedAlt = 2;
			sp.play("ALT_2M");
		}
		else*/if(ds.altitude >= 3.0 && ds.altitude < 4 && pronouncedAlt != 3)
		{
			pronouncedAlt = 3;
			sp.play("ALT_3M");
		}
		else if(ds.altitude >= 5.0 && ds.altitude < 6 && pronouncedAlt != 5)
		{
			pronouncedAlt = 5;
			sp.play("ALT_5M");
		}
		else if(ds.altitude >= 7.0 && ds.altitude < 8 && pronouncedAlt != 7)
		{
			pronouncedAlt = 7;
			sp.play("ALT_7M");
		}
		else if(ds.altitude >= 10.0 && ds.altitude < 13 && pronouncedAlt != 10)
		{
			pronouncedAlt = 10;
			sp.play("ALT_10M");
		}
		else if(ds.altitude >= 15.0 && ds.altitude < 18 && pronouncedAlt != 15)
		{
			pronouncedAlt = 15;
			sp.play("ALT_15M");
		}
		else if(ds.altitude >= 20.0 && ds.altitude < 25 && pronouncedAlt != 20)
		{
			pronouncedAlt = 20;
			sp.play("ALT_20M");
		}
		else if(ds.altitude >= 30.0 && ds.altitude < 35 && pronouncedAlt != 30)
		{
			pronouncedAlt = 30;
			sp.play("ALT_30M");
		}
		else if(ds.altitude >= 40.0 && ds.altitude < 45 && pronouncedAlt != 40)
		{
			pronouncedAlt = 40;
			sp.play("ALT_40M");
		}
		else if(ds.altitude >= 50.0 && ds.altitude < 55 && pronouncedAlt != 50)
		{
			pronouncedAlt = 50;
			sp.play("ALT_50M");
		}
	}
	
	private boolean oldMotorsEnabled = false;
	private boolean oldStabilizationEnabled = false;
	private DroneState.TrickMode oldTrickMode = DroneState.TrickMode.DISABLED;
	private boolean oldVideoState = false;
	
	public void speakSystemState(DroneState ds, SoundProvider sp)
	{
		if(ds.motorsEnabled && !oldMotorsEnabled)
		{
			sp.play("MOTORS_ENABLED");
				
			// play these sounds in other thread with some delay from motors_enabled sound
			// to prevent motors_enabled and these sounds simultaneously
			if(DroneAlarmCenter.instance().getAlarm() != null)
			{
				sp.playLater("SYSTEM_BAD",2000);
			}
			else
			{
				sp.playLater("SYSTEM_OK",2000);
			}	
		}
		else if(!ds.motorsEnabled && oldMotorsEnabled)
		{
			sp.play("MOTORS_DISABLED");
		}
				
		oldMotorsEnabled = ds.motorsEnabled;
			
		if(ds.stabilizationEnabled && !oldStabilizationEnabled)
		{
			sp.play("STABILIZATION_ENABLED");
		}
		else if(!ds.stabilizationEnabled && oldStabilizationEnabled)
		{
			sp.playLater("STABILIZATION_DISABLED",2000);
		}
				
		oldStabilizationEnabled = ds.stabilizationEnabled;
				
		if(ds.trickMode != null)
		{
			if(	ds.trickMode != DroneState.TrickMode.DISABLED &&
				ds.trickMode != oldTrickMode)
			{
				sp.play("TRICK_MODE_ENABLED");
			}
			else if(ds.trickMode == DroneState.TrickMode.DISABLED &&
					ds.trickMode != oldTrickMode)
			{
				sp.play("TRICK_MODE_DISABLED");
			}
			
			oldTrickMode = ds.trickMode;
		}
		
		if(ds.videoState && !oldVideoState)
		{
			sp.play("VIDEO_STARTED");
		}
		else if(!ds.videoState && oldVideoState)
		{
			sp.play("VIDEO_STOPED");
		}
		
		oldVideoState = ds.videoState;
	}
	
	private int pronouncedBat;	// to say bat15 one time
	
	public void speakBatteryState(DroneState ds, SoundProvider sp)
	{
		if(ds.baseGas > DroneState.Motors.maxGas /8 )
		{
			if(	ds.battery.percent > 10.0 &&
				ds.battery.percent <= 15.0 && 
				pronouncedBat != 15)	// we say bat15 one time
			{
				sp.play("BAT15");
				pronouncedBat = 15;
			}
			else if(	ds.battery.percent > 5.0 && 
						ds.battery.percent <= 10.0 &&
						pronouncedBat != 10 )
			{
				sp.play("BAT10");
				pronouncedBat = 10;
			}
			else if(ds.battery.percent <= 5.0 && ds.battery.percent > 1.0)
			{
				sp.play("BAT5");
			}
			else if(ds.battery.percent <= 1.0)
			{
				sp.play("BAT1");
			}
		}
		else
		{
			pronouncedBat = 0;
		}	
	}
	
	private boolean oldDroneConnected = false;
	
	public void speakRadioState(DroneState ds, SoundProvider sp)
	{
		if(DroneAlarmCenter.instance().getAlarm(Alarm.ALARM_LOW_RADIO_SIGNAL))
		{
			sp.play("LOW_RADIO_SIGNAL");
		}
		
		if(DroneTelemetry.instance().isDroneConnected() && !oldDroneConnected)
		{
			sp.play(SoundProvider.DRONE_CONNECTED);
		}
		else if(!DroneTelemetry.instance().isDroneConnected() && oldDroneConnected)
		{
			sp.play(SoundProvider.DRONE_DISCONNECTED);
		}
		
		oldDroneConnected = DroneTelemetry.instance().isDroneConnected();
	}
	
	public void speakDroneState(DroneState ds, SoundProvider sp)
	{
		if(DroneState.sounds.enabled == false)
		{
			return;
		}
		
		if(DroneState.sounds.altitude)
		{
			speakAltitude(ds,sp);
		}
		
		if(DroneState.sounds.system)
		{
			speakSystemState(ds,sp);
		}
				
		if(DroneState.sounds.battery)
		{
			speakBatteryState(ds,sp);
		}
		
		if(DroneState.sounds.radio)
		{
			speakRadioState(ds,sp);
		}
	}
}
