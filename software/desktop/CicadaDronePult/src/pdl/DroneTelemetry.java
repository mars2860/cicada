package pdl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.util.Deque;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedDeque;

public class DroneTelemetry extends java.util.Observable implements Runnable
{
	private static DroneTelemetry mSingleton;
	
	public static DroneTelemetry instance()
	{
		if(mSingleton == null)
			mSingleton = new DroneTelemetry();
		
		return mSingleton;
	}
	
	public static final int TIMEOUT = 3000;
	public static final int DRONE_MAX_PACKET_SIZE = 1096;
	
	private DroneState mDroneState = new DroneState();
	private int mBlackBoxSize;
	private Deque<DroneState> mBlackBox;
	//private static final int MAX_BLACK_BOX_SIZE = 64000;	// about 18 Mb
	private int mDroneTelemetryPort;
	private InetAddress mDroneInetAddress;
	private DatagramSocket mSocket;
	private Thread mTelemetryThread;
	private boolean mTelemetryRun;
	private Object objTelemetrySync;
	private Object objDataSync;
	private boolean mDroneConnected;
	private double mFlyTime;
	private double oldTimestamp;

	private DroneTelemetry()
	{
		objTelemetrySync = new Object();
		objDataSync = new Object();
		mBlackBox = new ConcurrentLinkedDeque<DroneState>();
		mBlackBoxSize = 0;
	}
	
	public double getFlyTime()
	{
		return mFlyTime;
	}
	
	public void resetFlyTime()
	{
		mFlyTime = 0;
	}
	
	/** Start new thread to receive telemetry packets. If thread is started it will be restarted */
	public void start(String ip, int port) throws UnknownHostException, SocketException
	{
		if(mTelemetryRun || mSocket != null)
			this.stop();
		
		mDroneTelemetryPort = port;
		mDroneInetAddress = InetAddress.getByName(ip);
		mSocket = new DatagramSocket(mDroneTelemetryPort);
		mSocket.setSoTimeout(TIMEOUT);
		mTelemetryRun = true;
		mTelemetryThread = new Thread(this);
		mTelemetryThread.setName("DroneTelemetryReceiver");
		mTelemetryThread.start();
	}
	
	/** Stops telemetry service. But It doesn't clear observers list and doesn't clear Black Box! */
	public void stop()
	{
		mTelemetryRun = false;
		mDroneConnected = false;
		
		try
		{
			synchronized(objTelemetrySync)
			{
				// Unblock telemetry thread
				objTelemetrySync.notify();
				//System.out.println("Wait until telemetry thread stops");
				// Wait until thread stops its work
				if(mTelemetryThread != null && mTelemetryThread.isAlive())
				{
					objTelemetrySync.wait();
				}
				//System.out.println("End to wait for telemetry thread stop");
			}
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
		
		if(mSocket != null)
		{
			mSocket.close();
			mSocket = null;
		}

		DroneAlarmCenter.instance().clearAll();
	}
	
	public boolean isDroneConnected()
	{
		return mDroneConnected;
	}
	
	public DroneState getDroneState()
	{
		DroneState state = new DroneState();
		
		synchronized(objDataSync)
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
	
	public int getTelemetryPeriod()
	{
		int result;
		
		synchronized(objDataSync)
		{
			result = DroneState.net.telemetryPeriod;
		}
		
		return result;
	}
	
	public long droneStateSize;

	@Override
	public void run()
	{	
		byte[] receivedData = new byte[DRONE_MAX_PACKET_SIZE];
		
		while(mTelemetryRun)
		{
			DatagramPacket receivedPacket = new DatagramPacket(receivedData, receivedData.length);
			
			try
			{
				mSocket.receive(receivedPacket);
				
				if(receivedPacket.getAddress().equals(mDroneInetAddress))
				{
					DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_DRONE_NOT_FOUND);
					DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_RECEIVE_ERROR);

					synchronized(objDataSync)
					{
						mDroneConnected = true;
						BinaryParser parser = new BinaryParser();
						droneStateSize = parser.getUint32t(receivedPacket);
						mDroneState = new DroneState();
						mDroneState.parse(parser, receivedPacket);
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
			}
			catch(SocketTimeoutException e)
			{
				mDroneConnected = false;
				DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DRONE_NOT_FOUND);
			}
			catch(IOException e)
			{
				DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_RECEIVE_ERROR);
				e.printStackTrace();
			}
		}
		
		//System.out.println("Telemetry thread finish its work");
		
		synchronized(objTelemetrySync)
		{
			//System.out.println("Notify telemetry thread is stopped");
			objTelemetrySync.notify();
		}
	}
	
	protected void updateFlyTime()
	{
		if(mDroneState.baseGas > (DroneState.Motors.maxGas - DroneState.Motors.minGas) / 8)
		{
			mFlyTime += mDroneState.timestamp - oldTimestamp;
		}
		
		oldTimestamp = mDroneState.timestamp;
	}
	
	protected void checkDroneStateForAlarms(DroneState ds)
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
		
		if(ds.rssi <= DroneAlarmCenter.WIFI_LOW_LEVEL)
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_LOW_RADIO_SIGNAL);
		}
		else if(ds.rssi >= DroneAlarmCenter.WIFI_LOW_LEVEL + 4.0)
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
		int telemetryPeriod = DroneState.net.telemetryPeriod;
		
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
		
		return result; 
	}
	
	public void clearBlackBox()
	{
		synchronized(objDataSync)
		{
			mBlackBox.clear();
			mBlackBoxSize = 0;
		}
		
		this.setChanged();
		this.notifyObservers();
		this.clearChanged();
	}

}
