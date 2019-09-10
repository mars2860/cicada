package copter;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;

public class CopterTelemetry extends java.util.Observable implements Runnable
{
	private static CopterTelemetry mSingleton;
	
	public static CopterTelemetry instance()
	{
		if(mSingleton == null)
			mSingleton = new CopterTelemetry();
		
		return mSingleton;
	}
	
	public static final int TIMEOUT = 3000;
	
	private int mCopterTelemetryPort;
	private InetAddress mCopterInetAddress;
	private DatagramSocket mSocket;
	private Thread mTelemetryThread;
	private boolean mTelemetryRun;
	private Object objTelemetrySync;
	private Object objDataSync;
	
	private float mBatteryVoltage;
	private float mBatteryPercent;
	
	private CopterTelemetry()
	{
		objTelemetrySync = new Object();
		objDataSync = new Object();
	}
	
	public void start(String ip, int port) throws UnknownHostException, SocketException
	{
		if(mSocket != null)
			this.stop();
		
		mCopterTelemetryPort = port;
		mCopterInetAddress = InetAddress.getByName(ip);
		mSocket = new DatagramSocket(mCopterTelemetryPort);
		mSocket.setSoTimeout(TIMEOUT);
		mTelemetryRun = true;
		mTelemetryThread = new Thread(this);
		mTelemetryThread.setName("CopterTelemetryReceiver");
		mTelemetryThread.start();
	}
	
	public void stop()
	{
		mTelemetryRun = false;
		
		//System.out.println("Telemetry thread has started");
		
		try
		{
			synchronized(objTelemetrySync)
			{
				// Unblock telemetry thread
				objTelemetrySync.notify();
				//System.out.println("Wait until telemetry thread stops");
				// Wait until thread stops its work
				if(mTelemetryThread != null && mTelemetryThread.isAlive())
					objTelemetrySync.wait();
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
		}
		
		AlarmCenter.instance().clearAlarm(Alarm.COPTER_NOT_FOUND);
		AlarmCenter.instance().clearAlarm(Alarm.COPTER_RECEIVE_ERROR);
	}
	
	public float getBatteryVoltage()
	{
		float voltage = 0;
		
		synchronized(objDataSync)
		{
			voltage = mBatteryVoltage;
		}
		
		return voltage;
	}
	
	public float getBatteryPercent()
	{
		float percent = 0;
		
		synchronized(objDataSync)
		{
			percent = mBatteryPercent;
		}
		
		return percent;
	}

	@Override
	public void run()
	{	
		byte[] receiveData = new byte[1024];
		
		while(mTelemetryRun)
		{
			DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
			
			try
			{
				mSocket.receive(receivePacket);
				
				if(receivePacket.getAddress().equals(mCopterInetAddress))
				{
					AlarmCenter.instance().clearAlarm(Alarm.COPTER_NOT_FOUND);
					AlarmCenter.instance().clearAlarm(Alarm.COPTER_RECEIVE_ERROR);
					
					int pos = 0;
					// Battery voltage
					int lb = receivePacket.getData()[pos++] & 0xFF;
					int hb = receivePacket.getData()[pos++] & 0xFF;
					synchronized(objDataSync)
					{
						mBatteryVoltage = ((float)((hb << 8) | lb)) * 0.00125f;
					}
					// Battery percent
					lb = receivePacket.getData()[pos++] & 0xFF;
					hb = receivePacket.getData()[pos++] & 0xFF;
					synchronized(objDataSync)
					{
						mBatteryPercent = (float)(hb) + ((float)(lb))/256.f;
					}
					
					this.setChanged();
					this.notifyObservers();
					this.clearChanged();
				}
			}
			catch(SocketTimeoutException e)
			{
				AlarmCenter.instance().setAlarm(Alarm.COPTER_NOT_FOUND);
			}
			catch(IOException e)
			{
				AlarmCenter.instance().setAlarm(Alarm.COPTER_RECEIVE_ERROR);
				e.printStackTrace();
			}
		}
		
		synchronized(objTelemetrySync)
		{
			//System.out.println("Notify telemetry thread is stopped");
			objTelemetrySync.notify();
		}
	}
}
