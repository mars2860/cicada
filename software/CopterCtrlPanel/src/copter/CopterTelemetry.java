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
	
	private int mParsePos;
	
	private float mBatteryVoltage;
	private float mBatteryPercent;
	private int mWifiLevel;
	private int mAccelX;
	private int mAccelY;
	private int mAccelZ;
	private int mGyroX;
	private int mGyroY;
	private int mGyroZ;
	private int mMagnetX;
	private int mMagnetY;
	private int mMagnetZ;
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
	
	public int getAccelX()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mAccelX;
		}
		
		return result;
	}
	
	public int getAccelY()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mAccelY;
		}
		
		return result;
	}
	
	public int getAccelZ()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mAccelZ;
		}
		
		return result;
	}
	
	public int getGyroX()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mGyroX;
		}
		
		return result;
	}
	
	public int getGyroY()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mGyroY;
		}
		
		return result;
	}
	
	public int getGyroZ()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mGyroZ;
		}
		
		return result;
	}
	
	public int getWifiLevel()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mWifiLevel;
		}
		
		return result;
	}
	
	public int getMagnetX()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mMagnetX;
		}
		
		return result;
	}
	
	public int getMagnetY()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mMagnetY;
		}
		
		return result;
	}
	
	public int getMagnetZ()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mMagnetZ;
		}
		
		return result;
	}
	
	public int getAccelXOffset()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mAccelXOffset;
		}
		
		return result;
	}
	
	public int getAccelYOffset()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mAccelYOffset;
		}
		
		return result;
	}
	
	public int getAccelZOffset()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mAccelZOffset;
		}
		
		return result;
	}
	
	public int getGyroXOffset()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mGyroXOffset;
		}
		
		return result;
	}
	
	public int getGyroYOffset()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mGyroYOffset;
		}
		
		return result;
	}
	
	public int getGyroZOffset()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mGyroZOffset;
		}
		
		return result;
	}
	
	public int getMagnetXOffset()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mMagnetXOffset;
		}
		
		return result;
	}
	
	public int getMagnetYOffset()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mMagnetYOffset;
		}
		
		return result;
	}
	
	public int getMagnetZOffset()
	{
		int result = 0;
		
		synchronized(objDataSync)
		{
			result = mMagnetZOffset;
		}
		
		return result;
	}
	
	public float getMagnetXScale()
	{
		float result = 0;
		
		synchronized(objDataSync)
		{
			result = mMagnetXScale;
		}
		
		return result;
	}
	
	public float getMagnetYScale()
	{
		float result = 0;
		
		synchronized(objDataSync)
		{
			result = mMagnetYScale;
		}
		
		return result;
	}
	
	public float getMagnetZScale()
	{
		float result = 0;
		
		synchronized(objDataSync)
		{
			result = mMagnetZScale;
		}
		
		return result;
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

					synchronized(objDataSync)
					{
						mParsePos = 0;
						// Battery voltage
						mBatteryVoltage = (float)(this.getUint16t(receivePacket)) * 0.00125f;
						// Battery percent
						int temp = this.getUint16t(receivePacket);
						mBatteryPercent = (float)(temp>>8) + ((float)(temp & 0xFF))/256.f;
						// Wifi level
						mWifiLevel = this.getInt32t(receivePacket);
						// Accel data
						mAccelX = this.getInt16t(receivePacket);
						mAccelY = this.getInt16t(receivePacket);
						mAccelZ = this.getInt16t(receivePacket);
						// Gyro data
						mGyroX = this.getInt16t(receivePacket);
						mGyroY = this.getInt16t(receivePacket);
						mGyroZ = this.getInt16t(receivePacket);
						// Magnet data
						mMagnetX = this.getInt16t(receivePacket);
						mMagnetY = this.getInt16t(receivePacket);
						mMagnetZ = this.getInt16t(receivePacket);
						// Accel offsets
						mAccelXOffset = this.getInt16t(receivePacket);
						mAccelYOffset = this.getInt16t(receivePacket);
						mAccelZOffset = this.getInt16t(receivePacket);
						// Gyro offsets
						mGyroXOffset = this.getInt16t(receivePacket);
						mGyroYOffset = this.getInt16t(receivePacket);
						mGyroZOffset = this.getInt16t(receivePacket);
						// Magnet offsets
						mMagnetXOffset = this.getInt16t(receivePacket);
						mMagnetYOffset = this.getInt16t(receivePacket);
						mMagnetZOffset = this.getInt16t(receivePacket);
						// Magnet scales
						mMagnetXScale = this.getFloat(receivePacket);
						mMagnetYScale = this.getFloat(receivePacket);
						mMagnetZScale = this.getFloat(receivePacket);
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
	
	private int getInt16t(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 8;
		result <<= 16;
		result >>= 16;
		
		return result;
	}
	
	private int getInt32t(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 8;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 16;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 24;
		
		return result;
	}
	
	private int getUint16t(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 8;
		
		return result;
	}
	
	private float getFloat(DatagramPacket packet)
	{
		float result = 0;
		
		int bits = this.getInt32t(packet);
		result = Float.intBitsToFloat(bits);
		
		return result;
	}
}
