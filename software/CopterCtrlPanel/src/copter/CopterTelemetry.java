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
	private float mYaw;
	private float mPitch;
	private float mRoll;
	private float mHeading;
	private int mYawPidEnabled;
	private float mYawPidKp;
	private float mYawPidKi;
	private float mYawPidKd;
	private float mYawPidTarget;
	private int mPitchPidEnabled;
	private float mPitchPidKp;
	private float mPitchPidKi;
	private float mPitchPidKd;
	private float mPitchPidTarget;
	private int mRollPidEnabled;
	private float mRollPidKp;
	private float mRollPidKi;
	private float mRollPidKd;
	private float mRollPidTarget;
	private int mAltPidEnabled;
	private float mAltPidKp;
	private float mAltPidKi;
	private float mAltPidKd;
	private float mAltPidTarget;
	private int mMotorsEnabled;
	private int mMotorGas0;
	private int mMotorGas1;
	private int mMotorGas2;
	private int mMotorGas3;
	private int mTelemetryPeriod;
	private int mPidPeriod;
	private int mStabilizationEnabled;
	private float mYawPidOutput;
	private float mPitchPidOutput;
	private float mRollPidOutput;
	private float mAltPidOutput;
	private int mLoopTime;
	private float mTemperature;
	private float mPressure;
	private float mAltitude;
	private float mSeaLevelPressure;
	
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
	
	public float getYaw()
	{
		float result = 0;
		
		synchronized(objDataSync)
		{
			result = (float)Math.toDegrees(mYaw);
		}
		
		return result;
	}
	
	public float getPitch()
	{
		float result = 0;
		
		synchronized(objDataSync)
		{
			result = (float)Math.toDegrees(mPitch);
		}
		
		return result;
	}
	
	public float getRoll()
	{
		float result = 0;
		
		synchronized(objDataSync)
		{
			result = (float)Math.toDegrees(mRoll);
		}
		
		return result;
	}
	
	public float getHeading()
	{
		float result = 0;
		
		synchronized(objDataSync)
		{
			result = (float)Math.toDegrees(mHeading);
		}
		
		return result;
	}
	
	public boolean getYawPidEnabled()
	{
		boolean result;
		
		synchronized(objDataSync)
		{
			if(mYawPidEnabled > 0)
				result = true;
			else
				result = false;
		}
		
		return result;
	}
	
	public boolean getPitchPidEnabled()
	{
		boolean result;
		
		synchronized(objDataSync)
		{
			if(mPitchPidEnabled > 0)
				result = true;
			else
				result = false;
		}
		
		return result;
	}
	
	public boolean getRollPidEnabled()
	{
		boolean result;
		
		synchronized(objDataSync)
		{
			if(mRollPidEnabled > 0)
				result = true;
			else
				result = false;
		}
		
		return result;
	}
	
	public boolean getAltPidEnabled()
	{
		boolean result;
		
		synchronized(objDataSync)
		{
			if(mAltPidEnabled > 0)
				result = true;
			else
				result = false;
		}
		
		return result;
	}
	
	public float getYawPidKp()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mYawPidKp;
		}
		
		return result;
	}
	
	public float getPitchPidKp()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mPitchPidKp;
		}
		
		return result;
	}
	
	public float getRollPidKp()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mRollPidKp;
		}
		
		return result;
	}
	
	public float getAltPidKp()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mAltPidKp;
		}
		
		return result;
	}
	
	public float getYawPidKi()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mYawPidKi;
		}
		
		return result;
	}
	
	public float getPitchPidKi()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mPitchPidKi;
		}
		
		return result;
	}
	
	public float getRollPidKi()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mRollPidKi;
		}
		
		return result;
	}
	
	public float getAltPidKi()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mAltPidKi;
		}
		
		return result;
	}
	
	public float getYawPidKd()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mYawPidKd;
		}
		
		return result;
	}
	
	public float getPitchPidKd()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mPitchPidKd;
		}
		
		return result;
	}
	
	public float getRollPidKd()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mRollPidKd;
		}
		
		return result;
	}
	
	public float getAltPidKd()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mAltPidKd;
		}
		
		return result;
	}
	
	public float getYawPidTarget()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = (float)Math.toDegrees(mYawPidTarget);
		}
		
		return result;
	}
	
	public float getPitchPidTarget()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = (float)Math.toDegrees(mPitchPidTarget);
		}
		
		return result;
	}
	
	public float getRollPidTarget()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = (float)Math.toDegrees(mRollPidTarget);
		}
		
		return result;
	}
	
	public float getAltPidTarget()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mAltPidTarget;
		}
		
		return result;
	}
	
	public boolean getMotorsEnabled()
	{
		boolean result;
		
		synchronized(objDataSync)
		{
			if(mMotorsEnabled > 0)
				result = true;
			else
				result = false;
		}
		
		return result;
	}
	
	public int getMotorGas0()
	{
		int result;
		
		synchronized(objDataSync)
		{
			result = mMotorGas0;
		}
		
		return result;
	}
	
	public int getMotorGas1()
	{
		int result;
		
		synchronized(objDataSync)
		{
			result = mMotorGas1;
		}
		
		return result;
	}
	
	public int getMotorGas2()
	{
		int result;
		
		synchronized(objDataSync)
		{
			result = mMotorGas2;
		}
		
		return result;
	}
	
	public int getMotorGas3()
	{
		int result;
		
		synchronized(objDataSync)
		{
			result = mMotorGas3;
		}
		
		return result;
	}
	
	public int getTelemetryPeriod()
	{
		int result;
		
		synchronized(objDataSync)
		{
			result = mTelemetryPeriod;
		}
		
		return result;
	}
	
	public int getPidPeriod()
	{
		int result;
		
		synchronized(objDataSync)
		{
			result = mPidPeriod;
		}
		
		return result;
	}
	
	public boolean getStabilizationEnabled()
	{
		boolean result;
		
		synchronized(objDataSync)
		{
			result = (mStabilizationEnabled > 0)?true:false;
		}
		
		return result;
	}
	
	public float getYawPidOutput()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mYawPidOutput;
		}
		
		return result;
	}
	
	public float getPitchPidOutput()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mPitchPidOutput;
		}
		
		return result;
	}
	
	public float getRollPidOutput()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mRollPidOutput;
		}
		
		return result;
	}
	
	public float getAltPidOutput()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mAltPidOutput;
		}
		
		return result;
	}
	
	public int getLoopTime()
	{
		int result;
		
		synchronized(objDataSync)
		{
			result = mLoopTime;
		}
		
		return result;
	}
	
	public float getTemperature()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mTemperature;
		}
		
		return result;
	}
	
	public float getPressure()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mPressure;
		}
		
		return result;
	}
	
	public float getAltitude()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mAltitude;
		}
		
		return result;
	}
	
	public float getSeaLevelPressure()
	{
		float result;
		
		synchronized(objDataSync)
		{
			result = mSeaLevelPressure;
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
						// Yaw, Pitch, Roll
						mYaw = this.getFloat(receivePacket);
						mPitch = this.getFloat(receivePacket);
						mRoll = this.getFloat(receivePacket);
						// Heading
						mHeading = this.getFloat(receivePacket);
						// Yaw Pid
						mYawPidEnabled = this.getUint8t(receivePacket);
						mYawPidKp = this.getFloat(receivePacket);
						mYawPidKi = this.getFloat(receivePacket);
						mYawPidKd = this.getFloat(receivePacket);
						mYawPidTarget = this.getFloat(receivePacket);
						// Pitch Pid
						mPitchPidEnabled = this.getUint8t(receivePacket);
						mPitchPidKp = this.getFloat(receivePacket);
						mPitchPidKi = this.getFloat(receivePacket);
						mPitchPidKd = this.getFloat(receivePacket);
						mPitchPidTarget = this.getFloat(receivePacket);
						// Roll Pid
						mRollPidEnabled = this.getUint8t(receivePacket);
						mRollPidKp = this.getFloat(receivePacket);
						mRollPidKi = this.getFloat(receivePacket);
						mRollPidKd = this.getFloat(receivePacket);
						mRollPidTarget = this.getFloat(receivePacket);
						// Alt Pid
						mAltPidEnabled = this.getUint8t(receivePacket);
						mAltPidKp = this.getFloat(receivePacket);
						mAltPidKi = this.getFloat(receivePacket);
						mAltPidKd = this.getFloat(receivePacket);
						mAltPidTarget = this.getFloat(receivePacket);
						// Motors
						mMotorsEnabled = this.getUint8t(receivePacket);
						mMotorGas0 = this.getInt32t(receivePacket);
						mMotorGas1 = this.getInt32t(receivePacket);
						mMotorGas2 = this.getInt32t(receivePacket);
						mMotorGas3 = this.getInt32t(receivePacket);
						// Periods
						mTelemetryPeriod = this.getInt32t(receivePacket);
						mPidPeriod = this.getInt32t(receivePacket);
						// Stabilization
						mStabilizationEnabled = this.getUint8t(receivePacket);
						// PIDs output
						mYawPidOutput = this.getFloat(receivePacket);
						mPitchPidOutput = this.getFloat(receivePacket);
						mRollPidOutput = this.getFloat(receivePacket);
						mAltPidOutput = this.getFloat(receivePacket);
						// Loop time
						mLoopTime = this.getInt32t(receivePacket);
						// Baro
						mTemperature = this.getFloat(receivePacket);
						mPressure = this.getFloat(receivePacket);
						mAltitude = this.getFloat(receivePacket);
						mSeaLevelPressure = this.getFloat(receivePacket);
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
	
	private int getUint8t(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		
		return result;
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
