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
	
	public class DroneState implements Cloneable
	{
		public class Battery implements Cloneable
		{
			public float voltage;
			public float percent;
			
			private void parse(DatagramPacket packet)
			{
				voltage = CopterTelemetry.this.getFloat(packet);
				percent = CopterTelemetry.this.getFloat(packet);
			}
			
			@Override
			public Object clone() throws CloneNotSupportedException
			{
				return super.clone();
			}
		}
		
		public class TripleAxisSensor implements Cloneable
		{
			public int raw[] = new int[3];
			public int filtered[] = new int[3];
			public float pure[] = new float[3];
			public int offset[] = new int[3];
			// no sense variable, this is 2 bytes gap in C structure
			protected int gap;
			
			private void parse(DatagramPacket packet)
			{
				for(int i = 0; i < 3; i++)
					pure[i] = CopterTelemetry.this.getFloat(packet);
				
				for(int i = 0; i < 3; i++)
					raw[i] = CopterTelemetry.this.getInt16t(packet);
				
				for(int i = 0; i < 3; i++)
					filtered[i] = CopterTelemetry.this.getInt16t(packet);

				for(int i = 0; i < 3; i++)
					offset[i] = CopterTelemetry.this.getInt16t(packet);
				
				// read gap
				gap = CopterTelemetry.this.getInt16t(packet);
			}
			
			@Override
			public Object clone() throws CloneNotSupportedException
			{
				return super.clone();
			}
		}
		
		public class Pid implements Cloneable
		{
			public boolean enabled;
			public float kp;
			public float ki;
			public float kd;
			public float target;
			public float out;
			public float errSum;
			public float prevErr;
			
			private void parse(DatagramPacket packet)
			{
				kp = CopterTelemetry.this.getFloat(packet);
				ki = CopterTelemetry.this.getFloat(packet);
				kd = CopterTelemetry.this.getFloat(packet);
				target = CopterTelemetry.this.getFloat(packet);
				out = CopterTelemetry.this.getFloat(packet);
				errSum = CopterTelemetry.this.getFloat(packet);
				prevErr = CopterTelemetry.this.getFloat(packet);
				enabled = (CopterTelemetry.this.getUint32t(packet) > 0)?true:false;
			}
			
			@Override
			public Object clone() throws CloneNotSupportedException
			{
				return super.clone();
			}
		}
		
		public long timestamp;
		public Battery battery = new Battery();
		public int wifiLevel;
		public TripleAxisSensor accel = new TripleAxisSensor();
		public TripleAxisSensor gyro = new TripleAxisSensor();
		public TripleAxisSensor magneto = new TripleAxisSensor();
		public float yaw;
		public float pitch;
		public float roll;
		public float heading;
		public Pid yawRatePid = new Pid();
		public Pid pitchPid = new Pid();
		public Pid rollPid = new Pid();
		public Pid altPid = new Pid();
		public boolean motorsEnabled;
		public int baseGas;
		public int motorGas[] = new int[4];
		public boolean stabilizationEnabled;
		public int mainLoopTime;
		public float temperature;
		public float pressure;
		public float altitude;
		public float seaLevel;
		// no sense variable, this is 2 bytes gap in C structure
		protected int gap;
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			DroneState state = (DroneState)super.clone();
			state.battery = (Battery)battery.clone();
			state.accel = (TripleAxisSensor)accel.clone();
			state.gyro = (TripleAxisSensor)gyro.clone();
			state.magneto = (TripleAxisSensor)magneto.clone();
			state.yawRatePid = (Pid)yawRatePid.clone();
			state.pitchPid = (Pid)pitchPid.clone();
			state.rollPid = (Pid)rollPid.clone();
			state.altPid = (Pid)altPid.clone();
			return state;
		}
		
		private void parse(DatagramPacket packet)
		{
			timestamp = CopterTelemetry.this.getUint32t(packet);
			battery.parse(packet);
			wifiLevel = CopterTelemetry.this.getInt32t(packet);
			accel.parse(packet);
			gyro.parse(packet);
			magneto.parse(packet);
			yaw = CopterTelemetry.this.getFloat(packet);
			pitch = CopterTelemetry.this.getFloat(packet);
			roll = CopterTelemetry.this.getFloat(packet);
			heading = CopterTelemetry.this.getFloat(packet);
			mainLoopTime = CopterTelemetry.this.getInt32t(packet);
			temperature = CopterTelemetry.this.getFloat(packet);
			pressure = CopterTelemetry.this.getFloat(packet);
			altitude = CopterTelemetry.this.getFloat(packet);
			seaLevel = CopterTelemetry.this.getFloat(packet);
			yawRatePid.parse(packet);
			pitchPid.parse(packet);
			rollPid.parse(packet);
			altPid.parse(packet);
			baseGas = CopterTelemetry.this.getInt32t(packet);
			motorGas[0] = CopterTelemetry.this.getInt32t(packet);
			motorGas[1] = CopterTelemetry.this.getInt32t(packet);
			motorGas[2] = CopterTelemetry.this.getInt32t(packet);
			motorGas[3] = CopterTelemetry.this.getInt32t(packet);
			motorsEnabled = CopterTelemetry.this.getBool(packet);
			stabilizationEnabled = CopterTelemetry.this.getBool(packet);
			gap = CopterTelemetry.this.getInt16t(packet);
		}
	}
	
	private DroneState droneState;
	private int mTelemetryPeriod;
	
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
	
	public DroneState getDroneState()
	{
		DroneState state = new DroneState();
		
		synchronized(objDataSync)
		{
			try
			{
				state = (DroneState)droneState.clone();
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
			result = mTelemetryPeriod;
		}
		
		return result;
	}
	
	public long droneStateSize;

	@Override
	public void run()
	{	
		byte[] receivedData = new byte[1024];
		
		while(mTelemetryRun)
		{
			DatagramPacket receivedPacket = new DatagramPacket(receivedData, receivedData.length);
			
			try
			{
				mSocket.receive(receivedPacket);
				
				if(receivedPacket.getAddress().equals(mCopterInetAddress))
				{
					AlarmCenter.instance().clearAlarm(Alarm.COPTER_NOT_FOUND);
					AlarmCenter.instance().clearAlarm(Alarm.COPTER_RECEIVE_ERROR);

					synchronized(objDataSync)
					{
						mParsePos = 0;
						droneStateSize = this.getUint32t(receivedPacket);
						droneState = new DroneState();
						droneState.parse(receivedPacket);
						mTelemetryPeriod = (int)this.getUint32t(receivedPacket);
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
	
	private boolean getBool(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		
		return (result > 0)? true : false;
	}
	
	@SuppressWarnings("unused")
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
	
	private long getUint32t(DatagramPacket packet)
	{
		long result = 0;
		long t = 0;
		
		t = packet.getData()[mParsePos++] & 0xFF;
		result = t;
		t = (packet.getData()[mParsePos++] & 0xFF);
		result |= t << 8;
		t = (packet.getData()[mParsePos++] & 0xFF);
		result |= t << 16;
		t = (packet.getData()[mParsePos++] & 0xFF);
		result |= t << 24;
		
		return result;
	}
	
	@SuppressWarnings("unused")
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
