package copter;

import java.io.IOException;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.util.Deque;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedDeque;

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
	
	@Retention(RetentionPolicy.RUNTIME)
	@Target(ElementType.FIELD)
	public @interface NoChart {}
	
	/** @note all types are double for fast processing charts */
	public class DroneState implements Cloneable
	{	
		public class Battery implements Cloneable
		{
			public double voltage;
			public double percent;
			
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
			public double rawX;
			public double rawY;
			public double rawZ;
			public double filteredX;
			public double filteredY;
			public double filteredZ;
			public double pureX;
			public double pureY;
			public double pureZ;
			@NoChart
			public double offsetX;
			@NoChart
			public double offsetY;
			@NoChart
			public double offsetZ;
			// no sense variable, this is 2 bytes gap in C structure
			protected int gap;
			
			private void parse(DatagramPacket packet)
			{
				pureX = CopterTelemetry.this.getFloat(packet);
				pureY = CopterTelemetry.this.getFloat(packet);
				pureZ = CopterTelemetry.this.getFloat(packet);
				
				rawX = CopterTelemetry.this.getInt16t(packet);
				rawY = CopterTelemetry.this.getInt16t(packet);
				rawZ = CopterTelemetry.this.getInt16t(packet);
				
				filteredX = CopterTelemetry.this.getInt16t(packet);
				filteredY = CopterTelemetry.this.getInt16t(packet);
				filteredZ = CopterTelemetry.this.getInt16t(packet);

				offsetX = CopterTelemetry.this.getInt16t(packet);
				offsetY = CopterTelemetry.this.getInt16t(packet);
				offsetZ = CopterTelemetry.this.getInt16t(packet);
				
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
			@NoChart
			public boolean enabled;
			@NoChart
			public double kp;
			@NoChart
			public double ki;
			@NoChart
			public double kd;
			@NoChart
			public double target;
			@NoChart
			public double errSum;
			@NoChart
			public double prevErr;
			
			public double out;
			
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
		
		@NoChart
		public double timestamp;
		public Battery battery = new Battery();
		public double wifiLevel;
		public TripleAxisSensor accel = new TripleAxisSensor();
		public TripleAxisSensor gyroRad = new TripleAxisSensor();
		public TripleAxisSensor gyroDeg = new TripleAxisSensor();
		public TripleAxisSensor magneto = new TripleAxisSensor();
		public double yawRad;
		public double yawDeg;
		public double pitchRad;
		public double pitchDeg;
		public double rollRad;
		public double rollDeg;
		public double headingRad;
		public double headingDeg;
		public Pid yawRadRatePid = new Pid();
		public Pid yawDegRatePid = new Pid();
		public Pid pitchRadPid = new Pid();
		public Pid pitchDegPid = new Pid();
		public Pid rollRadPid = new Pid();
		public Pid rollDegPid = new Pid();
		public Pid altPid = new Pid();
		@NoChart
		public boolean motorsEnabled;
		public double baseGas;
		public double motorGas0;
		public double motorGas1;
		public double motorGas2;
		public double motorGas3;
		@NoChart
		public boolean stabilizationEnabled;
		public double mainLoopTime;
		public double temperature;
		public double pressure;
		public double altitude;
		@NoChart
		public double seaLevel;
		// no sense variable, this is 2 bytes gap in C structure
		protected int gap;
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			DroneState state = (DroneState)super.clone();
			state.battery = (Battery)battery.clone();
			state.accel = (TripleAxisSensor)accel.clone();
			state.gyroRad = (TripleAxisSensor)gyroRad.clone();
			state.magneto = (TripleAxisSensor)magneto.clone();
			state.yawRadRatePid = (Pid)yawRadRatePid.clone();
			state.pitchRadPid = (Pid)pitchRadPid.clone();
			state.rollRadPid = (Pid)rollRadPid.clone();
			state.altPid = (Pid)altPid.clone();
			return state;
		}
		
		private void parse(DatagramPacket packet)
		{
			timestamp = CopterTelemetry.this.getUint32t(packet);
			battery.parse(packet);
			wifiLevel = CopterTelemetry.this.getInt32t(packet);
			accel.parse(packet);
			gyroRad.parse(packet);
			magneto.parse(packet);
			yawRad = CopterTelemetry.this.getFloat(packet);
			yawDeg = Math.toDegrees(yawRad);
			pitchRad = CopterTelemetry.this.getFloat(packet);
			pitchDeg = Math.toDegrees(pitchRad);
			rollRad = CopterTelemetry.this.getFloat(packet);
			rollDeg = Math.toDegrees(rollRad);
			headingRad = CopterTelemetry.this.getFloat(packet);
			headingDeg = Math.toDegrees(headingRad);
			mainLoopTime = CopterTelemetry.this.getInt32t(packet);
			temperature = CopterTelemetry.this.getFloat(packet);
			pressure = CopterTelemetry.this.getFloat(packet);
			altitude = CopterTelemetry.this.getFloat(packet);
			seaLevel = CopterTelemetry.this.getFloat(packet);
			yawRadRatePid.parse(packet);
			pitchRadPid.parse(packet);
			rollRadPid.parse(packet);
			altPid.parse(packet);
			baseGas = CopterTelemetry.this.getInt32t(packet);
			motorGas0 = CopterTelemetry.this.getInt32t(packet);
			motorGas1 = CopterTelemetry.this.getInt32t(packet);
			motorGas2 = CopterTelemetry.this.getInt32t(packet);
			motorGas3 = CopterTelemetry.this.getInt32t(packet);
			motorsEnabled = CopterTelemetry.this.getBool(packet);
			stabilizationEnabled = CopterTelemetry.this.getBool(packet);
			gap = CopterTelemetry.this.getInt16t(packet);
			
			try
			{
				gyroDeg = (TripleAxisSensor)gyroRad.clone();
				gyroDeg.pureX = Math.toDegrees(gyroRad.pureX);
				gyroDeg.pureY = Math.toDegrees(gyroRad.pureY);
				gyroDeg.pureZ = Math.toDegrees(gyroRad.pureZ);
				
				yawDegRatePid = (Pid)yawRadRatePid.clone();
				yawDegRatePid.target = Math.toDegrees(yawRadRatePid.target);
				
				pitchDegPid = (Pid)pitchRadPid.clone();
				pitchDegPid.target = Math.toDegrees(pitchRadPid.target);
				
				rollDegPid = (Pid)rollRadPid.clone();
				rollDegPid.target = Math.toDegrees(rollRadPid.target);
			}
			catch(CloneNotSupportedException e)
			{
				e.printStackTrace();
			}
		}
	}
	
	private DroneState mDroneState;
	private int mTelemetryPeriod;
	private int mBlackBoxSize;
	private Deque<DroneState> mBlackBox;
	private static final int MAX_BLACK_BOX_SIZE = 64000;	// about 18 Mb
	
	
	private CopterTelemetry()
	{
		objTelemetrySync = new Object();
		objDataSync = new Object();
		mBlackBox = new ConcurrentLinkedDeque<DroneState>();
		mBlackBoxSize = 0;
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
						mDroneState = new DroneState();
						mDroneState.parse(receivedPacket);
						mTelemetryPeriod = (int)this.getUint32t(receivedPacket);
						mBlackBox.offer(mDroneState);
						mBlackBoxSize++;
						if(mBlackBoxSize >= MAX_BLACK_BOX_SIZE)
						{
							mBlackBox.poll();
							mBlackBoxSize--;
						}
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
	
	/**
	 * @param range microseconds from now to (now minus range). if 0 then return all
	 * */
	public DroneState[] getBlackBox(int range)
	{
		DroneState[] result = new DroneState[0];
		
		if(range > 0 && mTelemetryPeriod > 0)
		{
			int count = Math.min(range/mTelemetryPeriod, mBlackBoxSize);

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
