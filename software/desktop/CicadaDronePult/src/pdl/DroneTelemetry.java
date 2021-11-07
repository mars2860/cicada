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
	
	private DroneState mDroneState = new DroneState();
	private int mBlackBoxSize;
	private Deque<DroneState> mBlackBox;
	private static final int MAX_BLACK_BOX_SIZE = 64000;	// about 18 Mb
	private int mDroneTelemetryPort;
	private InetAddress mDroneInetAddress;
	private DatagramSocket mSocket;
	private Thread mTelemetryThread;
	private boolean mTelemetryRun;
	private Object objTelemetrySync;
	private Object objDataSync;
	private boolean mDroneConnected;

	private DroneTelemetry()
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
		
		mDroneTelemetryPort = port;
		mDroneInetAddress = InetAddress.getByName(ip);
		mSocket = new DatagramSocket(mDroneTelemetryPort);
		mSocket.setSoTimeout(TIMEOUT);
		mTelemetryRun = true;
		mTelemetryThread = new Thread(this);
		mTelemetryThread.setName("DroneTelemetryReceiver");
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
		
		DroneAlarmCenter.instance().clearAlarm(Alarm.DRONE_NOT_FOUND);
		DroneAlarmCenter.instance().clearAlarm(Alarm.DRONE_RECEIVE_ERROR);
	}
	
	public boolean isCopterConnected()
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
			result = mDroneState.net.telemetryPeriod;
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
				
				if(receivedPacket.getAddress().equals(mDroneInetAddress))
				{
					DroneAlarmCenter.instance().clearAlarm(Alarm.DRONE_NOT_FOUND);
					DroneAlarmCenter.instance().clearAlarm(Alarm.DRONE_RECEIVE_ERROR);

					synchronized(objDataSync)
					{
						mDroneConnected = true;
						BinaryParser parser = new BinaryParser();
						droneStateSize = parser.getUint32t(receivedPacket);
						mDroneState = new DroneState();
						mDroneState.parse(parser, receivedPacket);
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
				mDroneConnected = false;
				DroneAlarmCenter.instance().setAlarm(Alarm.DRONE_NOT_FOUND);
			}
			catch(IOException e)
			{
				DroneAlarmCenter.instance().setAlarm(Alarm.DRONE_RECEIVE_ERROR);
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
		int telemetryPeriod = mDroneState.net.telemetryPeriod;
		
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
