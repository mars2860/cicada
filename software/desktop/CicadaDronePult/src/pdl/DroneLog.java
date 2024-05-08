package pdl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;

public class DroneLog extends java.util.Observable implements Runnable
{
	private static DroneLog mSingleton;
	
	public static DroneLog instance()
	{
		if(mSingleton == null)
			mSingleton = new DroneLog();
		
		return mSingleton;
	}
	
	public static final int TIMEOUT = 3000;
	
	private int mDroneLogPort;
	private InetAddress mDroneInetAddress;
	private DatagramSocket mSocket;
	private Thread mLogThread;
	private boolean mLogRun;
	private Object objLogSync;
	private Object objDataSync;
	
	private String mLog;

	private DroneLog()
	{
		objLogSync = new Object();
		objDataSync = new Object();
		mLog = new String();
	}
	
	/** Start new thread to receive log packets. If thread is started it will be restarted */
	public void start(String ip, int port) throws UnknownHostException, SocketException
	{
		if(mLogRun || mSocket != null)
			this.stop();
		
		mDroneLogPort = port;
		mDroneInetAddress = InetAddress.getByName(ip);
		mSocket = new DatagramSocket(mDroneLogPort);
		mSocket.setSoTimeout(TIMEOUT);
		mLogRun = true;
		mLogThread = new Thread(this);
		mLogThread.setName("DroneLogReceiver");
		mLogThread.start();
	}
	
	public void stop()
	{
		mLogRun = false;
		
		try
		{
			synchronized(objLogSync)
			{
				// Unblock log thread
				objLogSync.notify();
				//System.out.println("Wait until log thread stops");
				// Wait until thread stops its work
				if(mLogThread != null && mLogThread.isAlive())
				{
					objLogSync.wait();
				}
				//System.out.println("End to wait for log thread stop");
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
	}
	
	public String getLog()
	{
		String log = new String();
		
		synchronized(objDataSync)
		{
			log = new String(mLog);
		}
		
		return log;
	}
	
	@Override
	public void run()
	{	
		byte[] receivedData = new byte[1024];
		
		while(mLogRun)
		{
			DatagramPacket receivedPacket = new DatagramPacket(receivedData, receivedData.length);
			
			try
			{
				mSocket.receive(receivedPacket);
				
				if(receivedPacket.getAddress().equals(mDroneInetAddress))
				{
					String msg = "";
					
					synchronized(objDataSync)
					{
						msg = new String(receivedPacket.getData(),0,receivedPacket.getLength());
					    mLog += msg;
						
						/* Debug
						System.out.println("log sz = " + receivedPacket.getLength());
						
						for(int i = 0; i < receivedPacket.getLength(); i++) {
							System.out.printf("%c ", receivedData[i]);
						}
						
						System.out.println(msg);
						System.out.println("new log");
						*/
					}

					this.setChanged();
					this.notifyObservers(msg);
					this.clearChanged();
				}
			}
			catch(SocketTimeoutException e)
			{
				// timeout
			}
			catch(IOException e)
			{
				e.printStackTrace();
			}
		}
		
		synchronized(objLogSync)
		{
			//System.out.println("Notify log thread is stopped");
			objLogSync.notify();
		}
	}
}
