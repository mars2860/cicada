package pdl;

import pdl.wlan.WlanLogPacket;

public class DroneLog extends java.util.Observable
{
	private static DroneLog mSingleton;
	
	public static DroneLog instance()
	{
		if(mSingleton == null)
			mSingleton = new DroneLog();
		
		return mSingleton;
	}

	private Object mLogLock;
	private String mLog;

	private DroneLog()
	{
		mLog = new String();
		mLogLock = new Object();
	}
		
	public void append(WlanLogPacket packet)
	{
		synchronized(mLogLock)
		{
			mLog += packet.getText();
		}
		
		this.setChanged();
		this.notifyObservers(packet.getText());
		this.clearChanged();
	}
	
	public String getLog()
	{
		String log = new String();
		
		synchronized(mLogLock)
		{
			log = new String(mLog);
		}
		
		return log;
	}
	
	public void clearLog()
	{
		synchronized(mLogLock)
		{
			mLog = "";
		}
		
		this.setChanged();
		this.notifyObservers("");
		this.clearChanged();
	}
}
