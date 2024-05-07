package pdl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.concurrent.Semaphore;

import pdl.commands.AbstractCopterCmd;
import pdl.commands.CmdSetAccel;
import pdl.commands.CmdSetAltPid;
import pdl.commands.CmdSetGyro;
import pdl.commands.CmdSetMagneto;
import pdl.commands.CmdSetPeriods;
import pdl.commands.CmdSetPitchPid;
import pdl.commands.CmdSetPitchRatePid;
import pdl.commands.CmdSetRollPid;
import pdl.commands.CmdSetRollRatePid;
import pdl.commands.CmdSetSeaLevelPressure;
import pdl.commands.CmdSetVelocityXPid;
import pdl.commands.CmdSetVelocityYPid;
import pdl.commands.CmdSetVelocityZPid;
import pdl.commands.CmdSetYawRatePid;
import pdl.commands.CmdSetup;

public class DroneCommander implements Runnable
{
	public static final int SETUP_ATTEMPTS_COUNT = 10;
	
	private static DroneCommander mSingleton;

	public static DroneCommander instance()
	{
		if(mSingleton == null)
			mSingleton = new DroneCommander();
		
		return mSingleton;
	}
	
	private int mDroneCmdPort;
	private InetAddress mDroneInetAddress;
	private DatagramSocket mSocket;
	private Thread mSender;
	private boolean mSenderRun;
	private ArrayList<AbstractCopterCmd> mCmds;
	private Semaphore mListMutex;
	private Object objSenderSync;
	
	private DroneCommander() 
	{
		mCmds = new ArrayList<AbstractCopterCmd>();
		mListMutex = new Semaphore(1);
		objSenderSync = new Object();
	};
	
	public void start(String ip, int cmdPort) throws UnknownHostException, SocketException
	{
		if(mSocket != null)
			this.stop();
		
		mDroneCmdPort = cmdPort;
		mDroneInetAddress = InetAddress.getByName(ip);
		mSocket = new DatagramSocket();
		mSenderRun = true;
		mSender = new Thread(this);
		mSender.setName("DroneCommandSender");
		mSender.start();
	}
	
	public void stop()
	{
		mSenderRun = false;
		
		//System.out.println("Commander thread has started");
		
		try
		{
			synchronized(objSenderSync)
			{
				// Unblock commander thread
				objSenderSync.notify();
				//System.out.println("Wait until commander thread stops");
				// Wait until commander thread stops its work
				if(mSender != null && mSender.isAlive())
					objSenderSync.wait();
				//System.out.println("End to wait for commander thread stop");
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
		
		DroneAlarmCenter.instance().clearAlarm(Alarm.DRONE_SEND_ERROR);
	}
	
	public void addCmd(AbstractCopterCmd cmd)
	{
		try
		{
			mListMutex.acquire();
			mCmds.add(cmd);
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
		finally
		{
			mListMutex.release();
			synchronized(objSenderSync)
			{
				// Resume sender thread
				//System.out.println("Notify commander thread about new command");
				objSenderSync.notify();
			}
		}
	}

	@Override
	public void run()
	{
		AbstractCopterCmd cmd = null;
		int cmdIdx = 0;
		
		while(mSenderRun)
		{
			try
			{
				mListMutex.acquire();
				
				if(mCmds.size() > 0)
				{
					cmdIdx = mCmds.size() - 1;
					cmd = mCmds.get(cmdIdx);
					mCmds.remove(cmdIdx);
				}
				else
				{
					cmd = null;
					cmdIdx = 0;
				}
			}
			catch(Exception e)
			{
				e.printStackTrace();
			}
			finally
			{
				mListMutex.release();
			}
			
			if(cmd != null)
			{
				byte buf[] = cmd.getPacketData();
				DatagramPacket packet = new DatagramPacket(buf, buf.length, mDroneInetAddress, mDroneCmdPort);
				//System.out.println("Send command");
				try
				{
					mSocket.send(packet);
					// give time to copter to process a command
					Thread.sleep(20);
				}
				catch(IOException e)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.DRONE_SEND_ERROR);
					e.printStackTrace();
				}
				catch (InterruptedException e)
				{
					e.printStackTrace();
				}
				
				cmd = null;
				cmdIdx = 0;
			}
			else	// wait until new command will be
			{
				try
				{
					synchronized(objSenderSync)
					{
						//System.out.println("Wait until there are commands");
						objSenderSync.wait();
					}
				}
				catch(Exception e)
				{
					e.printStackTrace();
				}
			}
		}
		
		synchronized(objSenderSync)
		{
			//System.out.println("Notify commander thread is stopped");
			objSenderSync.notify();
		}
	}
	
	public void sendSetupCmd(CmdSetup cmd)
	{
		int attempt = 0;
		while(attempt < SETUP_ATTEMPTS_COUNT)
		{
			DroneCommander.instance().addCmd(cmd);
			// wait for command execution
			try
			{
				Thread.sleep(50);
			}
			catch(Exception e)
			{
				e.printStackTrace();
			}
			// check command execution
			DroneState ds = DroneTelemetry.instance().getDroneState();
			if(cmd.settingsEquals(ds))
				break;
			attempt++;
		}
		
		if(attempt >= (SETUP_ATTEMPTS_COUNT - 1) )
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.DRONE_SEND_ERROR);
			System.out.println("Number of attempts exceeded for " + cmd.toString());
		}
	}
	
	public void sendSettingsToDrone(DroneState ds)
	{
		DroneAlarmCenter.instance().clearAlarm(Alarm.DRONE_SEND_ERROR);
		
		CmdSetup cmd;
		// setup accel
		cmd = new CmdSetAccel(ds.accel);
		sendSetupCmd(cmd);
		
		// setup gyro
		cmd = new CmdSetGyro(ds.gyro);
		sendSetupCmd(cmd);
		
		// setup magneto
		cmd = new CmdSetMagneto(ds.magneto,1.f,1.f,1.f);
		sendSetupCmd(cmd);
		
		// setup yawRatePid
		cmd = new CmdSetYawRatePid(ds.yawRatePid);
		sendSetupCmd(cmd);
		
		// setup pitchRatePid
		cmd = new CmdSetPitchRatePid(ds.pitchRatePid);
		sendSetupCmd(cmd);
		
		// setup rollRatePid
		cmd = new CmdSetRollRatePid(ds.rollRatePid);
		sendSetupCmd(cmd);
		
		// setup pitchPid
		cmd = new CmdSetPitchPid(ds.pitchPid);
		sendSetupCmd(cmd);
		
		// setup rollPid
		cmd = new CmdSetRollPid(ds.rollPid);
		sendSetupCmd(cmd);
		
		// setup altPid
		cmd = new CmdSetAltPid(ds.altPid);
		sendSetupCmd(cmd);
		
		// setup velocityXPid
		cmd = new CmdSetVelocityXPid(ds.velocityXPid);
		sendSetupCmd(cmd);
		
		// setup velocityYPid
		cmd = new CmdSetVelocityYPid(ds.velocityYPid);
		sendSetupCmd(cmd);
		
		// setup velocityZPid
		cmd = new CmdSetVelocityZPid(ds.velocityZPid);
		sendSetupCmd(cmd);
		
		// setup telemetry period
		cmd = new CmdSetPeriods(ds.net.telemetryPeriod,0);
		sendSetupCmd(cmd);
		
		// setup barometer
		cmd = new CmdSetSeaLevelPressure((float)ds.baro.seaLevelPressure);
		sendSetupCmd(cmd);
	}
}
