package pdl;

import java.util.ArrayList;

import com.fazecast.jSerialComm.SerialPort;
import com.fazecast.jSerialComm.SerialPortInvalidPortException;

import pdl.DroneState.RemoteCtrl.MoveBy;
import pdl.DroneState.RemoteCtrl.RotateBy;
import pdl.commands.AbstractCmdSetPidTarget;
import pdl.commands.AbstractDroneCmd;
import pdl.commands.CmdEnableDynamicIp;
import pdl.commands.CmdEnableStabilization;
import pdl.commands.CmdEnableWifiBroadcast;
import pdl.commands.CmdSetTrickMode;
import pdl.commands.CmdLoadDefaultCfg;
import pdl.commands.CmdResetAltitude;
import pdl.commands.CmdSaveDefaultCfg;
import pdl.commands.CmdSetAccel;
import pdl.commands.CmdSetAltPid;
import pdl.commands.CmdSetAltitude;
import pdl.commands.CmdSetBaseGas;
import pdl.commands.CmdSetBattery;
import pdl.commands.CmdSetCameraAngle;
import pdl.commands.CmdSetEsc;
import pdl.commands.CmdSetFrame;
import pdl.commands.CmdSetGateway;
import pdl.commands.CmdSetGps;
import pdl.commands.CmdSetGyro;
import pdl.commands.CmdSetHeading;
import pdl.commands.CmdSetHeadingPid;
import pdl.commands.CmdSetIp;
import pdl.commands.CmdSetKalman;
import pdl.commands.CmdSetLoad;
import pdl.commands.CmdSetMagneto;
import pdl.commands.CmdSetMotorsDir;
import pdl.commands.CmdSetMotorsGas;
import pdl.commands.CmdSetTelemetryPeriod;
import pdl.commands.CmdSetPitch;
import pdl.commands.CmdSetPitchPid;
import pdl.commands.CmdSetPitchRate;
import pdl.commands.CmdSetRoll;
import pdl.commands.CmdSetRollRate;
import pdl.commands.CmdSetSsid;
import pdl.commands.CmdSetSubnet;
import pdl.commands.CmdSetAccUpOffset;
import pdl.commands.CmdSetVelocityX;
import pdl.commands.CmdSetVelocityY;
import pdl.commands.CmdSetVelocityZ;
import pdl.commands.CmdSetYRatePid;
import pdl.commands.CmdSetYRate;
import pdl.commands.CmdSetPosNorthPid;
import pdl.commands.CmdSetPsk;
import pdl.commands.CmdSetPosEastPid;
import pdl.commands.CmdSetRollPid;
import pdl.commands.CmdSetXRatePid;
import pdl.commands.CmdSetXRate;
import pdl.commands.CmdSetVelocityXPid;
import pdl.commands.CmdSetVelocityYPid;
import pdl.commands.CmdSetVelocityZPid;
import pdl.commands.CmdSetYawRate;
import pdl.commands.CmdSetZRatePid;
import pdl.commands.CmdSetZRate;
import pdl.commands.CmdSetup;
import pdl.commands.CmdSetupWifi;
import pdl.commands.CmdStartStopVideo;
import pdl.commands.CmdSwitchMotors;
import pdl.commands.CmdTakePhoto;
import pdl.commands.CmdVeloZTakeoff;
import pdl.res.Profile;
import pdl.wlan.Modem;
import pdl.wlan.ModemType;
import pdl.wlan.WifiBroadcastModem;
import pdl.wlan.WifiBroadcastModem.WifiPhy;
import pdl.wlan.WifiBroadcastModem.WifiRate;
import pdl.wlan.WifiUdpModem;
import pdl.wlan.WlanCommandPacket;
import pdl.wlan.WlanLogPacket;
import pdl.wlan.WlanTelemetryPacket;

public class DroneCommander
{
	public static final int SETUP_ATTEMPTS_COUNT = 10;

	public static final double MIN_YAW_RATE = 10; // degrees per second
	public static final double MIN_PITCH_ROLL_RATE = 5; // degrees per second
	public static final double MIN_VELOCITY_Z = 0.1;   // meters per second
	public static final double MIN_VELOCITY_XY = 0.1; // meters per second
	public static final double MIN_PITCH_ROLL = 0.5;    // degrees

	public static final double CTRL_ERR = 0.001;
	public static final boolean LOG_CTRL_CMD = true;
	
	/** Delay in milliseconds between commands.
	 *  This delay is needed for the drone have time to process last received command
	 */
	public static final int DELAY_BTW_CMDS = 20;
	// FIXME It is strange but in wifi broadcast mode If we send to many commands the drone is reseted. This is noticed only when we invoke sendSettingsToDrone
	public static final int DELAY_BTW_CMDS_BROADCAST_MODE = 50;
	
	private static DroneCommander mSingleton;

	public static DroneCommander instance()
	{
		if(mSingleton == null)
			mSingleton = new DroneCommander();
		
		return mSingleton;
	}
	
	private ArrayList<AbstractDroneCmd> mCmds;

	private boolean antiTurtleEnabled;
	private int antiTurtleToggle;
	private Modem mModem;
	private Thread mthModemRx;
	private Thread mthModemTx;
	private Thread mthConnect;
	private boolean mModemRxRun;
	private boolean mModemTxRun;
	private Object mCmdLock;
	private Object mNewCmdLock;
	private String mSsid;
	private int mWifiChl;
	private int mWifiPhy;
	private int mWifiRate;
	private float mWifiTpwDbm;
	private boolean mDroneConnected = false;
	// the host system timestamp at the drone were connected
	private long connectTimestamp;
	
	private DroneCommander() 
	{
		mCmds = new ArrayList<AbstractDroneCmd>();
		mCmdLock = new Object();
		mNewCmdLock = new Object();
		connectTimestamp = System.currentTimeMillis();
	};
	
	private class OnModemRx implements Runnable
	{
		@Override
		public void run()
		{
			int rxTimeout = 100;
			int pingErrCounter = 0;
			int recvErrCounter = 0;
			
			mModemRxRun = true;
			
			while(mModemRxRun && mModem != null)
			{
				// select receive timeout
				if(DroneTelemetry.instance().isDroneConnected())
				{
					DroneState ds = DroneTelemetry.instance().getDroneState();
					int telemetryPeriod = 2*ds.telemetry.period / 1000; // convert microseconds to milliseconds
					if(telemetryPeriod >= 100 && telemetryPeriod != rxTimeout)
					{
						rxTimeout = telemetryPeriod;
					}
				}
				// we assume that the drone is disconnected if it doesn't send any data in 3 seconds
				int maxRecvErrCounter = Math.max(1, 3000 / rxTimeout);
				
				byte[] data = mModem.receive(rxTimeout);
				
				if(data == null)
				{
					recvErrCounter++;
					// there is no data from the drone
					// in this case test connection with modem
					if(mModem.ping(100) == false)
					{
						pingErrCounter++;
					}
					else
					{
						pingErrCounter = 0;
					}
				}
				else
				{
					pingErrCounter = 0;
					
					WlanLogPacket logPacket = WlanLogPacket.parse(data);
					
					if(logPacket != null)
					{
						if(logPacket.getSsid().compareTo(mSsid) == 0)
						{
							DroneLog.instance().append(logPacket);
							recvErrCounter = 0;
						}
						else
						{
							System.out.println("Invalid log ssid");
						}
					}
					else
					{
						WlanTelemetryPacket telemetryPacket = WlanTelemetryPacket.parse(data);
						
						if(telemetryPacket.getSsid().compareTo(mSsid) == 0)
						{
							DroneTelemetry.instance().append(telemetryPacket);
							recvErrCounter = 0;
						}
					}
				}
				
				if(recvErrCounter > maxRecvErrCounter)
				{
					recvErrCounter = maxRecvErrCounter;
					setDroneConnected(false);
				}
				else if(recvErrCounter == 0)
				{
					setDroneConnected(true);
					DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_RECEIVE_ERROR);
				}
				else
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_RECEIVE_ERROR);
				}
				
				if(pingErrCounter > 10)
				{
					pingErrCounter = 10;
					setDroneConnected(false);
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_MODEM_CONNECTION_LOST);
					// try to reconnect
					if(mModem != null)
					{
						mModem.connect();
					}
				}
				else if(pingErrCounter == 0)
				{
					DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_MODEM_CONNECTION_LOST);
				}
			}
		}
	}
	
	private class OnModemTx implements Runnable
	{
		@Override
		public void run()
		{
			mModemTxRun = true;
			AbstractDroneCmd cmd = null;
			
			while(mModemTxRun && mModem != null)
			{
				synchronized(mCmdLock)
				{
					if(mCmds.size() > 0)
					{
						cmd = mCmds.get(0);
						mCmds.remove(0);
					}
					else
					{
						cmd = null;
					}
				}
					
				if(cmd != null)
				{
					WlanCommandPacket packet = new WlanCommandPacket(cmd, mSsid);
										
					if(mModem.send(packet.getDataToSend()))
					{
						DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_SEND_ERROR);
					}
					else
					{
						DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_SEND_ERROR);
					}
					
					try
					{
						// give time to the drone to process a command
						if(mModem instanceof WifiBroadcastModem)
						{
							Thread.sleep(DELAY_BTW_CMDS_BROADCAST_MODE);
						}
						else
						{
							Thread.sleep(DELAY_BTW_CMDS);
						}
					}
					catch(Exception e)
					{
						e.printStackTrace();
					}

					cmd = null;
				}
				else
				{
					try
					{
						synchronized(mNewCmdLock)
						{
							//long timestamp = System.currentTimeMillis();
							mNewCmdLock.wait(1000);
							//timestamp = System.currentTimeMillis() - timestamp;
							//System.out.println("Wait cmd=" + timestamp);
						}
					}
					catch(Exception e)
					{
						e.printStackTrace();
					}
				}
			}
		}
	}
	
	private void setDroneConnected(boolean state)
	{
		if(state != mDroneConnected)
		{
			connectTimestamp = System.currentTimeMillis();
			mDroneConnected = state;
			if(state)
				DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_DRONE_NOT_FOUND);
			else
				DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DRONE_NOT_FOUND);
		}
	}
	
	public long getDroneConnectionTime()
	{
		if(this.isDroneConnected() == false)
			return 0;
		
		return System.currentTimeMillis() - connectTimestamp;
	}
	
	public boolean isDroneConnected()
	{
		return mDroneConnected;
	}
	
	public void connect()
	{
		if(DroneAlarmCenter.instance().getAlarm(Alarm.ALARM_CONNECTING))
			return;
		
		disconnect();
		
		DroneLog.instance().clearLog();
		DroneTelemetry.instance().clearBlackBox();
		DroneTelemetry.instance().resetFlyTime();
		DroneAlarmCenter.instance().clearAll();
		
		DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_CONNECTING);
		
		// Store current ssid
		// After the user can change ssid in settings
		// but it will be not actual ssid
		// it get actual after drone is reloaded
		mSsid = DroneState.net.ssid;
		mWifiChl = DroneState.net.wifiChannel;
		mWifiPhy = DroneState.net.wifiPhy;
		mWifiRate = DroneState.net.wifiRate;
		mWifiTpwDbm = DroneState.net.wifiTxPowerDbm;
		
		mthConnect = new Thread(new Runnable()
		{
			@Override
			public void run()
			{					
				DroneState ds = Profile.instance().getDroneSettings();
				// We always have to start with UDP modem!
				// Start UDP modem
				mModem = connectWifiUdpModem(DroneState.net.ip,DroneState.net.udpPort);
				// Connect
				if(mModem == null)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_UDP_MODEM_CONNECTION_ERROR);
					DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_CONNECTING);
					return;
				}
				// Start modem rxtx threads
				mthModemRx = new Thread(new OnModemRx(), "DroneModemRx");
				mthModemRx.start();
				mthModemTx = new Thread(new OnModemTx(), "DroneModemTx");
				mthModemTx.start();
				// Send new settings
				DroneCommander.instance().sendSettingsToDrone(ds);
				// Check connection
				if(DroneCommander.instance().isDroneConnected() == false)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DRONE_NOT_FOUND);
				}
				// Connection ends
				DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_CONNECTING);
			}
		});
		
		mthConnect.start();
	}
	
	public void disconnect()
	{
		this.clearCmdQueue();
		mModemRxRun = false;
		mModemTxRun = false;
		try
		{
			if(mthConnect != null)
			{
				mthConnect.join(3000);
			}
			
			if(mthModemRx != null)
			{
				mthModemRx.join(3000);
			}
			
			if(mthModemTx != null)
			{
				mthModemTx.join(3000);
			}
			mthConnect = null;
			mthModemRx = null;
			mthModemTx = null;
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}

		if(mModem != null)
		{
			mModem.disconnect();
			mModem = null;
		}
		
		DroneAlarmCenter.instance().clearAll();
		DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DRONE_NOT_FOUND);
	}
	
	/** @return false if the list of commands has the command with the same code */
	public boolean addCmd(AbstractDroneCmd cmd)
	{
		boolean result = true;

		synchronized(mCmdLock)
		{	
			mCmds.add(cmd);
		}
		
		synchronized(mNewCmdLock)
		{
			mNewCmdLock.notifyAll();
		}
		
		return result;
	}

	public int getCmdCount()
	{
		int result = 0;
		
		synchronized(mCmdLock)
		{
			result = mCmds.size();
		}
		
		return result;
	}
	
	public void clearCmdQueue()
	{
		synchronized(mCmdLock)
		{
			mCmds.clear();
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
				Thread.sleep(100);
			}
			catch(Exception e)
			{
				e.printStackTrace();
			}
			
			// if drone were disconnected and connected again
			// we would get old-connection droneState from DroneTelemetry
			// to prevent this situation check for connection 
			if(DroneTelemetry.instance().isDroneConnected())
			{
				// check command execution
				DroneState ds = DroneTelemetry.instance().getDroneState();
				if(cmd.settingsEquals(ds))
					break;
			}
			attempt++;
		}
		
		if(attempt >= (SETUP_ATTEMPTS_COUNT - 1) )
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_SEND_SETUP_ERROR);
			System.out.println("Number of attempts exceeded for " + cmd.toString());
		}
	}
	
	public void sendSettingsToDrone(DroneState ds)
	{
		DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_SEND_ERROR);
		
		// TODO May be it would be better to replace all setup commands by one command - send DroneState!
		
		CmdSetup cmd;
		
		// setup kalman filter
		cmd = new CmdSetKalman(ds.kfSettings);
		sendSetupCmd(cmd);
		
		// setup ESC
		cmd = new CmdSetEsc(ds.motors.esc);
		sendSetupCmd(cmd);
		
		// setup Frame
		cmd = new CmdSetFrame(ds.motors.frame);
		sendSetupCmd(cmd);
		
		// setup gyro
		cmd = new CmdSetGyro(ds.gyro);
		sendSetupCmd(cmd);
		
		// setup accel
		cmd = new CmdSetAccel(ds.accel);
		sendSetupCmd(cmd);
		
		// setup magneto
		cmd = new CmdSetMagneto(ds.magneto);
		sendSetupCmd(cmd);
		
		// setup zRatePid
		cmd = new CmdSetZRatePid(ds.zRatePid);
		sendSetupCmd(cmd);
		
		// setup yRatePid
		cmd = new CmdSetYRatePid(ds.yRatePid);
		sendSetupCmd(cmd);
		
		// setup xRatePid
		cmd = new CmdSetXRatePid(ds.xRatePid);
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
		
		// setup headingPid
		cmd = new CmdSetHeadingPid(ds.headingPid);
		sendSetupCmd(cmd);
		
		// setup posXPid
		cmd = new CmdSetPosNorthPid(ds.posNorthPid);
		sendSetupCmd(cmd);
		
		// setup posYPid
		cmd = new CmdSetPosEastPid(ds.posEastPid);
		sendSetupCmd(cmd);
		
		// setup telemetry period
		cmd = new CmdSetTelemetryPeriod(ds.telemetry.period);
		sendSetupCmd(cmd);
		
		// setup battery
		cmd = new CmdSetBattery( ds.battery.voltScaler,
								 ds.battery.curnScaler,
								 ds.battery.minVoltage,
								 ds.battery.maxVoltage);
		sendSetupCmd(cmd);
		
		// setup Gps
		cmd = new CmdSetGps(ds.gps);
		sendSetupCmd(cmd);
		
		// setup Load1
		cmd = new CmdSetLoad(0,ds.load1.enabled,ds.load1.period);
		sendSetupCmd(cmd);
		
		// setup kalman filter again to reset it
		cmd = new CmdSetKalman(ds.kfSettings);
		sendSetupCmd(cmd);
		
		// velupOffset
		cmd = new CmdSetAccUpOffset(ds.velocityZPid.accUpOffset);
		sendSetupCmd(cmd);
		
		// reset altitude
		AbstractDroneCmd cmdResAlt = new CmdResetAltitude();
		addCmd(cmdResAlt);
		
		// setup WiFi
		// ssid
		AbstractDroneCmd netCmd = new CmdSetSsid(DroneState.net.ssid);
		addCmd(netCmd);
		// psk
		netCmd = new CmdSetPsk(DroneState.net.psk);
		addCmd(netCmd);
		// ip
		netCmd = new CmdSetIp(DroneState.net.ip);
		addCmd(netCmd);
		// gateway
		netCmd = new CmdSetGateway(DroneState.net.gateway);
		addCmd(netCmd);
		// subnet
		netCmd = new CmdSetSubnet(DroneState.net.subnet);
		addCmd(netCmd);
		// dynamic ip
		CmdEnableDynamicIp enDynIp = new CmdEnableDynamicIp(DroneState.net.dynamicIp);
		addCmd(enDynIp);
		
		// wifi phy
		CmdSetupWifi wifiCmd = new CmdSetupWifi(
				DroneState.net.wifiStaMode,
				DroneState.net.wifiChannel,
				DroneState.net.wifiTxPowerDbm,
				DroneState.net.wifiPhy,
				DroneState.net.wifiRate );
		
		addCmd(wifiCmd);

		// Switch the drone to wifi broadcast mode or back to wifi normal mode
		if(DroneState.net.wifiBroadcastEnabled)
		{
			switchToWifiBroadcastModem();
		}
		else	// switch to wifi normal mode
		{
			switchToWifiUdpModem();
		}
	}
	
	private void sendSetModemCmd(boolean wifiBroadcastEnabled)
	{
		CmdEnableWifiBroadcast cmd = new CmdEnableWifiBroadcast(wifiBroadcastEnabled);

		DroneCommander.instance().addCmd(cmd);
		DroneCommander.instance().addCmd(cmd);
		DroneCommander.instance().addCmd(cmd);

		while(getCmdCount() > 0)	// wait until all commands are sent
		{
			try
			{
				Thread.sleep(100);
			}
			catch(Exception e)
			{
				e.printStackTrace();
			}
		}
	}
	
	private void setCurrentModem(Modem mdm)
	{
		if(mModem != null)
		{
			Modem oldModem = mModem;
			mModem = mdm;
			// if we disconnect old modem before applying new modem
			// RxThread blocks old modem trying to reconnect them
			oldModem.disconnect();
		}
		else
		{
			mModem = mdm;
		}
	}
	
	private WifiUdpModem connectWifiUdpModem(String ip, int port)
	{
		WifiUdpModem mdm = new WifiUdpModem(ip,port);
		
		if( mdm.connect() == true &&
			mdm.ping(100) == true )
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_UDP_MODEM_CONNECTION_ERROR);
		}
		else
		{
			mdm = null;
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_UDP_MODEM_CONNECTION_ERROR);
		}
		
		return mdm;
	}
	
	private WifiBroadcastModem connectWifiBroadcastModem(String comPortName)
	{
		WifiBroadcastModem mdm = null;
		
		try
		{
			// Test modem connection
			SerialPort port = SerialPort.getCommPort(comPortName);
			mdm = new WifiBroadcastModem(port);
			if( mdm.connect() == true &&
				mdm.ping(100) == true )
			{
				DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_WIFI_BROADCAST_MODEM_NOT_FOUND);
				setupWifiBroadcastModem(mdm);
			}
			else
			{
				mdm.disconnect();
				mdm = null;
				DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_WIFI_BROADCAST_MODEM_NOT_FOUND);
			}
		}
		catch(SerialPortInvalidPortException e)
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_CANT_OPEN_COM_PORT);
		}
		
		return mdm;
	}
	
	private void setCurrentModem(ModemType t)
	{
		DroneAlarmCenter.instance().clearModemAlarms();
		
		boolean wifiBroadcastEnabled = false;
		Modem mdm = null;
		Alarm err = null;
		
		switch(t)
		{
		case WIFI_UDP_MODEM:
			mdm = connectWifiUdpModem(DroneState.net.ip,DroneState.net.udpPort);
			wifiBroadcastEnabled = false;
			err = Alarm.ALARM_UDP_MODEM_CONNECTION_ERROR;
			break;
		case WIFI_BROADCAST_MODEM:
			mdm = connectWifiBroadcastModem(DroneState.net.wifiBroadcastModemComPort);
			wifiBroadcastEnabled = true;
			err = Alarm.ALARM_WIFI_BROADCAST_MODEM_NOT_FOUND;
			break;
		}

		if(mdm != null)
		{
			sendSetModemCmd(wifiBroadcastEnabled);
			setCurrentModem(mdm);
		}
		else
		{
			DroneAlarmCenter.instance().setAlarm(err);	
		}
	}
	
	private void switchToWifiBroadcastModem()
	{
		if(mModem instanceof WifiBroadcastModem)
		{
			setupWifiBroadcastModem((WifiBroadcastModem)mModem);
			return;
		}
		
		setCurrentModem(ModemType.WIFI_BROADCAST_MODEM);
	}
	
	private void switchToWifiUdpModem()
	{
		if(mModem instanceof WifiUdpModem)
		{
			return;
		}
			
		setCurrentModem(ModemType.WIFI_UDP_MODEM);
	}
	
	private void setupWifiBroadcastModem(WifiBroadcastModem mdm)
	{
		mdm.setChannel(mWifiChl);
		mdm.setMaxTpwDbm(mWifiTpwDbm);
		mdm.setWifiPhy(WifiPhy.fromeInt(mWifiPhy));
		mdm.setWifiRate(WifiRate.fromeInt(mWifiRate));	
	}

	/** Sets new target of pitchRatePid,pitchPid,velocityXPid, etc
	 *  depending from DroneState.rc.moveBy
	 *
	 * @param pwrFwd Moving power in forward from -1 to 1. New PID target will be DroneState.rc.moveDelta * pwrFwd
	 * @param pwrRight Moving power in right from -1 to 1. New PID target will be DroneState.rc.moveDelta * pwrRight
	 */
	public void moveDrone(double pwrFwd, double pwrRight)
	{
		DroneState ds = DroneTelemetry.instance().getDroneState();

		if(ds.motorsEnabled == false || ds.stabilizationEnabled == false)
			return;
		
		if(antiTurtleEnabled)
			return;
		
		checkCmdQueueSize();
		
		DroneState.RemoteCtrl.MoveBy moveBy = DroneState.rc.moveBy;
		float moveDelta = DroneState.rc.moveDelta;
		
		if(ds.trickMode != DroneState.TrickMode.DISABLED)
		{
			moveBy = DroneState.RemoteCtrl.MoveBy.MOVE_BY_X_Y_RATE;
			moveDelta = DroneState.rc.trickDelta;
		}

		switch(moveBy)
		{
			case MOVE_BY_X_Y_RATE:
			case MOVE_BY_PITCH_ROLL_RATE:
				double pitchRateTarget = Math.toRadians(-pwrFwd * moveDelta);
				double rollRateTarget = Math.toRadians(pwrRight * moveDelta);
				double curPitchRateTarget = ds.pitchRateTarget;
				double curRollRateTarget = ds.rollRateTarget;
				String pitchRateLog = "set pitchRate = ";
				String rollRateLog = "set rollRate = ";
				AbstractCmdSetPidTarget cmdPitchRate = new CmdSetPitchRate(0);
				AbstractCmdSetPidTarget cmdRollRate = new CmdSetRollRate(0);
				
				if(moveBy == MoveBy.MOVE_BY_X_Y_RATE)
				{
					curPitchRateTarget = ds.yRatePid.target;
					curRollRateTarget = ds.xRatePid.target;
					cmdPitchRate = new CmdSetYRate(0);
					cmdRollRate = new CmdSetXRate(0);
					pitchRateLog = "set yRate = ";
					rollRateLog = "set xRate = ";
				}

				if(Math.abs(pitchRateTarget) < Math.toRadians(MIN_PITCH_ROLL_RATE))
				{
					pitchRateTarget = 0;
				}

				if(Math.abs(rollRateTarget) < Math.toRadians(MIN_PITCH_ROLL_RATE))
				{
					rollRateTarget = 0;
				}

				if(pitchRateTarget == 0 && ds.pitchPid.enabled && ds.pitchPidFlag > 0)
				{
					if(	DroneState.rc.pitchAutoLevel &&
						ds.trickMode == DroneState.TrickMode.DISABLED ) 
					{
						if(Math.abs(ds.pitchPid.target) > CTRL_ERR)
						{
							CmdSetPitch cmd = new CmdSetPitch(0);
							DroneCommander.instance().addCmd(cmd);

							if(LOG_CTRL_CMD)
							{
								System.out.println("pitchAutoLevel");
							}
						}
					}
				}
				else if( Math.abs(curPitchRateTarget - pitchRateTarget) > CTRL_ERR)
				{
					cmdPitchRate.setTarget((float)pitchRateTarget);
					DroneCommander.instance().addCmd(cmdPitchRate);

					if(LOG_CTRL_CMD)
					{
						System.out.print(pitchRateLog);
						System.out.println(Math.toDegrees(pitchRateTarget));
					}
				}

				if(rollRateTarget == 0 && ds.rollPid.enabled && ds.rollPidFlag > 0)
				{
					if( DroneState.rc.rollAutoLevel &&
						ds.trickMode == DroneState.TrickMode.DISABLED ) 
					{
						if(Math.abs(ds.rollPid.target) > CTRL_ERR)
						{
							CmdSetRoll cmd = new CmdSetRoll(0);
							DroneCommander.instance().addCmd(cmd);

							if(LOG_CTRL_CMD)
							{
								System.out.println("rollAutoLevel");
							}
						}
					}
				}
				else if(Math.abs(curRollRateTarget - rollRateTarget) > CTRL_ERR)
				{
					cmdRollRate.setTarget((float)rollRateTarget);
					DroneCommander.instance().addCmd(cmdRollRate);

					if(LOG_CTRL_CMD)
					{
						System.out.print(rollRateLog);
						System.out.println(Math.toDegrees(rollRateTarget));
					}
				}

				break;
			case MOVE_BY_PITCH_ROLL:
				double pitchTarget = Math.toRadians(-pwrFwd * DroneState.rc.moveDelta);
				double rollTarget = Math.toRadians(pwrRight * DroneState.rc.moveDelta);

				if(Math.abs(pitchTarget) < Math.toRadians(MIN_PITCH_ROLL))
				{
					pitchTarget = 0;
				}

				if(Math.abs(rollTarget) < Math.toRadians(MIN_PITCH_ROLL))
				{
					rollTarget = 0;
				}

				if(	pitchTarget == 0 && 
					ds.velocityXPid.enabled &&
					ds.veloXPidFlag > 0)
				{
					; 	// prevent to send discontinuous pitch = 0 if veloPid is allowed
				}
				else if( Math.abs(ds.pitchPid.target - pitchTarget) > CTRL_ERR && ds.pitchPid.enabled && ds.pitchPidFlag > 0)
				{
					CmdSetPitch cmd = new CmdSetPitch((float)pitchTarget);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD) {
						System.out.print("set pitch = ");
						System.out.println(Math.toDegrees(pitchTarget));
					}
				}

				if(rollTarget == 0 && ds.velocityYPid.enabled && ds.veloYPidFlag > 0)
				{
					; 	// prevent to send discontinuous roll = 0 if veloPid is allowed
				}
				else if( Math.abs(ds.rollPid.target - rollTarget) > CTRL_ERR && ds.rollPid.enabled && ds.rollPidFlag > 0)
				{
					CmdSetRoll cmd = new CmdSetRoll((float)rollTarget);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD)
					{
						System.out.print("set roll = ");
						System.out.println(Math.toDegrees(rollTarget));
					}
				}
				break;
			case MOVE_BY_VELOCITY:
				double veloXTarget = pwrFwd * DroneState.rc.moveDelta;
				double veloYTarget = pwrRight * DroneState.rc.moveDelta;

				if(Math.abs(veloXTarget) < MIN_VELOCITY_XY)
					veloXTarget = 0;
				if(Math.abs(veloYTarget) < MIN_VELOCITY_XY)
					veloYTarget = 0;

				if(veloXTarget == 0 && ds.posNorthPid.enabled && ds.posNorthPidFlag > 0)
				{
					; 	// prevent to send discontinuous velo = 0 if posPid is allowed
				}
				else if( Math.abs(ds.velocityXPid.target - veloXTarget) > CTRL_ERR && ds.velocityXPid.enabled && ds.veloXPidFlag > 0)
				{
					CmdSetVelocityX cmd = new CmdSetVelocityX((float)veloXTarget);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD)
					{
						System.out.print("set velocity-x = ");
						System.out.println(veloXTarget);
					}
				}

				if(veloYTarget == 0 && ds.posEastPid.enabled && ds.posEastPidFlag > 0)
				{
					; 	// prevent to send discontinuous velo = 0 if posPid is allowed
				}
				else if( Math.abs(ds.velocityYPid.target - veloYTarget) > CTRL_ERR && ds.velocityYPid.enabled && ds.veloYPidFlag > 0)
				{
					CmdSetVelocityY cmd = new CmdSetVelocityY((float)veloYTarget);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD)
					{
						System.out.print("set velocity-y = ");
						System.out.println(veloYTarget);
					}
				}
				break;
		}
	}

	/** Sets new target of yawRatePid,headingPid
	 *  depending from DroneState.rc.rotateBy
	 *
	 * @param pwr Rotating power from -1 to 1. New PID target will be DroneState.rc.rotateDelta * pwr
	 */
	public void rotateDrone(double pwr)
	{
		DroneState ds = DroneTelemetry.instance().getDroneState();

		if(ds.motorsEnabled == false || ds.stabilizationEnabled == false)
			return;
		
		if(antiTurtleEnabled)
			return;
		
		checkCmdQueueSize();
		
		DroneState.RemoteCtrl.RotateBy rotateBy = DroneState.rc.rotateBy;
		float rotateDelta = DroneState.rc.rotateDelta;
		
		if(ds.trickMode != DroneState.TrickMode.DISABLED)
		{
			if(rotateBy == DroneState.RemoteCtrl.RotateBy.ROTATE_BY_HEADING)
			{
				rotateDelta = DroneState.rc.trickDelta;
			}
			rotateBy = DroneState.RemoteCtrl.RotateBy.ROTATE_BY_Z_RATE;
			
		}

		switch(rotateBy)
		{
			case ROTATE_BY_Z_RATE:
			case ROTATE_BY_YAW_RATE:
				double yawRateTarget = Math.toRadians(pwr * rotateDelta);
				double curYawRateTarget = ds.yawRateTarget;
				String yawRateLog = "set yawRate = ";
				AbstractCmdSetPidTarget cmdYawRate = new CmdSetYawRate(0);
				
				if(rotateBy == RotateBy.ROTATE_BY_Z_RATE)
				{
					curYawRateTarget = ds.zRatePid.target;
					yawRateLog = "set zRate = ";
					cmdYawRate = new CmdSetZRate(0);
				}

				if(Math.abs(yawRateTarget) < Math.toRadians(MIN_YAW_RATE))
				{
					yawRateTarget = 0;
				}

				if(yawRateTarget == 0 && ds.headingPid.enabled && ds.headingPidFlag > 0)
				{
					;    // prevent to send discontinuous yawRate = 0 if headingPid is allowed
				}
				else if( Math.abs(curYawRateTarget - yawRateTarget) > CTRL_ERR )
				{
					cmdYawRate.setTarget((float)yawRateTarget);
					DroneCommander.instance().addCmd(cmdYawRate);

					if(LOG_CTRL_CMD)
					{
						System.out.print(yawRateLog);
						System.out.println(Math.toDegrees(yawRateTarget));
					}
				}
				break;
			case ROTATE_BY_HEADING:
				double delta = pwr * (double)DroneState.rc.rotateDelta;
				float heading = (float)(ds.headingPid.targetDeg + delta);
				if(heading < 0)
					heading += 360.f;
				else if(heading > 360.f)
					heading -= 360.f;
					
				if(ds.headingPid.enabled && ds.headingPidFlag > 0)
				{
					CmdSetHeading cmd = new CmdSetHeading((float)Math.toRadians(heading));
					DroneCommander.instance().addCmd(cmd);
					
					if(LOG_CTRL_CMD)
					{
						System.out.print("set heading = ");
						System.out.println(heading);
					}
				}
				break;
		}
	}
	
	private void checkCmdQueueSize()
	{
		// if we have too many commands we get big control lag
		// this is can be if we connect slow modem
		// try to fix it by clear commands queue
		if(this.getCmdCount() > DroneState.misc.maxCtrlCmdsInQueue)
		{
			synchronized(mCmdLock)
			{	
				mCmds.clear();
				System.out.println("checkCmdQueueSize: clear cmd queue");
			}	
		}
	}

	/** Sets new target of velocityZPid,altPid or DroneState.baseGas
	 *  depending from DroneState.rc.liftBy
	 *
	 * @param pwr Lift power from -1 to 1. New PID target will be DroneState.rc.liftDelta * pwr
	 */
	public void liftDrone(double pwr)
	{
		DroneState ds = DroneTelemetry.instance().getDroneState();

		if(ds.motorsEnabled == false)
			return;
		
		if(antiTurtleEnabled)
			return;
		
		checkCmdQueueSize();
		
		DroneState.RemoteCtrl.LiftBy liftBy = DroneState.rc.liftBy;
		
		if(ds.trickMode != DroneState.TrickMode.DISABLED)
		{
			liftBy = DroneState.RemoteCtrl.LiftBy.LIFT_BY_GAS;
		}

		switch(liftBy)
		{
			case LIFT_BY_VELOCITY:
				double vv = pwr * DroneState.rc.liftDelta;

				if(Math.abs(vv) < MIN_VELOCITY_Z)
					vv = 0;

				if(ds.velocityZPid.enabled == false)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_ENABLE_VELOCITY_Z_PID);
					return;
				}

				if(ds.stabilizationEnabled == false && vv > MIN_VELOCITY_Z)
				{
					//CmdEnableStabilization cmd = new CmdEnableStabilization(true);
					//DroneCommander.instance().addCmd(cmd);
					CmdVeloZTakeoff cmd = new CmdVeloZTakeoff((float)DroneState.VeloZPid.takeoffErrSum);
					DroneCommander.instance().addCmd(cmd);
				}
				else if(vv == 0 && ds.altPid.enabled && ds.altPidFlag > 0)
				{
					;    // prevent to send discontinuous velocityZ = 0 if altPid is allowed
				}
				else if( Math.abs(ds.velocityZPid.target - vv) > CTRL_ERR)
				{
					CmdSetVelocityZ cmd = new CmdSetVelocityZ((float)vv);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD)
					{
						System.out.print("set velocity-z = ");
						System.out.println(vv);
					}
				}
				break;
			case LIFT_BY_GAS:
				//double middleGas = (double)(DroneState.rc.maxGas - DroneState.rc.minGas) / 2.0;
			case LIFT_BY_GAS_FORSAGE:
				double middleGas = (double)DroneState.rc.middleGas;
				double maxMiddleGas = (double)DroneState.rc.maxMiddleGas;
				double gas = 0;
				double gasAddon = 0;
								
				if(pwr > 0)
					gas = pwr*((double)DroneState.rc.maxGas - middleGas) + middleGas;
				else
					gas = -pwr*((double)DroneState.rc.minGas - middleGas) + middleGas;
				
				// when battery gets low middle gas have to be bigger
				// to correct this we use linear approximation of
				// dependency middle gas from battery voltage
				
				if(maxMiddleGas > middleGas)
					gasAddon = (maxMiddleGas - middleGas) * (ds.battery.maxVoltage - ds.battery.voltage) / (ds.battery.maxVoltage - ds.battery.minVoltage);
				
				gas += gasAddon;

				if(ds.velocityZPid.enabled && ds.trickMode == DroneState.TrickMode.DISABLED)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DISABLE_VELOCITY_Z_PID);
					return;
				}

				if(ds.stabilizationEnabled == false)
				{
					//if(pwr > 0)	//stick up enables drone
					if(Math.abs(pwr) > 0.2)	//stick up or down enables drone
					{
						CmdEnableStabilization cmd = new CmdEnableStabilization(true);
						DroneCommander.instance().addCmd(cmd);
					}
				}
				else if(ds.baseGas != (int)gas)
				{
					CmdSetBaseGas cmd = new CmdSetBaseGas((int) gas);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD)
					{
						System.out.print("set baseGas = ");
						System.out.println(gas);
					}
				}
				break;
			case LIFT_BY_GAS_DELTA:
				double delta = pwr * (double)DroneState.rc.liftDelta;
				gas = (double)ds.baseGas + delta;

				if(ds.velocityZPid.enabled)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DISABLE_VELOCITY_Z_PID);
					return;
				}
				
				if(ds.baseGas == 0 && gas > 0)	// first time gas up
				{
					gas = DroneState.rc.middleGas;	// in this case we start from middle gas
				}
				
				if(gas < DroneState.rc.minGas)
					gas = DroneState.rc.minGas;
				if(gas > DroneState.rc.maxGas)
					gas = DroneState.rc.maxGas;

				if(ds.stabilizationEnabled == false)
				{
					//if(pwr > 0)
					if(Math.abs(pwr) > 0.2)	// stick up or down enables drone
					{
						CmdEnableStabilization cmd = new CmdEnableStabilization(true);
						DroneCommander.instance().addCmd(cmd);
					}
				}
				else if(ds.baseGas != (int)gas)
				{
					CmdSetBaseGas cmd = new CmdSetBaseGas((int) gas);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD)
					{
						System.out.print("set baseGas = ");
						System.out.println(gas);
					}
				}
				break;
			case LIFT_BY_ALT:
				delta = pwr * (double)DroneState.rc.liftDelta;
				double alt = (double)ds.altPid.target + delta;

				if(ds.velocityZPid.enabled == false)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_ENABLE_VELOCITY_Z_PID);
					return;
				}
				
				if(ds.altPid.enabled == false)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_ENABLE_ALT_PID);
					return;
				}

				if(ds.stabilizationEnabled == false)
				{
					//CmdEnableStabilization cmd = new CmdEnableStabilization(true);
					//DroneCommander.instance().addCmd(cmd);
					CmdVeloZTakeoff cmd = new CmdVeloZTakeoff((float)DroneState.VeloZPid.takeoffErrSum);
					DroneCommander.instance().addCmd(cmd);
				}
				else if( Math.abs(ds.altPid.target - alt) > CTRL_ERR)
				{
					CmdSetAltitude cmd = new CmdSetAltitude((float)alt);
					DroneCommander.instance().addCmd(cmd);
					
					if(LOG_CTRL_CMD)
					{
						System.out.print("set alt = ");
						System.out.println(alt);
					}
				}
				break;
		}
	}
	
	public void setTrickMode(DroneState.TrickMode mode)
	{
		DroneState ds = DroneTelemetry.instance().getDroneState();
		
		if(antiTurtleEnabled)
			return;
		
		if(ds.trickMode != mode)
		{
			CmdSetTrickMode cmd = new CmdSetTrickMode(mode);
			DroneCommander.instance().addCmd(cmd);
			
			System.out.println("setTrickMode = " + mode.toString());
		}
	}
	
	public void antiTurtle(boolean enabled, int gas)
	{
		DroneState ds = DroneTelemetry.instance().getDroneState();
		
		if(ds.stabilizationEnabled || ds.motorsEnabled == false)
			return;
		
		if(antiTurtleEnabled != enabled)
		{
			if(enabled)
			{
				antiTurtleToggle++;
				if(antiTurtleToggle > 2)
					antiTurtleToggle = 0;
			}
			else
			{
				CmdSetMotorsGas cmd = new CmdSetMotorsGas(0,0,0,0);
				this.addCmd(cmd);
			}
			
			CmdSetMotorsDir cmdDir = new CmdSetMotorsDir(enabled);
			this.addCmd(cmdDir);
			
			antiTurtleEnabled = enabled;

			// wait motors has been changed direction
			try
			{
				Thread.sleep(250);
			}
			catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}
		
		if(antiTurtleEnabled)
		{
			switch(antiTurtleToggle)
			{
			case 0:
				if(ds.motorGas[0] != gas || ds.motorGas[1] != gas)
				{
					CmdSetMotorsGas cmd = new CmdSetMotorsGas(gas,gas,0,0);
					this.addCmd(cmd);
				}
				break;
			case 1:
				if(ds.motorGas[2] != gas || ds.motorGas[3] != gas)
				{
					CmdSetMotorsGas cmd = new CmdSetMotorsGas(0,0,gas,gas);
					this.addCmd(cmd);
				}
				break;
			case 2:
				if(ds.motorGas[0] != gas || ds.motorGas[1] != gas || ds.motorGas[2] != gas || ds.motorGas[3] != gas)
				{
					CmdSetMotorsGas cmd = new CmdSetMotorsGas(gas,gas,gas,gas);
					this.addCmd(cmd);
				}
				break;
			}
		}
	}
	
	public void saveDefaultCfg()
	{
		CmdSaveDefaultCfg cmd = new CmdSaveDefaultCfg();
		this.addCmd(cmd);
	}
	
	public void loadDefaultCfg()
	{
		CmdLoadDefaultCfg cmd = new CmdLoadDefaultCfg();
		this.addCmd(cmd);
	}
	
	public void setCameraAngle(int angle)
	{
		CmdSetCameraAngle cmd = new CmdSetCameraAngle(angle);
		this.addCmd(cmd);
	}
	
	public void enableMotors(boolean enable)
	{
		CmdSwitchMotors cmd1 = new CmdSwitchMotors(enable);
		this.addCmd(cmd1);
		CmdEnableStabilization cmd2 = new CmdEnableStabilization(false);
		this.addCmd(cmd2);
	}
	
	public void startVideo()
	{
		CmdStartStopVideo cmd = new CmdStartStopVideo(true);
		this.addCmd(cmd);	
	}
	
	public void stopVideo()
	{
		CmdStartStopVideo cmd = new CmdStartStopVideo(false);
		this.addCmd(cmd);
	}
	
	public void takePhoto()
	{
		CmdTakePhoto cmd = new CmdTakePhoto();
		this.addCmd(cmd);
	}
}
