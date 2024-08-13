package pdl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.concurrent.Semaphore;

import pdl.commands.AbstractDroneCmd;
import pdl.commands.CmdEnableDynamicIp;
import pdl.commands.CmdEnableStabilization;
import pdl.commands.CmdEnableTrickMode;
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
import pdl.commands.CmdSetPeriods;
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
import pdl.commands.CmdSetPosNorthPid;
import pdl.commands.CmdSetPsk;
import pdl.commands.CmdSetPosEastPid;
import pdl.commands.CmdSetRollPid;
import pdl.commands.CmdSetXRatePid;
import pdl.commands.CmdSetVelocityXPid;
import pdl.commands.CmdSetVelocityYPid;
import pdl.commands.CmdSetVelocityZPid;
import pdl.commands.CmdSetYawRate;
import pdl.commands.CmdSetZRatePid;
import pdl.commands.CmdSetup;
import pdl.commands.CmdStartStopVideo;
import pdl.commands.CmdSwitchMotors;
import pdl.commands.CmdTakePhoto;
import pdl.commands.CmdVeloZTakeoff;

public class DroneCommander implements Runnable
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
	 *  This delay is needed to a drone have time to process last received command
	 */
	public static final int DELAY_BTW_CMDS = 20;
	
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
	private ArrayList<AbstractDroneCmd> mCmds;
	private Semaphore mListMutex;
	private Object objSenderSync;
	private boolean antiTurtleEnabled;
	private int antiTurtleToggle;
	
	private DroneCommander() 
	{
		mCmds = new ArrayList<AbstractDroneCmd>();
		mListMutex = new Semaphore(1);
		objSenderSync = new Object();
	};
	
	/** Start new thread to receive telemetry packets. If thread is started it will be restarted */
	public void start(String ip, int cmdPort) throws UnknownHostException, SocketException
	{
		if(mSenderRun || mSocket != null)
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
		
		try
		{
			synchronized(objSenderSync)
			{
				// Unblock commander thread
				objSenderSync.notify();
				//System.out.println("Wait until commander thread stops");
				// Wait until commander thread stops its work
				if(mSender != null && mSender.isAlive())
				{
					objSenderSync.wait();
				}
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
			mSocket = null;
		}

		DroneAlarmCenter.instance().clearAll();
		DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DRONE_NOT_FOUND);
	}
	
	/** @return false if the list of commands has the command with the same code */
	public boolean addCmd(AbstractDroneCmd cmd)
	{
		boolean result = true;
		try
		{
			mListMutex.acquire();
			
			for(AbstractDroneCmd c : mCmds)
			{
				if(c.getCode() == cmd.getCode())
				{
					System.out.println("command duplicate is in the commands list");
					result = false;
					break;
				}
			}
			
			if(result)
			{
				mCmds.add(cmd);
			}
		}
		catch(Exception e)
		{
			result = false;
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
		return result;
	}

	public int getCmdCount()
	{
		int result = 0;
		try
		{
			mListMutex.acquire();

			result = mCmds.size();
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
		finally
		{
			mListMutex.release();
		}
		return result;
	}

	@Override
	public void run()
	{
		AbstractDroneCmd cmd = null;
		int cmdIdx = 0;
		
		while(mSenderRun)
		{
			try
			{
				mListMutex.acquire();
				
				if(mCmds.size() > 0)
				{
					//cmdIdx = mCmds.size() - 1; // old code implements LIFO queue in this place
					cmdIdx = 0; // FIFO queue
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
					// give time to the drone to process a command
					Thread.sleep(DELAY_BTW_CMDS);
				}
				catch(IOException e)
				{
					DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_SEND_ERROR);
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
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_SEND_ERROR);
			System.out.println("Number of attempts exceeded for " + cmd.toString());
		}
	}
	
	public void sendSettingsToDrone(DroneState ds)
	{
		DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_SEND_ERROR);
		
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
		cmd = new CmdSetPeriods(DroneState.net.telemetryPeriod,0);
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
		
		// velupOffset
		CmdSetAccUpOffset cmdAccUpOffset = new CmdSetAccUpOffset(ds.velocityZPid.accUpOffset);
		addCmd(cmdAccUpOffset);
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
		
		DroneState.RemoteCtrl.MoveBy moveBy = DroneState.rc.moveBy;
		float moveDelta = DroneState.rc.moveDelta;
		
		if(ds.trickModeEnabled)
		{
			moveBy = DroneState.RemoteCtrl.MoveBy.MOVE_BY_PITCH_ROLL_RATE;
			moveDelta = DroneState.rc.trickDelta;
		}

		switch(moveBy)
		{
			case MOVE_BY_PITCH_ROLL_RATE:
				double pitchRateTarget = Math.toRadians(-pwrFwd * moveDelta);
				double rollRateTarget = Math.toRadians(pwrRight * moveDelta);

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
					; 	// prevent to send discontinuous pitchRate = 0 if pitchPid is allowed
				}
				else if( Math.abs(ds.pitchRateTarget - pitchRateTarget) > CTRL_ERR)
				{
					CmdSetPitchRate cmd = new CmdSetPitchRate((float)pitchRateTarget);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD)
					{
						System.out.print("set pitchRate = ");
						System.out.println(Math.toDegrees(pitchRateTarget));
					}
				}

				if(rollRateTarget == 0 && ds.rollPid.enabled && ds.rollPidFlag > 0)
				{
					; 	// prevent to send discontinuous rollRate = 0 if rollPid is allowed
				}
				else if(Math.abs(ds.rollRateTarget - rollRateTarget) > CTRL_ERR)
				{
					CmdSetRollRate cmd = new CmdSetRollRate((float)rollRateTarget);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD)
					{
						System.out.print("set rollRate = ");
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

				if(pitchTarget == 0 && ds.velocityXPid.enabled && ds.veloXPidFlag > 0)
				{
					; 	// prevent to send discontinuous pitch = 0 if veloPid is allowed
				}
				else if( Math.abs(ds.pitchPid.target - pitchTarget) > CTRL_ERR)
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
				else if( Math.abs(ds.rollPid.target - rollTarget) > CTRL_ERR)
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
				else if( Math.abs(ds.velocityXPid.target - veloXTarget) > CTRL_ERR)
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
				else if( Math.abs(ds.velocityYPid.target - veloYTarget) > CTRL_ERR)
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
		
		DroneState.RemoteCtrl.RotateBy rotateBy = DroneState.rc.rotateBy;
		float rotateDelta = DroneState.rc.rotateDelta;
		
		if(	ds.trickModeEnabled &&
			rotateBy != DroneState.RemoteCtrl.RotateBy.ROTATE_BY_YAW_RATE )
		{
			rotateBy = DroneState.RemoteCtrl.RotateBy.ROTATE_BY_YAW_RATE;
			rotateDelta = DroneState.rc.trickDelta;
		}

		switch(rotateBy)
		{
			case ROTATE_BY_YAW_RATE:
				double yawRateTarget = Math.toRadians(pwr * rotateDelta);

				if(Math.abs(yawRateTarget) < Math.toRadians(MIN_YAW_RATE))
				{
					yawRateTarget = 0;
				}

				if(yawRateTarget == 0 && ds.headingPid.enabled && ds.headingPidFlag > 0)
				{
					;    // prevent to send discontinuous yawRate = 0 if headingPid is allowed
				}
				else if( Math.abs(ds.yawRateTarget - yawRateTarget) > CTRL_ERR )
				{
					CmdSetYawRate cmd = new CmdSetYawRate((float)yawRateTarget);
					DroneCommander.instance().addCmd(cmd);

					if(LOG_CTRL_CMD)
					{
						System.out.print("set yawRate = ");
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
					
				CmdSetHeading cmd = new CmdSetHeading((float)Math.toRadians(heading));
				DroneCommander.instance().addCmd(cmd);
					
				if(LOG_CTRL_CMD)
				{
					System.out.print("set heading = ");
					System.out.println(heading);
				}
				break;
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
		
		DroneState.RemoteCtrl.LiftBy liftBy = DroneState.rc.liftBy;
		
		if(ds.trickModeEnabled)
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

				if(ds.velocityZPid.enabled && ds.trickModeEnabled == false)
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
	
	public void setTrickMode(boolean enabled)
	{
		DroneState ds = DroneTelemetry.instance().getDroneState();
		
		if(antiTurtleEnabled)
			return;
		
		if(ds.trickModeEnabled != enabled)
		{
			CmdEnableTrickMode cmd = new CmdEnableTrickMode(enabled);
			DroneCommander.instance().addCmd(cmd);
			
			System.out.println("setTrickMode = " + enabled);
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
