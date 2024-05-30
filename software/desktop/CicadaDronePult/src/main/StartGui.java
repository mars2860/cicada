package main;

import java.awt.Desktop;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Observable;
import java.util.Observer;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import main.AppSettings.WndState;
import net.miginfocom.swing.MigLayout;
import pdl.Alarm;
import pdl.DroneAlarmCenter;
import pdl.DroneCommander;
import pdl.DroneLog;
import pdl.DroneState;
import pdl.DroneTelemetry;
import pdl.commands.CmdResetAltitude;
import pdl.res.Profile;
import pdl.res.TextBox;

public class StartGui extends JSavedFrame
{
	private static final long serialVersionUID = 3294260254869672112L;

	private class OnBtnMotors implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mMotorsGui == null)
				mMotorsGui = new MotorsGui();
			
			if(mMotorsGui.isVisible() == false)
			{
				mMotorsGui = null;
				mMotorsGui = new MotorsGui();
				mMotorsGui.setVisible(true);
			}
		}	
	}
	
	private class OnBtnStatus implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mStatusGui == null)
				mStatusGui = new StatusGui();
			
			if(mStatusGui.isVisible() == false)
				mStatusGui.setVisible(true);
		}
	}
	
	private class OnBtnLocation implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mLocationGui == null)
				mLocationGui = new LocationGui();
			
			if(mLocationGui.isVisible() == false)
				mLocationGui.setVisible(true);
		}
	}
	
	private class OnBtnSensors implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mSensorsGui == null)
				mSensorsGui = new SensorsGui();
			
			if(mSensorsGui.isVisible() == false)
				mSensorsGui.setVisible(true);
		}
	}
	
	private class OnBtnCharts implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mChartsGui == null)
				mChartsGui = new ChartsGui();
			
			if(mChartsGui.isVisible() == false)
				mChartsGui.setVisible(true);
		}
	}
	
	private class OnBtnSettings implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mSettingsGui == null)
				mSettingsGui = new SettingsGui();
			
			if(mSettingsGui.isVisible() == false)
				mSettingsGui.setVisible(true);
		}
	}
	
	private class OnBtnRemoteControl implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mRemoteControlGui == null)
				mRemoteControlGui = new RemoteControlGui();
			
			if(mRemoteControlGui.isVisible() == false)
				mRemoteControlGui.setVisible(true);
		}
	}
	
	private class OnBtnLog implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mLogGui == null)
				mLogGui = new LogGui();
			
			if(mLogGui.isVisible() == false)
				mLogGui.setVisible(true);
		}
	}
	
	private class OnBtnInfo implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			File file = new File("./cicada-manual-ru.pdf");
			try
			{
				Desktop.getDesktop().open(file);
			}
			catch (IOException e1)
			{
				e1.printStackTrace();
			}
		}
	}
		
	private class OnAlarmUpdate implements Observer
	{
		@Override
		public void update(Observable o, Object arg)
		{
			Alarm alarm = DroneAlarmCenter.instance().getAlarm();
			printAlarm(alarm);
		}
	}
	
	private class OnTelemetryUpdate implements Observer
	{
		private long timestamp;
		
		@Override
		public void update(Observable o, Object arg)
		{
			if(	System.currentTimeMillis() - timestamp < 100)
				return;
				
			timestamp = System.currentTimeMillis();
				
			DroneState ds = DroneTelemetry.instance().getDroneState();
			
			// wait first 20 secs for drone system has been stabilized and initialized
			// this prevent from false messages at startup
			if(ds.timestamp < 20000000)
			{
				return;
			}
			
			speechState(ds);
		}
	}
	
	private class OnBtnConnect implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(DroneAlarmCenter.instance().getAlarm(Alarm.ALARM_CONNECTING))
				return;
			
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_CONNECTING);
			
			Thread thread = new Thread(new Runnable()
			{
				@Override
				public void run()
				{
					try
					{
						DroneTelemetry.instance().resetFlyTime();
						DroneState ds = Profile.instance().getDroneSettings();
						// Restart Telemetry/Commander/Log services because we can change Ip address of drone in settings
						DroneLog.instance().start(DroneState.net.ip, DroneState.net.logPort);
						DroneTelemetry.instance().start(DroneState.net.ip, DroneState.net.telemetryPort);
						DroneCommander.instance().start(DroneState.net.ip, DroneState.net.cmdPort);
						// DroneCommander.start clear all alarms
						DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_CONNECTING);
						// Send new settings
						DroneCommander.instance().sendSettingsToDrone(ds);
						CmdResetAltitude cmd = new CmdResetAltitude();
						// we can lose udp packet thus we send it three times
						DroneCommander.instance().addCmd(cmd);
						//DroneCommander.instance().addCmd(cmd);
						//DroneCommander.instance().addCmd(cmd);
						DroneTelemetry.instance().clearBlackBox();
						
						DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_CONNECTING);
					}
					catch(UnknownHostException e)
					{
						CicadaDronePultApp.showErrorMsg(StartGui.this,ResBox.text("INVALID_HOST"));
						DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_CONNECTING);
						e.printStackTrace();
					}
					catch(SocketException e)
					{
						CicadaDronePultApp.showErrorMsg(StartGui.this,ResBox.text("SOCKET_NOT_OPEN"));
						DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_CONNECTING);
						e.printStackTrace();
					}
				}
			});
			thread.start();
		}
	}

	private MotorsGui mMotorsGui;
	private StatusGui mStatusGui;
	private LocationGui mLocationGui;
	private SensorsGui mSensorsGui;
	private ChartsGui mChartsGui;
	private SettingsGui mSettingsGui;
	private RemoteControlGui mRemoteControlGui;
	private LogGui mLogGui;
	
	private JLabel mlbAlarmIcon;
	private JLabel mlbAlarmText;
	
	private JButton mbtnConnect;
	private JButton mbtnMotors;
	private JButton mbtnCharts;
	private JButton mbtnSensors;
	private JButton mbtnStatus;
	private JButton mbtnLocation;
	private JButton mbtnSettings;
	private JButton mbtnRemoteControl;
	private JButton mbtnLog;
	private JButton mbtnInfo;
	
	public StartGui()
	{
		super("Start", 536, 176);
		this.createUI();
		
		DroneAlarmCenter.instance().addObserver(new OnAlarmUpdate());
		DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DRONE_NOT_FOUND);
		DroneTelemetry.instance().addObserver(new OnTelemetryUpdate());
	}
	
	private void createUI()
	{
		this.setTitle(ResBox.text("APP_TITLE"));
		this.setIconImage(ResBox.icon("PROPELLER").getImage());
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		this.setLayout(new MigLayout("","[][grow][]","[][][grow]"));

		this.add(this.createToolbarPanel(),"spanx,grow");
		this.add(this.createAlarmPanel(),"spanx 2,grow");
	}
	
	private JPanel createToolbarPanel()
	{
		JPanel tb = new JPanel(new MigLayout());
		
		Insets zeroInsets = new Insets(0,0,0,0);
		
		mbtnConnect = new JButton(ResBox.icon("KEY"));
		mbtnConnect.setMargin(zeroInsets);
		mbtnConnect.setToolTipText(ResBox.text("CONNECT"));
		mbtnConnect.addActionListener(new OnBtnConnect());
		
		mbtnStatus = new JButton(ResBox.icon("GAUGE"));
		mbtnStatus.setMargin(zeroInsets);
		mbtnStatus.setToolTipText(ResBox.text("STATUS"));
		mbtnStatus.addActionListener(new OnBtnStatus());
		
		mbtnLocation = new JButton(ResBox.icon("LOCATION"));
		mbtnLocation.setMargin(zeroInsets);	
		mbtnLocation.setToolTipText(ResBox.text("LOCATION"));
		mbtnLocation.addActionListener(new OnBtnLocation());
		
		mbtnMotors = new JButton(ResBox.icon("PROPELLER"));
		mbtnMotors.setMargin(zeroInsets);
		mbtnMotors.setToolTipText(ResBox.text("MOTORS"));
		mbtnMotors.addActionListener(new OnBtnMotors());
		
		mbtnSensors = new JButton(ResBox.icon("SENSORS"));
		mbtnSensors.setMargin(zeroInsets);
		mbtnSensors.setToolTipText(ResBox.text("SENSORS"));
		mbtnSensors.addActionListener(new OnBtnSensors());
		
		mbtnCharts = new JButton(ResBox.icon("CHARTS"));
		mbtnCharts.setMargin(zeroInsets);
		mbtnCharts.setToolTipText(ResBox.text("CHARTS"));
		mbtnCharts.addActionListener(new OnBtnCharts());
		
		mbtnSettings = new JButton(ResBox.icon("SETTINGS"));
		mbtnSettings.setMargin(zeroInsets);
		mbtnSettings.setToolTipText(ResBox.text("SETTINGS"));
		mbtnSettings.addActionListener(new OnBtnSettings());
		
		mbtnRemoteControl = new JButton(ResBox.icon("REMOTE_CONTROL"));
		mbtnRemoteControl.setMargin(zeroInsets);
		mbtnRemoteControl.setToolTipText(ResBox.text("REMOTE_CONTROL"));
		mbtnRemoteControl.addActionListener(new OnBtnRemoteControl());
		
		mbtnLog = new JButton(ResBox.icon("LOG"));
		mbtnLog.setMargin(zeroInsets);
		mbtnLog.setToolTipText(ResBox.text("LOG"));
		mbtnLog.addActionListener(new OnBtnLog());
		
		mbtnInfo = new JButton(ResBox.icon("INFO"));
		mbtnInfo.setMargin(zeroInsets);
		mbtnInfo.setToolTipText(ResBox.text("INFO"));
		mbtnInfo.addActionListener(new OnBtnInfo());

		tb.add(mbtnConnect);
		tb.add(mbtnStatus);
		tb.add(mbtnLocation);
		tb.add(mbtnMotors);
		tb.add(mbtnSensors);
		tb.add(mbtnCharts);
		tb.add(mbtnSettings);
		tb.add(mbtnRemoteControl);
		tb.add(mbtnLog);
		tb.add(mbtnInfo);
		
		return tb;
	}
	
	private JPanel createAlarmPanel()
	{
		JPanel pnlAlarm = new JPanel(new MigLayout("","[]10[grow]","[]"));
		pnlAlarm.setBorder(new TitledBorder(ResBox.text("ALARMS")));
		
		mlbAlarmIcon = new JLabel();
		mlbAlarmText = new JLabel();
		
		mlbAlarmIcon.setIcon(ResBox.icon("OK"));
		mlbAlarmText.setText("");
		
		pnlAlarm.add(mlbAlarmIcon);
		pnlAlarm.add(mlbAlarmText,"grow");
		
		return pnlAlarm;
	}

	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getMainWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setMainWnd(ws);
	}
	
	private void printAlarm(Alarm alarm)
	{
		if(DroneAlarmCenter.instance().getAlarm(Alarm.ALARM_CONNECTING))
		{
			mlbAlarmText.setText(ResBox.text("ALARM_CONNECTING"));
			mlbAlarmIcon.setIcon(ResBox.icon("ERROR"));
			// don't show other Alarms when we try to connect
			return;
		}
		
		if(DroneAlarmCenter.instance().getAlarm(Alarm.ALARM_UNSUPPORTED_FIRMWARE))
		{
			mlbAlarmText.setText(ResBox.text("ALARM_UNSUPPORTED_FIRMWARE"));
			mlbAlarmIcon.setIcon(ResBox.icon("ERROR"));
			// don't show other Alarms if we have unsupported firmware
			return;
		}
		
		String alarmText = TextBox.get("SYSTEM_OK"); 
		
		if(alarm != null)
			alarmText = TextBox.get(alarm.name());
			
		mlbAlarmText.setText(alarmText);
	
		switch(DroneAlarmCenter.instance().getAlarmLevel())
		{
		case 0:
			mlbAlarmIcon.setIcon(ResBox.icon("OK"));
			break;
		case 1:
		default:
			mlbAlarmIcon.setIcon(ResBox.icon("ERROR"));
			break;
			
		}
	}
	
	private int speechAlt;
	private boolean speechBat15;	// to say bat15 one time
	
	private boolean oldMotorsEnabled;
	private boolean oldStabilizationEnabled;
	private boolean oldTrickModeEnabled;
	private boolean oldVideoState;
	
	private void speechState(DroneState ds)
	{
		if(DroneState.misc.sounds == false)
		{
			return;
		}
		
		if(ds.altitude < 1.5)
		{
			speechAlt = 0;
		}
		// it annoys when flying indoor
		/*if(ds.altitude < 0.5)
		{
			speechAlt = 0;
		}
				
		if(ds.altitude >= 1.0 && ds.altitude <= 1.5 && speechAlt != 1)
		{
			speechAlt = 1;
			ResBox.sound("ALT_1M").play();
		}
		if(ds.altitude >= 2.0 && ds.altitude <= 2.5 && speechAlt != 2)
		{
			speechAlt = 2;
			ResBox.sound("ALT_2M").play();
		}
		else*/if(ds.altitude >= 3.0 && ds.altitude <= 3.5 && speechAlt != 3)
		{
			speechAlt = 3;
			ResBox.sound("ALT_3M").play();
		}
		else if(ds.altitude >= 5.0 && ds.altitude <= 5.5 && speechAlt != 5)
		{
			speechAlt = 5;
			ResBox.sound("ALT_5M").play();
		}
		else if(ds.altitude >= 7.0 && ds.altitude <= 7.5 && speechAlt != 7)
		{
			speechAlt = 7;
			ResBox.sound("ALT_7M").play();
		}
		else if(ds.altitude >= 10.0 && ds.altitude <= 10.5 && speechAlt != 10)
		{
			speechAlt = 10;
			ResBox.sound("ALT_10M").play();
		}
		else if(ds.altitude >= 15.0 && ds.altitude <= 15.5 && speechAlt != 15)
		{
			speechAlt = 15;
			ResBox.sound("ALT_15M").play();
		}
		else if(ds.altitude >= 20.0 && ds.altitude <= 22.0 && speechAlt != 20)
		{
			speechAlt = 20;
			ResBox.sound("ALT_20M").play();
		}
		else if(ds.altitude >= 30.0 && ds.altitude <= 32.0 && speechAlt != 30)
		{
			speechAlt = 30;
			ResBox.sound("ALT_30M").play();
		}
		else if(ds.altitude >= 40.0 && ds.altitude <= 42.0 && speechAlt != 40)
		{
			speechAlt = 40;
			ResBox.sound("ALT_40M").play();
		}
		else if(ds.altitude >= 50.0 && ds.altitude <= 52.0 && speechAlt != 50)
		{
			speechAlt = 50;
			ResBox.sound("ALT_50M").play();
		}
					
		if(ds.motorsEnabled && !oldMotorsEnabled)
		{
			ResBox.sound("MOTORS_ENABLED").play();
					
			// play these sounds in other thread with some delay from motors_enabled sound
			// to prevent motors_enabled and these sounds simultaneously
			Timer tm = new Timer();
			tm.schedule(new TimerTask()
			{
				@Override
				public void run()
				{
					if(DroneAlarmCenter.instance().getAlarm() != null)
					{
						ResBox.sound("SYSTEM_BAD").play();
					}
					else
					{
						ResBox.sound("SYSTEM_OK").play();
					}	
				}
						
			}, 2000);
		}
		else if(!ds.motorsEnabled && oldMotorsEnabled)
		{
			ResBox.sound("MOTORS_DISABLED").play();
		}
					
		oldMotorsEnabled = ds.motorsEnabled;
				
		if(ds.stabilizationEnabled && !oldStabilizationEnabled)
		{
			ResBox.sound("STABILIZATION_ENABLED").play();
		}
		else if(!ds.stabilizationEnabled && oldStabilizationEnabled)
		{
			ResBox.sound("STABILIZATION_DISABLED").play();
		}
					
		oldStabilizationEnabled = ds.stabilizationEnabled;
					
		if(ds.trickModeEnabled && !oldTrickModeEnabled)
		{
			ResBox.sound("TRICK_MODE_ENABLED").play();
		}
		else if(!ds.trickModeEnabled && oldTrickModeEnabled)
		{
			ResBox.sound("TRICK_MODE_DISABLED").play();
		}
					
		oldTrickModeEnabled = ds.trickModeEnabled;
							
		if(ds.baseGas > DroneState.Motors.maxGas /8 )
		{
			if(	ds.battery.percent > 10.0 &&
				ds.battery.percent <= 15.0 && 
				speechBat15 == false)	// we say bat15 one time
			{
				ResBox.sound("BAT15").play();
				speechBat15 = true;
			}
			else if(ds.battery.percent > 5.0 && ds.battery.percent <= 10.0)
			{
				ResBox.sound("BAT10").play();
			}
			else if(ds.battery.percent <= 5.0)
			{
				ResBox.sound("BAT5").play();
			}
		}
		else
		{
			speechBat15 = false;
		}
			
		if(ds.rssi <= DroneAlarmCenter.WIFI_LOW_LEVEL)
		{
			ResBox.sound("LOW_RADIO_SIGNAL").play();
		}
		
		if(ds.videoState && !oldVideoState)
		{
			ResBox.sound("VIDEO_STARTED").play();
		}
		else if(!ds.videoState && oldVideoState)
		{
			ResBox.sound("VIDEO_STOPED").play();
		}
		oldVideoState = ds.videoState;
	}
}
