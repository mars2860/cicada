package main;

import java.awt.Desktop;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;
import java.util.Observable;
import java.util.Observer;

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
import pdl.DroneState;
import pdl.DroneTelemetry;
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
			
			if(DroneAlarmCenter.instance().getAlarm(Alarm.ALARM_CONNECTING))
				return;
			
			DroneState ds = DroneTelemetry.instance().getDroneState();

			DroneTelemetry.instance().speakDroneState(ds, pdlSoundProvider);
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
			
			// wait first 5 secs for drone system has been stabilized and initialized
			// this prevent from false messages at startup
			if(DroneTelemetry.instance().getDroneConnectionTime() < 5000)
			{
				return;
			}
			
			DroneTelemetry.instance().speakDroneState(ds, pdlSoundProvider);
		}
	}
	
	private class OnBtnConnect implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mLogGui != null)
			{
				mLogGui.clearLog();
			}
			
			DroneCommander.instance().connect();
		}
	}
	
	private class OnBtnRadioStatus implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mRadioGui == null)
			{
				mRadioGui = new RadioGui();
			}
			
			mRadioGui.setVisible(true);
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
	private RadioGui mRadioGui;
	
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
	private JButton mbtnRadioStatus;
	
	private PDLSoundProvider pdlSoundProvider;
	
	public StartGui()
	{
		super("Start", 536, 176);
		
		pdlSoundProvider = new PDLSoundProvider();
		
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
		
		mbtnRadioStatus = new JButton(ResBox.icon("RADIO"));
		mbtnRadioStatus.setMargin(zeroInsets);
		mbtnRadioStatus.setToolTipText(ResBox.text("RADIO_STATUS"));
		mbtnRadioStatus.addActionListener(new OnBtnRadioStatus());

		tb.add(mbtnConnect);
		tb.add(mbtnStatus);
		tb.add(mbtnLocation);
		tb.add(mbtnRadioStatus);
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
}
