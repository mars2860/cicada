package main;

import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import copter.Alarm;
import copter.AlarmCenter;
import copter.DroneState;
import main.Settings.WndState;
import net.miginfocom.swing.MigLayout;

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
				mMotorsGui.setVisible(true);
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
		
	private class OnAlarmUpdate implements Observer
	{
		@Override
		public void update(Observable o, Object arg)
		{
			mlbAlarmText.setText(AlarmCenter.instance().getAlarmText());
		
			switch(AlarmCenter.instance().getAlarmLevel())
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
	
	private class OnBtnConnect implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			DroneState ds = Settings.instance().getDroneSettings();
			ds.sendSettingsToDrone();
		}
	}

	private MotorsGui mMotorsGui;
	private StatusGui mStatusGui;
	private SensorsGui mSensorsGui;
	private ChartsGui mChartsGui;
	private SettingsGui mSettingsGui;
	private RemoteControlGui mRemoteControlGui;
	
	private JLabel mlbAlarmIcon;
	private JLabel mlbAlarmText;
	
	private JButton mbtnConnect;
	private JButton mbtnMotors;
	private JButton mbtnCharts;
	private JButton mbtnSensors;
	private JButton mbtnStatus;
	private JButton mbtnSettings;
	private JButton mbtnRemoteControl;
	
	public StartGui()
	{
		super("Start", 504, 176);
		this.createUI();
		
		AlarmCenter.instance().addObserver(new OnAlarmUpdate());
		AlarmCenter.instance().setAlarm(Alarm.COPTER_NOT_FOUND);
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

		tb.add(mbtnConnect);
		tb.add(mbtnStatus);
		tb.add(mbtnMotors);
		tb.add(mbtnSensors);
		tb.add(mbtnCharts);
		tb.add(mbtnSettings);
		tb.add(mbtnRemoteControl);
		
		return tb;
	}
	
	private JPanel createAlarmPanel()
	{
		JPanel pnlAlarm = new JPanel(new MigLayout("","[]10[grow]","[]"));
		pnlAlarm.setBorder(new TitledBorder(ResBox.text("ALARMS")));
		
		mlbAlarmIcon = new JLabel();
		mlbAlarmText = new JLabel();
		
		mlbAlarmIcon.setIcon(ResBox.icon("OK"));
		mlbAlarmText.setText(AlarmCenter.instance().getAlarmText());
		
		pnlAlarm.add(mlbAlarmIcon);
		pnlAlarm.add(mlbAlarmText,"grow");
		
		return pnlAlarm;
	}

	@Override
	protected WndState loadWndState()
	{
		return Settings.instance().getMainWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		Settings.instance().setMainWnd(ws);
	}
}
