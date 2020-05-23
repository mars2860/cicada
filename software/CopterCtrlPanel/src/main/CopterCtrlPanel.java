package main;

import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Locale;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import copter.Alarm;
import copter.AlarmCenter;
import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.commands.CmdCalibrateAccel;
import copter.commands.CmdCalibrateGyro;
import copter.commands.CmdCalibrateMagnet;
import copter.commands.CmdResetAltitude;
import copter.commands.CmdSetAltPid;
import copter.commands.CmdSetPeriods;
import copter.commands.CmdSetPitchPid;
import copter.commands.CmdSetRollPid;
import copter.commands.CmdSetYawPid;

import net.miginfocom.swing.MigLayout;

public class CopterCtrlPanel implements WindowListener
{
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
			SettingsDlg dlg = new SettingsDlg(CopterCtrlPanel.this);
			dlg.setVisible(true);
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
			int dx = Settings.instance().getAccelXOffset();
			int dy = Settings.instance().getAccelYOffset();
			int dz = Settings.instance().getAccelZOffset();
			
			CmdCalibrateAccel cmd1 = new CmdCalibrateAccel(dx,dy,dz);
			
			dx = Settings.instance().getGyroXOffset();
			dy = Settings.instance().getGyroYOffset();
			dz = Settings.instance().getGyroZOffset();
			
			CmdCalibrateGyro cmd2 = new CmdCalibrateGyro(dx,dy,dz);
			
			dx = Settings.instance().getMagnetXOffset();
			dy = Settings.instance().getMagnetYOffset();
			dz = Settings.instance().getMagnetZOffset();
			float sx = Settings.instance().getMagnetXScale();
			float sy = Settings.instance().getMagnetYScale();
			float sz = Settings.instance().getMagnetZScale();
			
			CmdCalibrateMagnet cmd3 = new CmdCalibrateMagnet(dx,dy,dz,sx,sy,sz);
			
			boolean enabled = Settings.instance().getYawPidEnabled();
			float kp = Settings.instance().getYawPidKp();
			float ki = Settings.instance().getYawPidKi();
			float kd = Settings.instance().getYawPidKd();
			CmdSetYawPid cmd4 = new CmdSetYawPid(enabled,kp,ki,kd);
			
			enabled = Settings.instance().getPitchPidEnabled();
			kp = Settings.instance().getPitchPidKp();
			ki = Settings.instance().getPitchPidKi();
			kd = Settings.instance().getPitchPidKd();
			CmdSetPitchPid cmd5 = new CmdSetPitchPid(enabled,kp,ki,kd);
			
			enabled = Settings.instance().getRollPidEnabled();
			kp = Settings.instance().getRollPidKp();
			ki = Settings.instance().getRollPidKi();
			kd = Settings.instance().getRollPidKd();
			CmdSetRollPid cmd6 = new CmdSetRollPid(enabled,kp,ki,kd);
			
			enabled = Settings.instance().getAltPidEnabled();
			kp = Settings.instance().getAltPidKp();
			ki = Settings.instance().getAltPidKi();
			kd = Settings.instance().getAltPidKd();
			CmdSetAltPid cmd7 = new CmdSetAltPid(enabled,kp,ki,kd);
			
			dx = Settings.instance().getTelemetryPeriod();
			dy = Settings.instance().getPidPeriod();
			CmdSetPeriods cmd8 = new CmdSetPeriods(dx,dy);
			
			CmdResetAltitude cmd9 = new CmdResetAltitude();
			
			CopterCommander.instance().addCmd(cmd1);
			CopterCommander.instance().addCmd(cmd1);
			CopterCommander.instance().addCmd(cmd1);
			
			CopterCommander.instance().addCmd(cmd2);
			CopterCommander.instance().addCmd(cmd2);
			CopterCommander.instance().addCmd(cmd2);
			
			CopterCommander.instance().addCmd(cmd3);
			CopterCommander.instance().addCmd(cmd3);
			CopterCommander.instance().addCmd(cmd3);
			
			CopterCommander.instance().addCmd(cmd4);
			CopterCommander.instance().addCmd(cmd4);
			CopterCommander.instance().addCmd(cmd4);
			
			CopterCommander.instance().addCmd(cmd5);
			CopterCommander.instance().addCmd(cmd5);
			CopterCommander.instance().addCmd(cmd5);
			
			CopterCommander.instance().addCmd(cmd6);
			CopterCommander.instance().addCmd(cmd6);
			CopterCommander.instance().addCmd(cmd6);
			
			CopterCommander.instance().addCmd(cmd7);
			CopterCommander.instance().addCmd(cmd7);
			CopterCommander.instance().addCmd(cmd7);
			
			CopterCommander.instance().addCmd(cmd8);
			CopterCommander.instance().addCmd(cmd8);
			CopterCommander.instance().addCmd(cmd8);
			
			CopterCommander.instance().addCmd(cmd9);
			CopterCommander.instance().addCmd(cmd9);
			CopterCommander.instance().addCmd(cmd9);
		}
	}
	
	private class OnBtnPID implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			PIDlg dlg = new PIDlg(mMainFrame);
			dlg.setVisible(true);
		}
	}

	private JSavedFrame mMainFrame;
	private MotorsGui mMotorsGui;
	private StatusGui mStatusGui;
	private SensorsGui mSensorsGui;
	private ChartsGui mChartsGui;
	
	private JLabel mlbAlarmIcon;
	private JLabel mlbAlarmText;
	
	private JButton mbtnConnect;
	private JButton mbtnMotors;
	private JButton mbtnCharts;
	private JButton mbtnSensors;
	private JButton mbtnStatus;
	private JButton mbtnSettings;
			
	public CopterCtrlPanel()
	{
	}
	
	private void createUI()
	{
		mMainFrame = new JSavedFrame(ResBox.text("APP_TITLE"),1024,768);
		
		mMainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		mMainFrame.addWindowListener(this);
		
		mMainFrame.setLayout(new MigLayout("","[][grow][]","[][][grow]"));

		mMainFrame.add(this.createToolbarPanel(),"spanx,grow");
		mMainFrame.add(this.createAlarmPanel(),"spanx 2,grow");
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

		tb.add(mbtnConnect);
		tb.add(mbtnStatus);
		tb.add(mbtnMotors);
		tb.add(mbtnSensors);
		tb.add(mbtnCharts);
		tb.add(mbtnSettings);
		
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
	
	public JFrame getMainFrame()
	{
		return mMainFrame;
	}

	public void start()
	{
		if(mMainFrame != null)
			return;
		
		Settings.instance().load();
		ResBox.load();
		this.createUI();
			
		mMainFrame.setVisible(true);
			
		AlarmCenter.instance().deleteObservers();
		AlarmCenter.instance().addObserver(new OnAlarmUpdate());
		
		CopterTelemetry.instance().deleteObservers();

		AlarmCenter.instance().setAlarm(Alarm.COPTER_NOT_FOUND);
			
		try
		{
			CopterCommander.instance().start(Settings.instance().getCopterIp(), Settings.instance().getCopterCmdPort());
			CopterTelemetry.instance().start(Settings.instance().getCopterIp(), Settings.instance().getCopterTelemetryPort());
		}
		catch(UnknownHostException e)
		{
			showErrorMsg(ResBox.text("INVALID_HOST"));
			e.printStackTrace();
		}
		catch(SocketException e)
		{
			showErrorMsg(ResBox.text("SOCKET_NOT_OPEN"));
			e.printStackTrace();
		}
	}
	
	public void stop()
	{
		CopterCommander.instance().stop();
		CopterTelemetry.instance().stop();
		
		Settings.instance().save();
		
		if(mMainFrame != null)
		{
			mMainFrame.setVisible(false);
			mMainFrame = null;
		}
	}
	
	private void showErrorMsg(String text)
	{
		JOptionPane.showMessageDialog(mMainFrame, text, ResBox.text("ERROR"), JOptionPane.ERROR_MESSAGE);
	}

	public static void main(String[] args)
	{
		CopterCtrlPanel app = new CopterCtrlPanel();
		
		Locale.setDefault(Locale.US);
		
		app.start();
	}

	@Override
	public void windowOpened(WindowEvent e) {}

	@Override
	public void windowClosing(WindowEvent e)
	{
		this.stop();
	}

	@Override
	public void windowClosed(WindowEvent e) {}

	@Override
	public void windowIconified(WindowEvent e) {}

	@Override
	public void windowDeiconified(WindowEvent e) {}

	@Override
	public void windowActivated(WindowEvent e) {}

	@Override
	public void windowDeactivated(WindowEvent e) {}
}
