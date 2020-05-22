package main;

import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.text.DecimalFormat;
import java.util.Locale;
import java.util.Observable;
import java.util.Observer;

import javax.swing.ImageIcon;
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
			{
				mMotorsGui = new MotorsGui();
				mMotorsGui.setIconImage(mIconPropeller.getImage());
			}
			
			if(mMotorsGui.isVisible() == false)
				mMotorsGui.setVisible(true);
		}	
	}
	
	private class OnBtnStatus implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			// TODO Auto-generated method stub
			
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
	
	private class OnBtnSensors implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			SensorsGui gui = new SensorsGui(mMainFrame);
			gui.setVisible(true);
		}
	}
	
	private class OnBtnCharts implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			ChartsGui gui = new ChartsGui(mMainFrame);
			gui.setVisible(true);
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
				mlbAlarmIcon.setIcon(mIconOk);
				break;
			case 1:
			default:
				mlbAlarmIcon.setIcon(mIconError);
				break;
				
			}
		}
	}
	
	private class OnTelemetryUpdate implements Observer
	{
		private long timestamp;
		@Override
		public void update(Observable o, Object arg)
		{
			if(System.currentTimeMillis() - timestamp < 100)
				return;
			
			timestamp = System.currentTimeMillis();
			
			DecimalFormat fmt1 = new DecimalFormat();
			fmt1.setMaximumFractionDigits(2);
			fmt1.setMinimumFractionDigits(0);
			fmt1.setGroupingUsed(false);
			DecimalFormat fmt2 = new DecimalFormat();
			fmt2.setMaximumFractionDigits(0);
			fmt2.setGroupingUsed(false);
			
			CopterTelemetry.DroneState droneState = CopterTelemetry.instance().getDroneState();
			
			String batState = fmt1.format(droneState.battery.voltage) + "V/" + 
								fmt1.format(droneState.battery.percent) + "%";
			
			mlbBattery.setText(batState);
			mlbWifiLevel.setText(Integer.toString((int)droneState.wifiLevel));
			
			double yaw = Math.toDegrees(droneState.yawRad);
			double yawTarget = Math.toDegrees(droneState.yawRadRatePid.target);
			double pitch = Math.toDegrees(droneState.pitchRad);
			double pitchTarget = Math.toDegrees(droneState.pitchRadPid.target);
			double roll = Math.toDegrees(droneState.rollRad);
			double rollTarget = Math.toDegrees(droneState.rollRadPid.target);
			double heading = Math.toDegrees(droneState.headingRad);
			
			mlbYawValue.setText(fmt1.format(yaw) + "/" + fmt1.format(yawTarget));
			mlbPitchValue.setText(fmt1.format(pitch) + "/" + fmt1.format(pitchTarget));
			mlbRollValue.setText(fmt1.format(roll) + "/" + fmt1.format(rollTarget));
			mlbHeading.setText(fmt2.format(heading));
			mlbLoopTime.setText(fmt2.format(droneState.mainLoopTime));
			mlbTemperature.setText(fmt1.format(droneState.temperature));
			mlbPressure.setText(fmt1.format(droneState.pressure) + "/" +
								fmt1.format(droneState.seaLevel));
			mlbAltitude.setText(fmt1.format(droneState.altitude) + "/" +
								fmt1.format(droneState.altPid.target));
			
			mlbYaw.setIcon(rotateImageIcon(mIconYaw, yaw));
			mlbPitch.setIcon(rotateImageIcon(mIconPitch, pitch));
			mlbRoll.setIcon(rotateImageIcon(mIconRoll, roll));

			
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
	
	private JLabel mlbAlarmIcon;
	private JLabel mlbAlarmText;
	private JLabel mlbBattery;
	private JLabel mlbWifiLevel;
	private JLabel mlbYaw;
	private JLabel mlbYawValue;
	private JLabel mlbPitch;
	private JLabel mlbPitchValue;
	private JLabel mlbRoll;
	private JLabel mlbRollValue;
	private JLabel mlbHeading;
	private JLabel mlbLoopTime;
	private JLabel mlbTemperature;
	private JLabel mlbPressure;
	private JLabel mlbAltitude;
	
	private JButton mbtnConnect;
	private JButton mbtnMotors;
	private JButton mbtnCharts;
	private JButton mbtnSensors;
	private JButton mbtnStatus;
	private JButton mbtnSettings;
	
	private ImageIcon mIconOk;
	private ImageIcon mIconError;
	private ImageIcon mIconBat;
	private ImageIcon mIconWifi;
	private ImageIcon mIconYaw;
	private ImageIcon mIconPitch;
	private ImageIcon mIconRoll;
	private ImageIcon mIconHeading;
	private ImageIcon mIconLoopTime;
	private ImageIcon mIconTemperature;
	private ImageIcon mIconPressure;
	private ImageIcon mIconAltitude;
	private ImageIcon mIconPropeller;
	private ImageIcon mIconConnect;
	private ImageIcon mIconCharts;
	private ImageIcon mIconSensors;
	private ImageIcon mIconGauge;
	private ImageIcon mIconSettings;
		
	public CopterCtrlPanel()
	{
	}
	
	private void createUI()
	{
		mMainFrame = new JSavedFrame(Text.get("APP_TITLE"),1024,768);
		
		mMainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		mMainFrame.addWindowListener(this);
		
		mMainFrame.setLayout(new MigLayout("","[][grow][]","[][][grow]"));

		mMainFrame.add(this.createToolbarPanel(),"spanx,grow");
		mMainFrame.add(this.createAlarmPanel(),"spanx 2,grow");
		mMainFrame.add(this.createSettingsPanel(),"grow,wrap");
		mMainFrame.add(this.createStatusPanel(),"growy");
	}
	
	private JPanel createToolbarPanel()
	{
		JPanel tb = new JPanel(new MigLayout());
		
		Insets zeroInsets = new Insets(0,0,0,0);
		
		mbtnConnect = new JButton(mIconConnect);
		mbtnConnect.setMargin(zeroInsets);
		mbtnConnect.setToolTipText(Text.get("CONNECT"));
		mbtnConnect.addActionListener(new OnBtnConnect());
		
		mbtnStatus = new JButton(mIconGauge);
		mbtnStatus.setMargin(zeroInsets);
		mbtnStatus.setToolTipText(Text.get("STATUS"));
		mbtnStatus.addActionListener(new OnBtnStatus());
		
		mbtnMotors = new JButton(mIconPropeller);
		mbtnMotors.setMargin(zeroInsets);
		mbtnMotors.setToolTipText(Text.get("MOTORS"));
		mbtnMotors.addActionListener(new OnBtnMotors());
		
		mbtnSensors = new JButton(mIconSensors);
		mbtnSensors.setMargin(zeroInsets);
		mbtnSensors.setToolTipText(Text.get("SENSORS"));
		mbtnSensors.addActionListener(new OnBtnSensors());
		
		mbtnCharts = new JButton(mIconCharts);
		mbtnCharts.setMargin(zeroInsets);
		mbtnCharts.setToolTipText(Text.get("CHARTS"));
		mbtnCharts.addActionListener(new OnBtnCharts());
		
		mbtnSettings = new JButton(mIconSettings);
		mbtnSettings.setMargin(zeroInsets);
		mbtnSettings.setToolTipText(Text.get("SETTINGS"));
		mbtnSettings.addActionListener(new OnBtnSettings());

		tb.add(mbtnConnect);
		tb.add(mbtnStatus);
		tb.add(mbtnMotors);
		tb.add(mbtnSensors);
		tb.add(mbtnCharts);
		tb.add(mbtnSettings);
		
		return tb;
	}
	
	private JPanel createSettingsPanel()
	{
		JPanel pnlSettings = new JPanel(new MigLayout("","","[grow, center]"));
		pnlSettings.setBorder(new TitledBorder(Text.get("SETTINGS")));
		
		JButton btnConnect = new JButton(Text.get("CONNECT"));
		btnConnect.addActionListener(new OnBtnConnect());

		JButton btnSettings = new JButton(Text.get("SETTINGS"));
		btnSettings.addActionListener(new OnBtnSettings());
		
		JButton btnSensors = new JButton(Text.get("SENSORS"));
		btnSensors.addActionListener(new OnBtnSensors());
		
		JButton btnPID = new JButton(Text.get("PID"));
		btnPID.addActionListener(new OnBtnPID());
		
		JButton btnCharts = new JButton(Text.get("CHARTS"));
		btnCharts.addActionListener(new OnBtnCharts());
		
		pnlSettings.add(btnConnect);
		pnlSettings.add(btnSensors);
		pnlSettings.add(btnSettings);
		pnlSettings.add(btnPID);
		pnlSettings.add(btnCharts);

		return pnlSettings;
	}
	
	private JPanel createAlarmPanel()
	{
		JPanel pnlAlarm = new JPanel(new MigLayout("","[]10[grow]","[]"));
		pnlAlarm.setBorder(new TitledBorder(Text.get("ALARMS")));
		
		mlbAlarmIcon = new JLabel();
		mlbAlarmText = new JLabel();
		
		mlbAlarmIcon.setIcon(mIconOk);
		mlbAlarmText.setText(AlarmCenter.instance().getAlarmText());
		
		pnlAlarm.add(mlbAlarmIcon);
		pnlAlarm.add(mlbAlarmText,"grow");
		
		return pnlAlarm;
	}
	
	private JPanel createStatusPanel()
	{
		JPanel pnlStatus = new JPanel(new MigLayout("","[][90!]",""));
		pnlStatus.setBorder(new TitledBorder(Text.get("STATUS")));

		mlbBattery = new JLabel();
		//mlbBattery.setHorizontalAlignment(JLabel.CENTER);
		
		mlbWifiLevel = new JLabel();
		//mlbWifiLevel.setHorizontalAlignment(JLabel.CENTER);
		
		mlbYawValue = new JLabel();
		//mlbYawValue.setHorizontalAlignment(JLabel.CENTER);
		
		mlbPitchValue = new JLabel();
		//mlbPitchValue.setHorizontalAlignment(JLabel.CENTER);
		
		mlbRollValue = new JLabel();
		//mlbRollValue.setHorizontalAlignment(JLabel.CENTER);
		
		mlbHeading = new JLabel();
		//mlbHeading.setHorizontalAlignment(JLabel.CENTER);
		
		mlbYaw = new JLabel(mIconYaw);
		mlbPitch = new JLabel(mIconPitch);
		mlbRoll = new JLabel(mIconRoll);
		
		mlbLoopTime = new JLabel();
		//mlbLoopTime.setHorizontalAlignment(JLabel.CENTER);
		
		mlbTemperature = new JLabel();
		//mlbTemperature.setHorizontalAlignment(JLabel.CENTER);
		
		mlbPressure = new JLabel();
		//mlbPressure.setHorizontalAlignment(JLabel.CENTER);
		
		mlbAltitude = new JLabel();
		//mlbAltitude.setHorizontalAlignment(JLabel.CENTER);
		
		pnlStatus.add(new JLabel(mIconBat));
		pnlStatus.add(mlbBattery,"wrap");
		pnlStatus.add(new JLabel(mIconWifi));
		pnlStatus.add(mlbWifiLevel,"wrap");
		pnlStatus.add(mlbYaw);
		pnlStatus.add(mlbYawValue,"wrap");
		pnlStatus.add(mlbPitch);
		pnlStatus.add(mlbPitchValue,"wrap");
		pnlStatus.add(mlbRoll);
		pnlStatus.add(mlbRollValue,"wrap");
		pnlStatus.add(new JLabel(mIconHeading));
		pnlStatus.add(mlbHeading,"wrap");
		pnlStatus.add(new JLabel(mIconAltitude));
		pnlStatus.add(mlbAltitude,"wrap");
		pnlStatus.add(new JLabel(mIconTemperature));
		pnlStatus.add(mlbTemperature,"wrap");
		pnlStatus.add(new JLabel(mIconPressure));
		pnlStatus.add(mlbPressure,"wrap");
		pnlStatus.add(new JLabel(mIconLoopTime));
		pnlStatus.add(mlbLoopTime,"wrap");
		
		return pnlStatus;
	}
	
	public JFrame getMainFrame()
	{
		return mMainFrame;
	}
	
	private void loadImages()
	{
		mIconOk = loadIcon("images/ok.png");
		mIconError = loadIcon("images/error.png");
		mIconBat = loadIcon("images/battery.png");
		mIconWifi = loadIcon("images/wifi.png");
		mIconYaw = loadIcon("images/yaw.png");
		mIconPitch = loadIcon("images/pitch.png");
		mIconRoll = loadIcon("images/roll.png");
		mIconHeading = loadIcon("images/heading.png");
		mIconLoopTime = loadIcon("images/cputime.png");
		mIconTemperature = loadIcon("images/temperature.png");
		mIconPressure = loadIcon("images/pressure.png");
		mIconAltitude = loadIcon("images/altitude.png");
		mIconPropeller = loadIcon("images/propeller.png");
		mIconConnect = loadIcon("images/key.png");
		mIconCharts = loadIcon("images/charts.png");
		mIconSensors = loadIcon("images/sensors.png");
		mIconGauge = loadIcon("images/gauge.png");
		mIconSettings = loadIcon("images/settings.png");
	}
	
	private ImageIcon loadIcon(String path)
	{
		return loadIcon(path,24,24);
	}
	
	private ImageIcon loadIcon(String path, int w, int h)
	{
		java.net.URL url = this.getClass().getResource(path);
		return new ImageIcon(new ImageIcon(url).getImage().getScaledInstance(w,h,Image.SCALE_SMOOTH));
	}
	
	public void start()
	{
		if(mMainFrame != null)
			return;
		
		Settings.instance().load();
		this.loadImages();
		Text.load();
		this.createUI();
			
		mMainFrame.setVisible(true);
			
		AlarmCenter.instance().deleteObservers();
		AlarmCenter.instance().addObserver(new OnAlarmUpdate());
		
		CopterTelemetry.instance().deleteObservers();
		CopterTelemetry.instance().addObserver(new OnTelemetryUpdate());
		
		AlarmCenter.instance().setAlarm(Alarm.COPTER_NOT_FOUND);
			
		try
		{
			CopterCommander.instance().start(Settings.instance().getCopterIp(), Settings.instance().getCopterCmdPort());
			CopterTelemetry.instance().start(Settings.instance().getCopterIp(), Settings.instance().getCopterTelemetryPort());
		}
		catch(UnknownHostException e)
		{
			showErrorMsg(Text.get("INVALID_HOST"));
			e.printStackTrace();
		}
		catch(SocketException e)
		{
			showErrorMsg(Text.get("SOCKET_NOT_OPEN"));
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
		JOptionPane.showMessageDialog(mMainFrame, text, Text.get("ERROR"), JOptionPane.ERROR_MESSAGE);
	}
	
    private ImageIcon rotateImageIcon(ImageIcon picture, double angle)
    {
        int w = picture.getIconWidth();
        int h = picture.getIconHeight();
        int type = BufferedImage.TYPE_INT_RGB;  // other options, see api
        BufferedImage image = new BufferedImage(h, w, type);
        Graphics2D g2 = image.createGraphics();
        double x = (h - w)/2.0;
        double y = (w - h)/2.0;
        AffineTransform at = AffineTransform.getTranslateInstance(x, y);
        at.rotate(Math.toRadians(angle), w/2.0, h/2.0);
        g2.setColor(mMainFrame.getBackground());
        g2.fillRect(0, 0, w, h);
        g2.drawImage(picture.getImage(), at, null);
        g2.dispose();
        picture = new ImageIcon(image);
 
        return picture;
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
