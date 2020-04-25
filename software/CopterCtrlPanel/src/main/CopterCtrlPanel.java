package main;

import java.awt.Graphics2D;
import java.awt.Image;
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
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import copter.Alarm;
import copter.AlarmCenter;
import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.commands.CmdCalibrateAccel;
import copter.commands.CmdCalibrateGyro;
import copter.commands.CmdCalibrateMagnet;
import copter.commands.CmdEnableStabilization;
import copter.commands.CmdResetAltitude;
import copter.commands.CmdSetAltPid;
import copter.commands.CmdSetBaseGas;
import copter.commands.CmdSetMotorsGas;
import copter.commands.CmdSetPeriods;
import copter.commands.CmdSetPitchPid;
import copter.commands.CmdSetRollPid;
import copter.commands.CmdSetYawPid;
import copter.commands.CmdSwitchMotors;

import net.miginfocom.swing.MigLayout;

public class CopterCtrlPanel implements WindowListener
{
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
	
	public static class OnMotorsEnabled implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			boolean state = !CopterTelemetry.instance().getDroneState().motorsEnabled;
			CopterCommander.instance().addCmd(new CmdSwitchMotors(state));		
		}
	}
	
	private class OnStabilizationEnabled implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			boolean state = !CopterTelemetry.instance().getDroneState().stabilizationEnabled;
			CopterCommander.instance().addCmd(new CmdEnableStabilization(state));
		}
	}
	
	private class MotorGasChanged implements ChangeListener
	{
		private int mChl;
		
		public MotorGasChanged(int channel)
		{
			mChl = channel;
		}
		
		@Override
		public void stateChanged(ChangeEvent e)
		{
			JSlider slider = (JSlider)e.getSource();
	        
			if(!slider.getValueIsAdjusting())
			{
				int value = slider.getValue();
				mCmdSetGas.setGas(mChl,value);
				
				CopterCommander.instance().addCmd(mCmdSetGas);
	        }	
		}
	}
	
	private class SetAllMotorGas implements ChangeListener
	{
		@Override
		public void stateChanged(ChangeEvent e)
		{
			JSlider slider = (JSlider)e.getSource();
	        
			if(!slider.getValueIsAdjusting())
			{
				int value = slider.getValue();
				CopterCommander.instance().addCmd(new CmdSetBaseGas(value));
	        }
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
		@Override
		public void update(Observable o, Object arg)
		{
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
			mlbWifiLevel.setText(Integer.toString(droneState.wifiLevel));
			
			float yaw = (float)Math.toDegrees(droneState.yaw);
			float yawTarget = (float)Math.toDegrees(droneState.yawRatePid.target);
			float pitch = (float)Math.toDegrees(droneState.pitch);
			float pitchTarget = (float)Math.toDegrees(droneState.pitchPid.target);
			float roll = (float)Math.toDegrees(droneState.roll);
			float rollTarget = (float)Math.toDegrees(droneState.rollPid.target);
			
			mlbYawValue.setText(fmt1.format(yaw) + "/" + fmt1.format(yawTarget));
			mlbPitchValue.setText(fmt1.format(pitch) + "/" + fmt1.format(pitchTarget));
			mlbRollValue.setText(fmt1.format(roll) + "/" + fmt1.format(rollTarget));
			mlbHeading.setText(fmt2.format(droneState.heading));
			mlbLoopTime.setText(fmt2.format(droneState.mainLoopTime));
			mlbTemperature.setText(fmt1.format(droneState.temperature));
			mlbPressure.setText(fmt1.format(droneState.pressure) + "/" +
								fmt1.format(droneState.seaLevel));
			mlbAltitude.setText(fmt1.format(droneState.altitude) + "/" +
								fmt1.format(droneState.altPid.target));
			
			mlbYaw.setIcon(rotateImageIcon(mIconYaw, yaw));
			mlbPitch.setIcon(rotateImageIcon(mIconPitch, pitch));
			mlbRoll.setIcon(rotateImageIcon(mIconRoll, roll));

			mgas0.setGas(droneState.motorGas[0],false);
			mgas1.setGas(droneState.motorGas[1],false);
			mgas2.setGas(droneState.motorGas[2],false);
			mgas3.setGas(droneState.motorGas[3],false);
			mgasBase.setGas(droneState.baseGas, false);
			
			mcbMotorsEnabled.setSelected(droneState.motorsEnabled);
			mcbStabilizationEnabled.setSelected(droneState.stabilizationEnabled);
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

	private JFrame mMainFrame;
	
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
	
	private JCheckBox mcbMotorsEnabled;
	private JCheckBox mcbStabilizationEnabled;
	private MotorGasSlider mgas0;
	private MotorGasSlider mgas1;
	private MotorGasSlider mgas2;
	private MotorGasSlider mgas3;
	private MotorGasSlider mgasBase;
	private CmdSetMotorsGas mCmdSetGas = new CmdSetMotorsGas();
	
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
		
	public CopterCtrlPanel() {}
	
	private void createUI()
	{
		mMainFrame = new JFrame();
		
		mMainFrame.setSize(800,600);
		mMainFrame.setTitle(Text.get("APP_TITLE"));
		mMainFrame.setLocationRelativeTo(null);
		mMainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		mMainFrame.addWindowListener(this);
		
		mMainFrame.setLayout(new MigLayout("","[][grow][]","[][grow]"));

		mMainFrame.add(this.createAlarmPanel(),"spanx 2,grow");
		mMainFrame.add(this.createSettingsPanel(),"grow,wrap");
		mMainFrame.add(this.createStatusPanel(),"growy");
		mMainFrame.add(this.createMotorsPanel(),"growy");
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
		
		pnlSettings.add(btnConnect);
		pnlSettings.add(btnSensors);
		pnlSettings.add(btnSettings);
		pnlSettings.add(btnPID);

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
	
	private JPanel createMotorsPanel()
	{
		JPanel pnlMotorsGas = new JPanel(new MigLayout("","","[][][grow]"));
		pnlMotorsGas.setBorder(new TitledBorder(Text.get("MOTORS")));
		
		mcbMotorsEnabled = new JCheckBox(Text.get("MOTORS_ENABLED"));
		mcbMotorsEnabled.addActionListener(new OnMotorsEnabled());
		
		mcbStabilizationEnabled = new JCheckBox(Text.get("STABILIZATION_ENABLED"));
		mcbStabilizationEnabled.addActionListener(new OnStabilizationEnabled());

		mgas0 = new MotorGasSlider("M1");
		mgas1 = new MotorGasSlider("M2");
		mgas2 = new MotorGasSlider("M3");
		mgas3 = new MotorGasSlider("M4");
		mgasBase = new MotorGasSlider("BASE");
				
		mgas0.addChangeListener(new MotorGasChanged(0));
		mgas1.addChangeListener(new MotorGasChanged(1));
		mgas2.addChangeListener(new MotorGasChanged(2));
		mgas3.addChangeListener(new MotorGasChanged(3));
		mgasBase.addChangeListener(new SetAllMotorGas());
		
		MotorGasSlider yawSlider = new MotorGasSlider("Yaw");
		yawSlider.setGas((MotorGasSlider.MIN_GAS + MotorGasSlider.MAX_GAS)/2,false);
		yawSlider.addChangeListener(new ChangeListener()
		{
			int oldValue;
			
			@Override
			public void stateChanged(ChangeEvent e)
			{
				JSlider sl = (JSlider)e.getSource();
				if(!sl.getValueIsAdjusting())
				{
					int dx = sl.getValue() - oldValue;
					oldValue = sl.getValue();
					
					mgas0.setGas(mgas0.getGas() + dx,true);
					mgas2.setGas(mgas2.getGas() + dx,true);
					mgas1.setGas(mgas1.getGas() - dx,true);
					mgas3.setGas(mgas3.getGas() - dx,true);
				}
			}
		});
		
		pnlMotorsGas.add(mcbMotorsEnabled,"span,wrap");
		pnlMotorsGas.add(mcbStabilizationEnabled,"span,wrap");
		pnlMotorsGas.add(mgas0,"grow");
		pnlMotorsGas.add(mgas1,"grow");
		pnlMotorsGas.add(mgas2,"grow");
		pnlMotorsGas.add(mgas3,"grow");
		pnlMotorsGas.add(mgasBase,"grow");
		pnlMotorsGas.add(yawSlider,"grow");
		
		return pnlMotorsGas;
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
		java.net.URL okUrl = this.getClass().getResource("images/ok.png");
		mIconOk = new ImageIcon(new ImageIcon(okUrl).getImage().getScaledInstance(32,32,Image.SCALE_SMOOTH));
		
		java.net.URL errUrl = this.getClass().getResource("images/error.png");
		mIconError = new ImageIcon(new ImageIcon(errUrl).getImage().getScaledInstance(32,32,Image.SCALE_SMOOTH));
		
		int statusIconWidth = 24;
		int statusIconHeight = 24;
		
		java.net.URL batUrl = this.getClass().getResource("images/battery.png");
		mIconBat = new ImageIcon(new ImageIcon(batUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
		
		java.net.URL wifiUrl = this.getClass().getResource("images/wifi.png");
		mIconWifi = new ImageIcon(new ImageIcon(wifiUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
		
		java.net.URL yawUrl = this.getClass().getResource("images/yaw.png");
		mIconYaw = new ImageIcon(new ImageIcon(yawUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
		
		java.net.URL pitchUrl = this.getClass().getResource("images/pitch.png");
		mIconPitch = new ImageIcon(new ImageIcon(pitchUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
		
		java.net.URL rollUrl = this.getClass().getResource("images/roll.png");
		mIconRoll = new ImageIcon(new ImageIcon(rollUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
		
		java.net.URL headingUrl = this.getClass().getResource("images/heading.png");
		mIconHeading = new ImageIcon(new ImageIcon(headingUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
		
		java.net.URL looptimeUrl = this.getClass().getResource("images/cputime.png");
		mIconLoopTime = new ImageIcon(new ImageIcon(looptimeUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
		
		java.net.URL temperatureUrl = this.getClass().getResource("images/temperature.png");
		mIconTemperature = new ImageIcon(new ImageIcon(temperatureUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
		
		java.net.URL pressureUrl = this.getClass().getResource("images/pressure.png");
		mIconPressure = new ImageIcon(new ImageIcon(pressureUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
		
		java.net.URL altitudeUrl = this.getClass().getResource("images/altitude.png");
		mIconAltitude = new ImageIcon(new ImageIcon(altitudeUrl).getImage().getScaledInstance(statusIconWidth,statusIconHeight,Image.SCALE_SMOOTH));
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
