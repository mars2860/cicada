package main;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
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
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;
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
import copter.commands.CmdSetMotorsGas;
import copter.commands.CmdSwitchMotors;
import helper.NumericDocument;
import net.miginfocom.swing.MigLayout;

public class CopterCtrlPanel implements WindowListener
{
	private class SettingsDlg extends JDialog
	{
		private static final long serialVersionUID = 5506867286826277615L;
		
		private JIpTextField mtfCopterIp;
		private JTextField mtfCopterCmdPort;
		private JTextField mtfCopterTelemetryPort;
		private JTextField mtfCopterVideoPort;
		private JComboBox<Locale> mcbLocale;
		
		private class OnBtnOk implements ActionListener
		{
			@Override
			public void actionPerformed(ActionEvent e)
			{
				Settings.instance().setCopterIp(mtfCopterIp.getIpAddressString());
				Settings.instance().setCopterCmdPort(Integer.parseInt(mtfCopterCmdPort.getText()));
				Settings.instance().setCopterTelemetryPort(Integer.parseInt(mtfCopterTelemetryPort.getText()));
				Settings.instance().setmCopterVideoPort(Integer.parseInt(mtfCopterVideoPort.getText()));
				
				Locale locale = (Locale)mcbLocale.getSelectedItem();
				Locale.setDefault(locale);
				
				SettingsDlg.this.setVisible(false);
				
				CopterCtrlPanel.this.stop();
				CopterCtrlPanel.this.start();
			}
		}
		
		private class OnBtnCancel implements ActionListener
		{
			@Override
			public void actionPerformed(ActionEvent e)
			{
				SettingsDlg.this.setVisible(false);
			}
		}
		
		public SettingsDlg()
		{
			super(mMainFrame, true);
			this.createUI();
		}
		
		private void createUI()
		{
			DecimalFormat fmt = new DecimalFormat();
			fmt.setMaximumFractionDigits(2);
			
			JPanel pnlSettings = new JPanel(new MigLayout());

			mtfCopterIp = new JIpTextField();
			mtfCopterCmdPort = new JTextField();
			mtfCopterTelemetryPort = new JTextField();
			mtfCopterVideoPort = new JTextField();
				
			mtfCopterCmdPort.setDocument(new NumericDocument(0, false));
			mtfCopterTelemetryPort.setDocument(new NumericDocument(0, false));
			mtfCopterVideoPort.setDocument(new NumericDocument(0, false));
				
			mtfCopterCmdPort.setHorizontalAlignment(JTextField.RIGHT);
			mtfCopterTelemetryPort.setHorizontalAlignment(JTextField.RIGHT);
			mtfCopterVideoPort.setHorizontalAlignment(JTextField.RIGHT);
				
			mtfCopterIp.setText(Settings.instance().getCopterIp());
			mtfCopterCmdPort.setText(Integer.toString(Settings.instance().getCopterCmdPort()));
			mtfCopterTelemetryPort.setText(Integer.toString(Settings.instance().getCopterTelemetryPort()));
			mtfCopterVideoPort.setText(Integer.toString(Settings.instance().getCopterVideoPort()));
			
			mcbLocale = new JComboBox<Locale>();
			
			mcbLocale.addItem(new Locale("en"));
			mcbLocale.addItem(new Locale("ru"));
			
			mcbLocale.setSelectedItem(Locale.getDefault());
			
			JPanel pnlNet = new JPanel(new MigLayout("","[40!][40!][40!]"));
			pnlNet.setBorder(new TitledBorder(Text.get("NET")));
			
			pnlNet.add(new JLabel(Text.get("IP_ADDRESS")),"span,grow,wrap");
			pnlNet.add(mtfCopterIp, "span,grow,wrap");
			
			pnlNet.add(new JLabel(Text.get("CMD_PORT")));
			pnlNet.add(new JLabel(Text.get("TELEMETRY_PORT")));
			pnlNet.add(new JLabel(Text.get("VIDEO_PORT")),"wrap");
			
			pnlNet.add(mtfCopterCmdPort, "grow");
			pnlNet.add(mtfCopterTelemetryPort, "grow");
			pnlNet.add(mtfCopterVideoPort, "grow, wrap");
			
			JPanel pnlCalibration = new JPanel(new MigLayout());
			pnlCalibration.setBorder(new TitledBorder(Text.get("CALIBRATION")));

			pnlCalibration.add(new JLabel("AccelXOffset"));
			String text = ": " + Integer.toString(Settings.instance().getAccelXOffset());
			pnlCalibration.add(new JLabel(text));
			pnlCalibration.add(new JLabel("GyroXOffset"));
			text = ": " + Integer.toString(Settings.instance().getGyroXOffset());
			pnlCalibration.add(new JLabel(text),"wrap");
			
			pnlCalibration.add(new JLabel("AccelYOffset"));
			text = ": " + Integer.toString(Settings.instance().getAccelYOffset());
			pnlCalibration.add(new JLabel(text));
			pnlCalibration.add(new JLabel("GyroYOffset"));
			text = ": " + Integer.toString(Settings.instance().getGyroYOffset());
			pnlCalibration.add(new JLabel(text),"wrap");
			
			pnlCalibration.add(new JLabel("AccelZOffset"));
			text = ": " + Integer.toString(Settings.instance().getAccelZOffset());
			pnlCalibration.add(new JLabel(text));
			pnlCalibration.add(new JLabel("GyroZOffset"));
			text = ": " + Integer.toString(Settings.instance().getGyroZOffset());
			pnlCalibration.add(new JLabel(text),"wrap");
			
			pnlCalibration.add(new JLabel("MagnetXOffset"));
			text = ": " + Integer.toString(Settings.instance().getMagnetXOffset());
			pnlCalibration.add(new JLabel(text));
			pnlCalibration.add(new JLabel("MagnetXScale"));
			text = ": " + fmt.format(Settings.instance().getMagnetXScale());
			pnlCalibration.add(new JLabel(text),"wrap");
			
			pnlCalibration.add(new JLabel("MagnetYOffset"));
			text = ": " + Integer.toString(Settings.instance().getMagnetYOffset());
			pnlCalibration.add(new JLabel(text));
			pnlCalibration.add(new JLabel("MagnetYScale"));
			text = ": " + fmt.format(Settings.instance().getMagnetYScale());
			pnlCalibration.add(new JLabel(text),"wrap");
			
			pnlCalibration.add(new JLabel("MagnetZOffset"));
			text = ": " + Integer.toString(Settings.instance().getMagnetZOffset());
			pnlCalibration.add(new JLabel(text));
			pnlCalibration.add(new JLabel("MagnetZScale"));
			text = ": " + fmt.format(Settings.instance().getMagnetZScale());
			pnlCalibration.add(new JLabel(text),"wrap");

			JPanel pnlOkCancel = new JPanel(new MigLayout("insets 0 0 0 0","[80!]"));
			JButton btnOk = new JButton(Text.get("OK"));
			JButton btnCancel = new JButton(Text.get("CANCEL"));
			
			btnOk.addActionListener(new OnBtnOk());
			btnCancel.addActionListener(new OnBtnCancel());
			
			pnlOkCancel.add(btnOk,"grow,wrap");
			pnlOkCancel.add(btnCancel,"grow");
			
			pnlSettings.add(pnlCalibration,"span,grow,wrap");
			pnlSettings.add(pnlNet,"span,grow,wrap");
			pnlSettings.add(new JLabel(Text.get("LANGUAGE")));
			pnlSettings.add(mcbLocale, "span,w 80!,wrap");
			
			pnlSettings.add(new JPanel(),"h 10!,span,wrap");
			pnlSettings.add(pnlOkCancel, "span, align right");

			this.setLayout(new MigLayout("","[grow]","[grow]"));
			this.setTitle(Text.get("SETTINGS"));
			this.setResizable(true);
			this.setSize(310, 450);
			this.setLocationRelativeTo(null);
			this.add(pnlSettings, "grow");
		}
	}
	
	private class OnBtnSettings implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			SettingsDlg dlg = new SettingsDlg();
			dlg.setVisible(true);
		}
	}
	
	private class OnBtnSensors implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			SensorsDlg dlg = new SensorsDlg(mMainFrame);
			dlg.setVisible(true);
		}
	}
	
	private class OnMotorsEnabled implements ItemListener
	{
		@Override
		public void itemStateChanged(ItemEvent e)
		{
			boolean state = mcbMotorsEnabled.isSelected();
			CopterCommander.instance().addCmd(new CmdSwitchMotors(state));	
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
				mCmdSetGas.setGas(mChl, value);
				
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
				
				for(int i = 0; i < 4; i++)
					mCmdSetGas.setGas(i, value);
				
				CopterCommander.instance().addCmd(mCmdSetGas);
				
				mgas0.setGas(value);
				mgas1.setGas(value);
				mgas2.setGas(value);
				mgas3.setGas(value);
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
			
			String batState = fmt1.format(CopterTelemetry.instance().getBatteryVoltage()) + "V/" + 
								fmt1.format(CopterTelemetry.instance().getBatteryPercent()) + "%";
			
			mlbBattery.setText(batState);
			mlbWifiLevel.setText(Integer.toString(CopterTelemetry.instance().getWifiLevel()));
			
			float yaw = CopterTelemetry.instance().getYaw();
			float pitch = CopterTelemetry.instance().getPitch();
			float roll = CopterTelemetry.instance().getRoll();
			
			mlbYawValue.setText(fmt1.format(yaw));
			mlbPitchValue.setText(fmt1.format(pitch));
			mlbRollValue.setText(fmt1.format(roll));
			mlbHeading.setText(fmt1.format(CopterTelemetry.instance().getHeading()));
			
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
			
			CopterCommander.instance().addCmd(cmd1);
			CopterCommander.instance().addCmd(cmd1);
			CopterCommander.instance().addCmd(cmd1);
			CopterCommander.instance().addCmd(cmd2);
			CopterCommander.instance().addCmd(cmd2);
			CopterCommander.instance().addCmd(cmd2);
			CopterCommander.instance().addCmd(cmd3);
			CopterCommander.instance().addCmd(cmd3);
			CopterCommander.instance().addCmd(cmd3);
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
	
	private JCheckBox mcbMotorsEnabled;
	private MotorGasSlider mgas0;
	private MotorGasSlider mgas1;
	private MotorGasSlider mgas2;
	private MotorGasSlider mgas3;
	private CmdSetMotorsGas mCmdSetGas = new CmdSetMotorsGas();
	
	private ImageIcon mIconOk;
	private ImageIcon mIconError;
	private ImageIcon mIconBat;
	private ImageIcon mIconWifi;
	private ImageIcon mIconYaw;
	private ImageIcon mIconPitch;
	private ImageIcon mIconRoll;
	private ImageIcon mIconHeading;
		
	public CopterCtrlPanel() {}
	
	private void createUI()
	{
		mMainFrame = new JFrame();
		
		mMainFrame.setSize(800,600);
		mMainFrame.setTitle(Text.get("APP_TITLE"));
		mMainFrame.setLocationRelativeTo(null);
		mMainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		mMainFrame.addWindowListener(this);
		
		mMainFrame.setLayout(new MigLayout("","[grow][]"));

		mMainFrame.add(this.createAlarmPanel(),"grow");
		mMainFrame.add(this.createSettingsPanel(),"grow,wrap");
		mMainFrame.add(this.createStatusPanel(),"span,grow,wrap");
		mMainFrame.add(this.createMotorsPanel());
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
		
		pnlSettings.add(btnConnect);
		pnlSettings.add(btnSensors);
		pnlSettings.add(btnSettings);

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
		JPanel pnlMotorsGas = new JPanel(new MigLayout());
		pnlMotorsGas.setBorder(new TitledBorder(Text.get("MOTORS")));
		
		mcbMotorsEnabled = new JCheckBox(Text.get("MOTORS_ENABLED"));
		mcbMotorsEnabled.addItemListener(new OnMotorsEnabled());

		mgas0 = new MotorGasSlider("M1");
		mgas1 = new MotorGasSlider("M2");
		mgas2 = new MotorGasSlider("M3");
		mgas3 = new MotorGasSlider("M4");
		MotorGasSlider mgasCom = new MotorGasSlider("COM");
				
		mgas0.addChangeListener(new MotorGasChanged(0));
		mgas1.addChangeListener(new MotorGasChanged(1));
		mgas2.addChangeListener(new MotorGasChanged(2));
		mgas3.addChangeListener(new MotorGasChanged(3));
		mgasCom.addChangeListener(new SetAllMotorGas());
		
		pnlMotorsGas.add(mcbMotorsEnabled,"span,wrap");
		pnlMotorsGas.add(mgas0);
		pnlMotorsGas.add(mgas1);
		pnlMotorsGas.add(mgas2);
		pnlMotorsGas.add(mgas3);
		pnlMotorsGas.add(mgasCom);
		
		return pnlMotorsGas;
	}
	
	private JPanel createStatusPanel()
	{
		JPanel pnlStatus = new JPanel(new MigLayout("","","[center]"));
		pnlStatus.setBorder(new TitledBorder(Text.get("STATUS")));

		mlbBattery = new JLabel();
		mlbBattery.setHorizontalAlignment(JLabel.CENTER);
		
		mlbWifiLevel = new JLabel();
		mlbWifiLevel.setHorizontalAlignment(JLabel.CENTER);
		
		mlbYawValue = new JLabel();
		mlbYawValue.setHorizontalAlignment(JLabel.CENTER);
		
		mlbPitchValue = new JLabel();
		mlbPitchValue.setHorizontalAlignment(JLabel.CENTER);
		
		mlbRollValue = new JLabel();
		mlbRollValue.setHorizontalAlignment(JLabel.CENTER);
		
		mlbHeading = new JLabel();
		mlbHeading.setHorizontalAlignment(JLabel.CENTER);
		
		mlbYaw = new JLabel(mIconYaw);
		mlbPitch = new JLabel(mIconPitch);
		mlbRoll = new JLabel(mIconRoll);
		
		pnlStatus.add(new JLabel(mIconBat));
		pnlStatus.add(mlbBattery,"w 80!");
		pnlStatus.add(new JLabel(mIconWifi));
		pnlStatus.add(mlbWifiLevel,"w 30!");
		pnlStatus.add(mlbYaw);
		pnlStatus.add(mlbYawValue,"w 60!");
		pnlStatus.add(mlbPitch);
		pnlStatus.add(mlbPitchValue,"w 60!");
		pnlStatus.add(mlbRoll);
		pnlStatus.add(mlbRollValue,"w 60!");
		pnlStatus.add(new JLabel(mIconHeading));
		pnlStatus.add(mlbHeading,"w 30!");
		
		return pnlStatus;
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
        // FOR YOU ...
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
