package main;

import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
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

import copter.AlarmCenter;
import copter.CopterCommander;
import copter.CopterTelemetry;
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
			JPanel pnlSettings = new JPanel(new MigLayout("","[40!][40!][40!]"));

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
			
			pnlSettings.add(new JLabel(Text.get("IP_ADDRESS")),"span,grow,wrap");
			pnlSettings.add(mtfCopterIp, "span,grow,wrap");
			
			pnlSettings.add(new JLabel(Text.get("CMD_PORT")));
			pnlSettings.add(new JLabel(Text.get("TELEMETRY_PORT")));
			pnlSettings.add(new JLabel(Text.get("VIDEO_PORT")),"wrap");
			
			pnlSettings.add(mtfCopterCmdPort, "grow");
			pnlSettings.add(mtfCopterTelemetryPort, "grow");
			pnlSettings.add(mtfCopterVideoPort, "grow, wrap");
			
			mcbLocale = new JComboBox<Locale>();
			
			mcbLocale.addItem(new Locale("en"));
			mcbLocale.addItem(new Locale("ru"));
			
			mcbLocale.setSelectedItem(Locale.getDefault());
			
			pnlSettings.add(new JLabel(Text.get("LANGUAGE")));
			pnlSettings.add(mcbLocale, "span,grow,wrap");
			
			JPanel pnlOkCancel = new JPanel(new MigLayout("insets 0 0 0 0","[80!]"));
			JButton btnOk = new JButton(Text.get("OK"));
			JButton btnCancel = new JButton(Text.get("CANCEL"));
			
			btnOk.addActionListener(new OnBtnOk());
			btnCancel.addActionListener(new OnBtnCancel());
			
			pnlOkCancel.add(btnOk,"grow,wrap");
			pnlOkCancel.add(btnCancel,"grow");
			
			pnlSettings.add(new JPanel(),"h 10!,span,wrap");
			pnlSettings.add(pnlOkCancel, "span, align right");

			this.setLayout(new MigLayout("","[grow]","[grow]"));
			this.setTitle(Text.get("SETTINGS"));
			this.setResizable(false);
			this.setSize(160, 240);
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
	
	private class OnMotorsEnabled implements ActionListener
	{
		private boolean state;
		
		@Override
		public void actionPerformed(ActionEvent e)
		{
			state = !state;
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
		}
	}

	private JFrame mMainFrame;
	
	private JLabel mlbAlarmIcon;
	private JLabel mlbAlarmText;
	private JLabel mlbBattery;
	private JLabel mlbWifiLevel;
	
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

		JButton btnSettings = new JButton(Text.get("SETTINGS"));
		btnSettings.addActionListener(new OnBtnSettings());
		
		JButton btnSensors = new JButton(Text.get("SENSORS"));
		btnSensors.addActionListener(new OnBtnSensors());
		
		pnlSettings.add(btnSettings);
		pnlSettings.add(btnSensors);

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
		mcbMotorsEnabled.addActionListener(new OnMotorsEnabled());

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
		
		pnlStatus.add(new JLabel(mIconBat));
		pnlStatus.add(mlbBattery,"w 80!");
		pnlStatus.add(new JLabel(mIconWifi));
		pnlStatus.add(mlbWifiLevel,"w 20!");
		
		return pnlStatus;
	}
	
	private void loadImages()
	{
		java.net.URL okUrl = this.getClass().getResource("images/ok.png");
		mIconOk = new ImageIcon(new ImageIcon(okUrl).getImage().getScaledInstance(32,32,Image.SCALE_SMOOTH));
		
		java.net.URL errUrl = this.getClass().getResource("images/error.png");
		mIconError = new ImageIcon(new ImageIcon(errUrl).getImage().getScaledInstance(32,32,Image.SCALE_SMOOTH));
		
		java.net.URL batUrl = this.getClass().getResource("images/battery.png");
		mIconBat = new ImageIcon(new ImageIcon(batUrl).getImage().getScaledInstance(16,16,Image.SCALE_SMOOTH));
		
		java.net.URL wifiUrl = this.getClass().getResource("images/wifi.png");
		mIconWifi = new ImageIcon(new ImageIcon(wifiUrl).getImage().getScaledInstance(16,16,Image.SCALE_SMOOTH));
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
