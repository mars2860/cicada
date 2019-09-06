package main;

import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Locale;
import java.util.Observable;
import java.util.Observer;
import java.util.Properties;

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
				CopterCtrlPanel.this.mCopterIp = mtfCopterIp.getIpAddressString();
				CopterCtrlPanel.this.mCopterCmdPort = Integer.parseInt(mtfCopterCmdPort.getText());
				CopterCtrlPanel.this.mCopterTelemetryPort = Integer.parseInt(mtfCopterTelemetryPort.getText());
				CopterCtrlPanel.this.mCopterVideoPort = Integer.parseInt(mtfCopterVideoPort.getText());
				
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
				
			mtfCopterIp.setText(CopterCtrlPanel.this.mCopterIp);
			mtfCopterCmdPort.setText(Integer.toString(CopterCtrlPanel.this.mCopterCmdPort));
			mtfCopterTelemetryPort.setText(Integer.toString(CopterCtrlPanel.this.mCopterTelemetryPort));
			mtfCopterVideoPort.setText(Integer.toString(CopterCtrlPanel.this.mCopterVideoPort));
			
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
	
	public static final String COPTER_DEFAULT_IP = "192.168.1.33";
	public static final int COPTER_DEFAULT_CMD_PORT = 4210;
	public static final int COPTER_DEFAULT_TELEMETRY_PORT = 4211;
	public static final int COPTER_DEFAULT_VIDEO_PORT = 4212;
	public static final String SETTINGS_FILENAME = "settings.properties";
	
	private JFrame mMainFrame;
	
	private String mCopterIp;
	private int mCopterCmdPort;
	private int mCopterTelemetryPort;
	private int mCopterVideoPort;
	
	private JLabel mlbAlarmIcon;
	private JLabel mlbAlarmText;
	
	private JCheckBox mcbMotorsEnabled;
	private MotorGasSlider mgas0;
	private MotorGasSlider mgas1;
	private MotorGasSlider mgas2;
	private MotorGasSlider mgas3;
	private CmdSetMotorsGas mCmdSetGas = new CmdSetMotorsGas();
	
	private ImageIcon mIconOk;
	private ImageIcon mIconError;
		
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
		mMainFrame.add(this.createMotorsPanel());
	}
	
	private JPanel createSettingsPanel()
	{
		JPanel pnlSettings = new JPanel(new MigLayout("","[grow, center]","[grow, center]"));
		pnlSettings.setBorder(new TitledBorder(Text.get("SETTINGS")));

		JButton btnSettings = new JButton(Text.get("SETTINGS"));
		btnSettings.addActionListener(new OnBtnSettings());
		
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
	
	private void loadImages()
	{
		java.net.URL okUrl = this.getClass().getResource("images/ok.png");
		mIconOk = new ImageIcon(new ImageIcon(okUrl).getImage().getScaledInstance(32,32,Image.SCALE_SMOOTH));
		
		java.net.URL errUrl = this.getClass().getResource("images/error.png");
		mIconError = new ImageIcon(new ImageIcon(errUrl).getImage().getScaledInstance(32,32,Image.SCALE_SMOOTH));
	}
	
	public void start()
	{
		if(mMainFrame != null)
			return;
		
		this.loadSettings();
		this.loadImages();
		Text.load();
		this.createUI();
			
		mMainFrame.setVisible(true);
			
		AlarmCenter.instance().deleteObservers();
		AlarmCenter.instance().addObserver(new OnAlarmUpdate());
			
		try
		{
			CopterCommander.instance().start(mCopterIp, mCopterCmdPort);
			CopterTelemetry.instance().start(mCopterIp, mCopterTelemetryPort);
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
		
		this.saveSettings();
		
		if(mMainFrame != null)
		{
			mMainFrame.setVisible(false);
			mMainFrame = null;
		}
	}
	
	private void loadSettings()
	{
		mCopterIp = COPTER_DEFAULT_IP;
		mCopterCmdPort = COPTER_DEFAULT_CMD_PORT;
		mCopterTelemetryPort = COPTER_DEFAULT_TELEMETRY_PORT;
		mCopterVideoPort = COPTER_DEFAULT_VIDEO_PORT;
		
		Properties prop = new Properties();
		
		try
		{
			prop.load(new FileInputStream(SETTINGS_FILENAME));
			
			String lang = prop.getProperty("Language","en");
			Locale locale = new Locale(lang);
			Locale.setDefault(locale);
			
			mCopterIp = prop.getProperty("CopterIp", mCopterIp);
			
			String value = prop.getProperty("CopterCmdPort", Integer.toString(mCopterCmdPort));
			mCopterCmdPort = Integer.parseInt(value);
			
			value = prop.getProperty("CopterTelemetryPort", Integer.toString(mCopterTelemetryPort));
			mCopterTelemetryPort = Integer.parseInt(value);
			
			value = prop.getProperty("CopterVideoPort", Integer.toString(mCopterVideoPort));
			mCopterVideoPort = Integer.parseInt(value);
		}
		catch (FileNotFoundException e)
		{
			e.printStackTrace();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}	
	}
	
	private void saveSettings()
	{
		Properties prop = new Properties();
		
		prop.setProperty("Language", Locale.getDefault().getLanguage());
		prop.setProperty("CopterIp", mCopterIp);
		prop.setProperty("CopterCmdPort", Integer.toString(mCopterCmdPort));
		prop.setProperty("CopterTelemetryPort", Integer.toString(mCopterTelemetryPort));
		prop.setProperty("CopterVideoPort", Integer.toString(mCopterVideoPort));
		
		try
		{
			prop.store(new FileOutputStream(SETTINGS_FILENAME), null);
		}
		catch(FileNotFoundException e)
		{
			e.printStackTrace();
		}
		catch(IOException e)
		{
			e.printStackTrace();
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
