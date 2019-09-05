package main;

import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;

import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Locale;
import java.util.Observable;
import java.util.Observer;

import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.JToggleButton;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import commands.CmdSetMotorsGas;
import commands.CmdSwitchMotors;
import commands.CopterCommander;
import helper.NumericDocument;
import net.miginfocom.swing.MigLayout;

public class CopterCtrlPanel implements WindowListener
{
	private class OnBtnStartStop implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent evt)
		{
			JToggleButton btn = (JToggleButton)evt.getSource();
			
			if(btn.isSelected())
			{
				btn.setText(Text.get("STOP"));
				
				try
				{
					CopterCommander.instance().start(	mtfCopterIp.getIpAddressString(),
														Integer.parseInt(mtfCopterCmdPort.getText()));
				}
				catch(UnknownHostException e)
				{
					showErrorMsg(Text.get("INVALID_HOST"));
				}
				catch(SocketException e)
				{
					showErrorMsg(Text.get("SOCKET_NOT_OPEN"));
				}
			}
			else
			{
				btn.setText(Text.get("START"));
				
				CopterCommander.instance().stop();
			}
			
		}
	}
	
	private class OnBtnSwitchMotors implements ActionListener
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
				
				mgas0.setValue(value);
				mgas1.setValue(value);
				mgas2.setValue(value);
				mgas3.setValue(value);
	        }
		}	
	}
	
	private class OnAlarmUpdate implements Observer
	{
		@Override
		public void update(Observable o, Object arg)
		{
			mlbAlarmText.setText(AlarmCenter.instance().getAlarmText());
		}
	}
	
	public static final String COPTER_DEFAULT_IP = "192.168.1.33";
	public static final int COPTER_DEFAULT_CMD_PORT = 4210;
	public static final int COPTER_DEFAULT_TELEMETRY_PORT = 4211;
	public static final int COPTER_DEFAULT_VIDEO_PORT = 4212;
	
	private JFrame mMainFrame;
	
	private JIpTextField mtfCopterIp;
	private JTextField mtfCopterCmdPort;
	private JTextField mtfCopterTelemetryPort;
	private JTextField mtfCopterVideoPort;
	
	private JLabel mlbAlarmIcon;
	private JLabel mlbAlarmText;
	
	private JSlider mgas0;
	private JSlider mgas1;
	private JSlider mgas2;
	private JSlider mgas3;
	private CmdSetMotorsGas mCmdSetGas = new CmdSetMotorsGas();
	
	private ImageIcon mIconOk;
		
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
		
		JButton btnEspLedToggle = new JButton(Text.get("MOTORS_ENABLED"));
		btnEspLedToggle.addActionListener(new OnBtnSwitchMotors());

		mMainFrame.add(this.createAlarmPanel(),"growx");
		mMainFrame.add(this.createSettingsPanel(),"wrap");
		mMainFrame.add(this.createMotorsPanel());
		
		mMainFrame.add(btnEspLedToggle);
	}
	
	private JPanel createSettingsPanel()
	{
		JPanel pnlSettings = new JPanel(new MigLayout("","[40!][40!][40!][80!]"));
		pnlSettings.setBorder(new TitledBorder(Text.get("SETTINGS")));

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
			
		mtfCopterIp.setText(COPTER_DEFAULT_IP);
		mtfCopterCmdPort.setText(Integer.toString(COPTER_DEFAULT_CMD_PORT));
		mtfCopterTelemetryPort.setText(Integer.toString(COPTER_DEFAULT_TELEMETRY_PORT));
		mtfCopterVideoPort.setText(Integer.toString(COPTER_DEFAULT_VIDEO_PORT));
		
		JToggleButton btnStartStop = new JToggleButton(Text.get("START"));
		btnStartStop.addActionListener(new OnBtnStartStop());
			
		pnlSettings.add(new JLabel(Text.get("IP_ADDRESS")),"span,wrap");
		pnlSettings.add(mtfCopterIp, "spanx 3,growx");
		
		pnlSettings.add(btnStartStop,"spany 3, grow, wrap");
		
		pnlSettings.add(new JLabel(Text.get("CMD_PORT")));
		pnlSettings.add(new JLabel(Text.get("TELEMETRY_PORT")));
		pnlSettings.add(new JLabel(Text.get("VIDEO_PORT")),"wrap");
		
		pnlSettings.add(mtfCopterCmdPort, "growx");
		pnlSettings.add(mtfCopterTelemetryPort, "growx");
		pnlSettings.add(mtfCopterVideoPort, "growx");

		return pnlSettings;
	}
	
	private JPanel createAlarmPanel()
	{
		JPanel pnlAlarm = new JPanel(new MigLayout("","[][grow]","[]"));
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
		pnlMotorsGas.add(new JLabel("M1"));
		pnlMotorsGas.add(new JLabel("M2"));
		pnlMotorsGas.add(new JLabel("M3"));
		pnlMotorsGas.add(new JLabel("M4"));
		pnlMotorsGas.add(new JLabel("COM"),"wrap");
		
		mgas0 = new JSlider(JSlider.VERTICAL, 0, 255, 0);
		mgas1 = new JSlider(JSlider.VERTICAL, 0, 255, 0);
		mgas2 = new JSlider(JSlider.VERTICAL, 0, 255, 0);
		mgas3 = new JSlider(JSlider.VERTICAL, 0, 255, 0);
		JSlider mgasCom = new JSlider(JSlider.VERTICAL, 0, 255, 0);
		
		mgas0.setPaintLabels(true);
		
		mgas0.addChangeListener(new MotorGasChanged(0));
		mgas1.addChangeListener(new MotorGasChanged(1));
		mgas2.addChangeListener(new MotorGasChanged(2));
		mgas3.addChangeListener(new MotorGasChanged(3));
		mgasCom.addChangeListener(new SetAllMotorGas());
		
		pnlMotorsGas.add(mgas0);
		pnlMotorsGas.add(mgas1);
		pnlMotorsGas.add(mgas2);
		pnlMotorsGas.add(mgas3);
		pnlMotorsGas.add(mgasCom);
		
		return pnlMotorsGas;
	}
	
	private void loadImages()
	{
		java.net.URL url = this.getClass().getResource("images/ok.png");
		
		mIconOk = new ImageIcon(new ImageIcon(url).getImage().getScaledInstance(32,32,Image.SCALE_SMOOTH));
	}
	
	public void start()
	{
		if(mMainFrame == null)
		{
			Locale.setDefault(Locale.US);
			Text.load();
			this.loadImages();
			this.createUI();
			
			mMainFrame.setVisible(true);
			
			AlarmCenter.instance().deleteObservers();
			AlarmCenter.instance().addObserver(new OnAlarmUpdate());
		}
	}
	
	public void stop()
	{
		CopterCommander.instance().stop();
		
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
