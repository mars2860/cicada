package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;

import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Locale;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import commands.CmdSetMotorsGas;
import commands.CmdSwitchMotors;
import commands.CopterCommander;
import net.miginfocom.swing.MigLayout;

public class CopterCtrlPanel implements WindowListener
{
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
	
	private JFrame mMainFrame;
	private JSlider mgas0;
	private JSlider mgas1;
	private JSlider mgas2;
	private JSlider mgas3;
	private CmdSetMotorsGas mCmdSetGas = new CmdSetMotorsGas();
		
	public CopterCtrlPanel() {}
	
	private void createUI()
	{
		mMainFrame = new JFrame();
		
		mMainFrame.setSize(800,600);
		mMainFrame.setTitle(Text.get("APP_TITLE"));
		mMainFrame.setLocationRelativeTo(null);
		mMainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		mMainFrame.addWindowListener(this);
		
		mMainFrame.setLayout(new MigLayout());
		
		JButton btnEspLedToggle = new JButton(Text.get("MOTORS_ENABLED"));
		btnEspLedToggle.addActionListener(new OnBtnSwitchMotors());

		mMainFrame.add(this.createMotorsPanel());
		
		mMainFrame.add(btnEspLedToggle);
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
	
	public void go()
	{
		if(mMainFrame == null)
		{
			Locale.setDefault(Locale.US);
			Text.load();
			this.createUI();
			mMainFrame.setVisible(true);
		
			try
			{
				CopterCommander.instance().start();
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
	}
	
	public void stop()
	{
		CopterCommander.instance().stop();
		
		if(mMainFrame != null)
			mMainFrame.setVisible(false);
	}
	
	private void showErrorMsg(String text)
	{
		JOptionPane.showMessageDialog(mMainFrame, text, Text.get("ERROR"), JOptionPane.ERROR_MESSAGE);
	}

	public static void main(String[] args)
	{
		CopterCtrlPanel app = new CopterCtrlPanel();
		
		app.go();
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
