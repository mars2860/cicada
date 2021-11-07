package main;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;

import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.DroneState;
import copter.commands.CmdSetAltitude;
import copter.commands.CmdSetBaseGas;
import copter.commands.CmdSetVelocityZ;
import copter.commands.CmdSetYPR;
import copter.commands.TakeOff;
import helper.NumericDocument;
import main.Settings.WndState;
import net.miginfocom.swing.MigLayout;

public class RemoteControlGui extends JSavedFrame
{
	private static final long serialVersionUID = 8512049441078339808L;

	private static final long KEYBOARD_PERIOD = 40; 	// FPS 25
	
	private static final float LEVEL_CTRL_DELTA = 7.f;
	private static final int MOTOR_CTRL_DELTA = 4;//2;
	private static final float ROTATE_CTRL_DELTA = 100.f;//80.f;
	private static final float ALT_CTRL_DELTA = 0.01f;
	private static final float VELOCITY_Z_CTRL_DELTA = 0.3f;
	
	private JTextField mtfYaw;
	private JTextField mtfPitch;
	private JTextField mtfRoll;
	private JTextField mtfAlt;
	private JButton btnGetUp;
	private JButton btnGetDown;
	private JButton btnLeft;
	private JButton btnRight;
	private JButton btnFwd;
	private JButton btnBck;
	private JButton btnCw;
	private JButton btnCcw;
	private JButton btnStop;
	
	private Timer tmKeyboard;
	private KeyDispatcher kd;
	
	private class OnBtnSendYpr implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			String msg = ResBox.text("YAW");
			try
			{
				msg = ResBox.text("YAW");
				float yaw = Float.parseFloat(mtfYaw.getText());
				msg = ResBox.text("PITCH");
				float pitch = Float.parseFloat(mtfPitch.getText());
				msg = ResBox.text("ROLL");
				float roll = Float.parseFloat(mtfRoll.getText());
				msg = ResBox.text("ALTITUDE");
				float alt = Float.parseFloat(mtfAlt.getText());
				
				yaw = (float)Math.toRadians(yaw);
				pitch = (float)Math.toRadians(pitch);
				roll = (float)Math.toRadians(roll);
			
				CmdSetYPR cmd = new CmdSetYPR(yaw,pitch,roll);
				CmdSetAltitude cmd2 = new CmdSetAltitude(alt);
				CopterCommander.instance().addCmd(cmd);
				CopterCommander.instance().addCmd(cmd2);
			}
			catch(NumberFormatException ex)
			{
				String text = ResBox.text("INVALID_VALUE") + " " + msg; 
				JOptionPane.showMessageDialog(
						RemoteControlGui.this,
						text,
						ResBox.text("ERROR"),
						JOptionPane.ERROR_MESSAGE);
			}
		}
	}
		
	private class KeyboardProcess extends TimerTask
	{
		private boolean yprTrigger = false;
		
		@Override
		public void run()
		{
			DroneState ds = CopterTelemetry.instance().getDroneState();
			
			CmdSetYPR cmdYpr = new CmdSetYPR(0,0,0);
			CmdSetVelocityZ cmdVz = null;
						
			// get up
			if(btnGetUp.getModel().isPressed())
			{
				if(ds.altPid.enabled == false)
				{
					CmdSetBaseGas cmd = new CmdSetBaseGas((int)ds.baseGas + MOTOR_CTRL_DELTA);
					CopterCommander.instance().addCmd(cmd);
				}
				else if(ds.velocityZPid.enabled)
				{
					// create instance to inform that we have hold a key
					cmdVz = new CmdSetVelocityZ(VELOCITY_Z_CTRL_DELTA);
					if(ds.velocityZPid.target <= 0.f)
						CopterCommander.instance().addCmd(cmdVz);
				}
				else
				{
					CmdSetAltitude cmd = new CmdSetAltitude((float)(ds.altPid.target + ALT_CTRL_DELTA));
					CopterCommander.instance().addCmd(cmd);
				}
			}

			// get down
			if(btnGetDown.getModel().isPressed())
			{
				if(ds.altPid.enabled == false)
				{
					CmdSetBaseGas cmd = new CmdSetBaseGas((int)ds.baseGas - MOTOR_CTRL_DELTA);
					CopterCommander.instance().addCmd(cmd);
				}
				else if(ds.velocityZPid.enabled)
				{
					cmdVz = new CmdSetVelocityZ(-VELOCITY_Z_CTRL_DELTA);
					if(ds.velocityZPid.target >= 0.f)
						CopterCommander.instance().addCmd(cmdVz);
				}
				else
				{
					CmdSetAltitude cmd = new CmdSetAltitude((float)(ds.altPid.target - ALT_CTRL_DELTA));
					CopterCommander.instance().addCmd(cmd);
				}
			}
			
			// roll left
			if(btnLeft.getModel().isPressed())
			{
				cmdYpr.setRollDeg(-LEVEL_CTRL_DELTA);
			}
			// roll right
			if(btnRight.getModel().isPressed())
			{
				cmdYpr.setRollDeg(LEVEL_CTRL_DELTA);
			}
			// pitch fwd
			if(btnFwd.getModel().isPressed())
			{
				cmdYpr.setPitchDeg(LEVEL_CTRL_DELTA);
			}
			// pitch bck
			if(btnBck.getModel().isPressed())
			{
				cmdYpr.setPitchDeg(-LEVEL_CTRL_DELTA);
			}
			// rotate cw
			if(btnCw.getModel().isPressed())
			{
				cmdYpr.setYawRateDeg(-ROTATE_CTRL_DELTA);
			}
			// rotate ccw
			if(btnCcw.getModel().isPressed())
			{
				cmdYpr.setYawRateDeg(ROTATE_CTRL_DELTA);
			}
			
			if(cmdYpr.isNull() == false)
			{
				if(	cmdYpr.getPitchRad() != ds.pitchPid.target ||
					cmdYpr.getRollRad() != ds.rollPid.target ||
					cmdYpr.getYawRad() != ds.yawRatePid.target )
				{
					CopterCommander.instance().addCmd(cmdYpr);
				}
				yprTrigger = true;
			}
			else if(yprTrigger) // to stop moving after we release a key
			{
				yprTrigger = false;
				cmdYpr = new CmdSetYPR(0,0,0);
				CopterCommander.instance().addCmd(cmdYpr);
				CopterCommander.instance().addCmd(cmdYpr);
				CopterCommander.instance().addCmd(cmdYpr);
			}
			
			if(cmdVz == null && ds.velocityZPid.enabled && ds.velocityZPid.target != 0.f)
			{
				cmdVz = new CmdSetVelocityZ(0);
				CopterCommander.instance().addCmd(cmdVz);
			}
			
			// stop
			if(btnStop.getModel().isPressed())
			{
				cmdYpr = new CmdSetYPR(0,0,0);
				CopterCommander.instance().addCmd(cmdYpr);
				CmdSetVelocityZ cmd = new CmdSetVelocityZ(0);
				CopterCommander.instance().addCmd(cmd);
			}
		}
	}
	
	private class KeyDispatcher implements KeyEventDispatcher
	{
		@Override
		public boolean dispatchKeyEvent(KeyEvent event)
		{
			if(event.getID() == KeyEvent.KEY_PRESSED)
			{
				if(event.getKeyCode() == KeyEvent.VK_Q)
				{
					btnCcw.requestFocus();
					btnCcw.getModel().setPressed(true);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_W)
				{
					btnCcw.requestFocus();
					btnFwd.getModel().setPressed(true);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_E)
				{
					btnCcw.requestFocus();
					btnCw.getModel().setPressed(true);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_A)
				{
					btnCcw.requestFocus();
					btnLeft.getModel().setPressed(true);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_S)
				{
					btnCcw.requestFocus();
					btnBck.getModel().setPressed(true);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_D)
				{
					btnCcw.requestFocus();
					btnRight.getModel().setPressed(true);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_Z)
				{
					btnCcw.requestFocus();
					btnGetUp.getModel().setPressed(true);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_X)
				{
					btnCcw.requestFocus();
					btnGetDown.getModel().setPressed(true);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_C)
				{
					btnCcw.requestFocus();
					btnStop.getModel().setPressed(true);
				}
			}
			
			if(event.getID() == KeyEvent.KEY_RELEASED)
			{
				if(event.getKeyCode() == KeyEvent.VK_Q)
				{
					btnCcw.getModel().setPressed(false);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_W)
				{
					btnFwd.getModel().setPressed(false);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_E)
				{
					btnCw.getModel().setPressed(false);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_A)
				{
					btnLeft.getModel().setPressed(false);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_S)
				{
					btnBck.getModel().setPressed(false);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_D)
				{
					btnRight.getModel().setPressed(false);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_Z)
				{
					btnGetUp.getModel().setPressed(false);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_X)
				{
					btnGetDown.getModel().setPressed(false);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_C)
				{
					btnStop.getModel().setPressed(false);
				}
				
				if(event.getKeyCode() == KeyEvent.VK_SHIFT)
				{
					TakeOff cmd = new TakeOff();
					cmd.run();
				}
			}
			
			return false;
		}
	}

	
	private class OnWndListener implements WindowListener
	{

		@Override
		public void windowOpened(WindowEvent e)
		{
			tmKeyboard = new Timer();
			tmKeyboard.schedule(new KeyboardProcess(), 0, KEYBOARD_PERIOD);
			kd = new KeyDispatcher();
			KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(kd);
		}

		@Override
		public void windowClosing(WindowEvent e) {}

		@Override
		public void windowClosed(WindowEvent e)
		{
			KeyboardFocusManager.getCurrentKeyboardFocusManager().removeKeyEventDispatcher(kd);
			tmKeyboard.cancel();
			tmKeyboard = null;
			kd = null;
		}

		@Override
		public void windowIconified(WindowEvent e) {}

		@Override
		public void windowDeiconified(WindowEvent e) {}

		@Override
		public void windowActivated(WindowEvent e) {}

		@Override
		public void windowDeactivated(WindowEvent e) {}
	}

	public RemoteControlGui()
	{
		super(ResBox.text("REMOTE_CONTROL"),200,400);
		createUI();
	}
	
	private void createUI()
	{
		this.setLayout(new MigLayout("","[center,grow][center,grow][center,grow][center,grow]"));
		this.setIconImage(ResBox.icon("REMOTE_CONTROL").getImage());

		mtfYaw = new JTextField();
		mtfYaw.setDocument(new NumericDocument(2,true));
		mtfYaw.setText("0");
		mtfPitch = new JTextField();
		mtfPitch.setDocument(new NumericDocument(2,true));
		mtfPitch.setText("0");
		mtfRoll = new JTextField();
		mtfRoll.setDocument(new NumericDocument(2,true));
		mtfRoll.setText("0");
		mtfAlt = new JTextField();
		mtfAlt.setDocument(new NumericDocument(3,true));
		mtfAlt.setText("0");
		
		JButton btnSendYpr = new JButton(ResBox.text("SEND"));
		btnSendYpr.addActionListener(new OnBtnSendYpr());
		
		this.add(new JLabel(ResBox.text("YAW") + "(deg)"));
		this.add(new JLabel(ResBox.text("PITCH") + "(deg)"));
		this.add(new JLabel(ResBox.text("ROLL") + "(deg)"));
		this.add(new JLabel(ResBox.text("ALTITUDE") + "(m)"),"wrap");
		this.add(mtfYaw,"grow");
		this.add(mtfPitch,"grow");
		this.add(mtfRoll,"grow");
		this.add(mtfAlt,"grow");
		this.add(btnSendYpr,"wrap");
		
		btnGetUp = new JButton(ResBox.icon("ARROW_GET_UP"));
		btnGetDown = new JButton(ResBox.icon("ARROW_GET_DOWN"));
		btnLeft = new JButton(ResBox.icon("ARROW_LEFT"));
		btnRight = new JButton(ResBox.icon("ARROW_RIGHT"));
		btnFwd = new JButton(ResBox.icon("ARROW_UP"));
		btnBck = new JButton(ResBox.icon("ARROW_DOWN"));
		btnCw = new JButton(ResBox.icon("ROTATE_CW"));
		btnCcw = new JButton(ResBox.icon("ROTATE_CCW"));
		btnStop = new JButton(ResBox.icon("STOP"));
		
		btnStop.requestFocus();
		
		JPanel pnlCtrl = new JPanel(new MigLayout());
		
		pnlCtrl.add(btnCcw);
		pnlCtrl.add(btnFwd);
		pnlCtrl.add(btnCw,"wrap");
		pnlCtrl.add(btnLeft);
		pnlCtrl.add(btnStop);
		pnlCtrl.add(btnRight,"wrap");
		pnlCtrl.add(btnGetUp);
		pnlCtrl.add(btnBck);
		pnlCtrl.add(btnGetDown);
		
		this.add(pnlCtrl,"span 4,grow");

		this.addWindowListener(new OnWndListener());
	}
	
	@Override
	protected WndState loadWndState()
	{
		return Settings.instance().getRcWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		Settings.instance().setRcWnd(ws);
	}
}
