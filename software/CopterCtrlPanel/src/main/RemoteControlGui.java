package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JTextField;

import copter.CopterCommander;
import copter.commands.CmdSetYPR;
import helper.NumericDocument;
import main.Settings.WndState;
import net.miginfocom.swing.MigLayout;

public class RemoteControlGui extends JSavedFrame
{
	private static final long serialVersionUID = 8512049441078339808L;
	
	private JTextField mtfYaw;
	private JTextField mtfPitch;
	private JTextField mtfRoll;
	
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
				
				yaw = (float)Math.toRadians(yaw);
				pitch = (float)Math.toRadians(pitch);
				roll = (float)Math.toRadians(roll);
			
				CmdSetYPR cmd = new CmdSetYPR(yaw,pitch,roll);
				CopterCommander.instance().addCmd(cmd);
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

	public RemoteControlGui()
	{
		super(ResBox.text("REMOTE_CONTROL"),200,200);
		createUI();
	}
	
	private void createUI()
	{
		this.setLayout(new MigLayout("","[center,grow][center,grow][center,grow]"));

		mtfYaw = new JTextField();
		mtfYaw.setDocument(new NumericDocument(0,true));
		mtfYaw.setText("0");
		mtfPitch = new JTextField();
		mtfPitch.setDocument(new NumericDocument(0,true));
		mtfPitch.setText("0");
		mtfRoll = new JTextField();
		mtfRoll.setDocument(new NumericDocument(0,true));
		mtfRoll.setText("0");
		
		JButton btnSendYpr = new JButton(ResBox.text("SEND"));
		btnSendYpr.addActionListener(new OnBtnSendYpr());
		
		this.add(new JLabel(ResBox.text("YAW") + "(deg)"));
		this.add(new JLabel(ResBox.text("PITCH") + "(deg)"));
		this.add(new JLabel(ResBox.text("ROLL") + "(deg)"),"wrap");
		this.add(mtfYaw,"grow");
		this.add(mtfPitch,"grow");
		this.add(mtfRoll,"grow");
		this.add(btnSendYpr);
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
