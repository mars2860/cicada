package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.JToggleButton;

import helper.NumericDocument;
import main.AppSettings.WndState;
import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;
import net.miginfocom.swing.MigLayout;
import pdl.Accelerator;
import pdl.DroneCommander;
import pdl.DroneTelemetry;
import pdl.DroneState;
import pdl.commands.CmdEnableStabilization;
import pdl.commands.CmdSetAltitude;
import pdl.commands.CmdSetLoad;
import pdl.commands.CmdSwitchMotors;

public class RemoteControlGui extends JSavedFrame
{
	private static final long serialVersionUID = 8512049441078339808L;

	public static final long KEYBOARD_PERIOD = 40; 	// FPS 25
	
	private static final double STICK_DEAD_VALUE = 0.001;

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
	private JButton btnLoad1;
	private JButton btnLoad2;
	//private JButton btnLoad3;
	private JButton btnAntiTurtle;
	private JButton btnCamAng0;
	private JButton btnCamAng45;
	private JButton btnCamAng90;
	private JButton btnPhoto;
	private JToggleButton btnVideo;
	
	private JLabel lblGamepad;
	
	private Timer tmKeyboard;
	
	private RcSettingsGui rcSettingsGui;
	
	private class OnBtnSettings implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			rcSettingsGui.setVisible(true);
		}
	}
	
	private class OnBtnSend implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			String msg = "";
			try
			{
				msg = ResBox.text("ALTITUDE");
				float alt = Float.parseFloat(mtfAlt.getText());
			
				CmdSetAltitude cmd = new CmdSetAltitude(alt);
				DroneCommander.instance().addCmd(cmd);
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
		
	private class OnInputsPoll extends TimerTask
	{
		private long oldMillis = 0;
		
		private Accelerator mLiftAccelerator;
		private Accelerator mPitchAccelerator;
		private Accelerator mRollAccelerator;
		private Accelerator mRotateAccelerator;
		private boolean requestTrickMode;
		
		double liftCtrl;
		double rollCtrl;
		double pitchCtrl;
		double rotateCtrl;
		
		// this flag is true if there is input from stick
		boolean stickLiftActive;
		boolean stickRotateActive;
		boolean stickPitchActive;
		boolean stickRollActive;
		
		boolean oldBtnPhotoState;
		boolean oldBtnVideoState;

		@Override
		public void run()
		{
			DroneState.RemoteCtrl rc = DroneState.rc;

			/// calculate dt to apply accelerators
			
			double dt = 0;
		
			if(oldMillis > 0)
			{
				dt = (System.currentTimeMillis() - oldMillis);
				dt /= 1000.0;
			}
			else
			{
				dt = KEYBOARD_PERIOD;
				dt /= 1000.0;
			}
			
			oldMillis = System.currentTimeMillis();
			
			/// grab accelerator settings
			
			if(	mLiftAccelerator == null || 
			    Math.abs(mLiftAccelerator.getAccelTime() - rc.liftAccelTime) > 0.01)
			{
				mLiftAccelerator = new Accelerator(-1.0,0,1.0,rc.liftAccelTime);
			}
			
			if(	mPitchAccelerator == null || 
				Math.abs(mPitchAccelerator.getAccelTime() - rc.moveAccelTime) > 0.01)
			{
				mPitchAccelerator = new Accelerator(-1.0,0,1.0,rc.moveAccelTime);
			}
			
			if(	mRollAccelerator == null || 
				Math.abs(mRollAccelerator.getAccelTime() - rc.moveAccelTime) > 0.01)
			{
				mRollAccelerator = new Accelerator(-1.0,0,1.0,rc.moveAccelTime);
			}
			
			if(	mRotateAccelerator == null || 
				Math.abs(mRotateAccelerator.getAccelTime() - rc.rotateAccelTime) > 0.01)
			{
				mRotateAccelerator = new Accelerator(-1.0,0,1.0,rc.rotateAccelTime);
			}
			
			/// input processing
			
			Event event = new Event();

			Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();

			for(int i = 0; i < controllers.length; i++)
			{
				Controller controller = controllers[i];
				
				if(	(controller.getType() == Controller.Type.KEYBOARD) ||
					( controller.getType() == Controller.Type.GAMEPAD &&
					  controller.getName().compareToIgnoreCase(AppSettings.instance().getInputMap().gamepad) == 0)
				  )
				{
					controller.poll();
					
				    EventQueue queue = controller.getEventQueue();

				    while(queue.getNextEvent(event))
				    {
				        Component comp = event.getComponent();

				        for(int j = 0; j < AppSettings.InputMap.INPUT_CONTROL_COUNT; j++)
			        	{
				        	AppSettings.InputCtrl ic = AppSettings.instance().getInputMap().controls[j];
				        	
				        	// update sticks
				        	
				        	if(	comp.isAnalog() &&
				        	    ic.stick.compareToIgnoreCase(comp.getIdentifier().getName()) == 0
				        	  )
				        	{
				        		switch(j)
				        		{
				        		case AppSettings.InputMap.UP:
				        		case AppSettings.InputMap.DOWN:
				        			liftCtrl = -comp.getPollData();
				        			stickLiftActive = (Math.abs(liftCtrl) <= STICK_DEAD_VALUE)?false:true;
				        			break;
				        		case AppSettings.InputMap.LEFT:
				        		case AppSettings.InputMap.RIGHT:
				        			rollCtrl = comp.getPollData();
				        			stickRollActive = (Math.abs(rollCtrl) <= STICK_DEAD_VALUE)?false:true;
				        			break;
				        		case AppSettings.InputMap.FWD:
				        		case AppSettings.InputMap.BACK:
				        			pitchCtrl = -comp.getPollData();
				        			stickPitchActive = (Math.abs(pitchCtrl) <= STICK_DEAD_VALUE)?false:true;
				        			break;
				        		case AppSettings.InputMap.TURN_CW:
				        		case AppSettings.InputMap.TURN_CCW:
			        				rotateCtrl = comp.getPollData();
			        				stickRotateActive = (Math.abs(rotateCtrl) <= STICK_DEAD_VALUE)?false:true;
				        			break;
				        		}
				        	}
				        	
				        	// update buttons on gamepad or keyboard
				        	
				        	String compId = comp.getIdentifier().getName();
				        	
				        	if(controller.getType() == Controller.Type.GAMEPAD)
				        	{
				        		if(comp.getIdentifier() == Component.Identifier.Axis.POV)
								{
									float povPos = comp.getPollData();
									
									if(Float.compare(povPos,Component.POV.UP) == 0)
									{
										compId = "G_UP";
									}
									else if(Float.compare(povPos,Component.POV.DOWN) == 0)
									{
										compId = "G_DOWN";
									}
									else if(Float.compare(povPos,Component.POV.LEFT) == 0)
									{
										compId = "G_LEFT";
									}
									else if(Float.compare(povPos,Component.POV.RIGHT) == 0)
									{
										compId = "G_RIGHT";
									}
									else if(Float.compare(povPos,Component.POV.OFF) == 0)
									{
										btnGetUp.getModel().setPressed(false);
										btnGetDown.getModel().setPressed(false);
										btnLeft.getModel().setPressed(false);
										btnRight.getModel().setPressed(false);
										btnFwd.getModel().setPressed(false);
										btnBck.getModel().setPressed(false);
										btnCw.getModel().setPressed(false);
										btnCcw.getModel().setPressed(false);
									}
								}
								else
								{
									compId = "G_" + comp.getIdentifier().getName();
								}
				        	}
				        	
				        	if(	comp.isAnalog() == false &&
				        		(ic.btn1.compareToIgnoreCase(compId) == 0 ||
				        		 ic.btn2.compareToIgnoreCase(compId) == 0
				        		)
				        	  )
				        	{
				        		boolean btnState = (Float.compare(comp.getPollData(),0) == 0)?false:true; 
				        		switch(j)
				        		{
				        		case AppSettings.InputMap.UP:
				        			btnGetUp.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.DOWN:
				        			btnGetDown.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.LEFT:
				        			btnLeft.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.RIGHT:
				        			btnRight.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.FWD:
				        			btnFwd.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.BACK:
				        			btnBck.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.TURN_CW:
				        			btnCw.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.TURN_CCW:
				        			btnCcw.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.ARM_DISARM:
				        			if( Float.compare(comp.getPollData(),0) == 0 &&	// arm/disarm button released
				        			    DroneTelemetry.instance().isDroneConnected() ) 
				        			{
				        				DroneState ds = DroneTelemetry.instance().getDroneState();
				        				boolean enabled = !ds.motorsEnabled;
				        				CmdSwitchMotors cmd = new CmdSwitchMotors(enabled);
				        				DroneCommander.instance().addCmd(cmd);
				        				if(enabled == false)	// also switch off stabilization if we turn off motors
				        				{
				        					CmdEnableStabilization cmd2 = new CmdEnableStabilization(false);
				        					DroneCommander.instance().addCmd(cmd2);
				        				}
				        			}
				        			break;
				        		case AppSettings.InputMap.TRICK_MODE:
				        			if(Float.compare(comp.getPollData(),0) == 0)
				        			{
				        				requestTrickMode = false;
				        			}
				        			else
				        			{
				        				requestTrickMode = true;
				        			}
				        			break;
				        		case AppSettings.InputMap.LOAD1:
				        			btnLoad1.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.LOAD2:
				        			btnLoad2.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.ANTITURTLE:
				        			btnAntiTurtle.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.CAMERA_ANGLE_0:
				        			btnCamAng0.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.CAMERA_ANGLE_45:
				        			btnCamAng45.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.CAMERA_ANGLE_90:
				        			btnCamAng90.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.PHOTO:
				        			btnPhoto.getModel().setPressed(btnState);
				        			break;
				        		case AppSettings.InputMap.VIDEO:
				        			if(btnState)
				        			{
				        				btnVideo.setSelected(!btnVideo.isSelected());
				        			}
				        			break;
				        		}
				        	}
			        	}
				    }
				}
			}
			
			if(DroneTelemetry.instance().isDroneConnected() == false)
				return; // can't execute code below if drone is disconnected
			
			// get up
			if(btnGetUp.getModel().isPressed())
			{
				liftCtrl = mLiftAccelerator.accelerate(dt);
			}

			// get down
			if(btnGetDown.getModel().isPressed())
			{
				liftCtrl = mLiftAccelerator.decelerate(dt);
			}
						
			// roll left
			if(btnLeft.getModel().isPressed())
			{
				rollCtrl = mRollAccelerator.decelerate(dt);
			}
			
			// roll right
			if(btnRight.getModel().isPressed())
			{
				rollCtrl = mRollAccelerator.accelerate(dt);
			}
			
			// pitch fwd
			if(btnFwd.getModel().isPressed())
			{
				pitchCtrl = mPitchAccelerator.accelerate(dt);
			}
			
			// pitch bck
			if(btnBck.getModel().isPressed())
			{
				pitchCtrl = mPitchAccelerator.decelerate(dt);
			}
			
			// rotate cw
			if(btnCw.getModel().isPressed())
			{
				rotateCtrl = mRotateAccelerator.accelerate(dt);
			}
			
			// rotate ccw
			if(btnCcw.getModel().isPressed())
			{
				rotateCtrl = mRotateAccelerator.decelerate(dt);
			}
			
			// stop
			if(btnStop.getModel().isPressed())
			{
				liftCtrl = 0;
				rollCtrl = 0;
				pitchCtrl = 0;
				rotateCtrl = 0;
				mLiftAccelerator.reset();
				mPitchAccelerator.reset();
				mRollAccelerator.reset();
				mRotateAccelerator.reset();
			}
			else
			{
				if(	btnGetUp.getModel().isPressed() == false &&
					btnGetDown.getModel().isPressed() == false)
				{
					mLiftAccelerator.normalize(dt);
					
					if(stickLiftActive == false)
						liftCtrl = mLiftAccelerator.getValue();
				}
				
				if( btnCw.getModel().isPressed() == false &&
					btnCcw.getModel().isPressed() == false )
				{
					mRotateAccelerator.normalize(dt);
					
					if(stickRotateActive == false)
						rotateCtrl = mRotateAccelerator.getValue();
				}
				
				if( btnFwd.getModel().isPressed() == false &&
					btnBck.getModel().isPressed() == false)
				{
					mPitchAccelerator.normalize(dt);
					
					if(stickPitchActive == false)
						pitchCtrl = mPitchAccelerator.getValue();
				}
				
				if( btnLeft.getModel().isPressed() == false && 
					btnRight.getModel().isPressed() == false )
				{
					mRollAccelerator.normalize(dt);
					
					if(stickRollActive == false)
						rollCtrl = mRollAccelerator.getValue();
				}
			}
			
			DroneCommander.instance().setTrickMode(requestTrickMode);;
			DroneCommander.instance().liftDrone(liftCtrl);
			DroneCommander.instance().rotateDrone(rotateCtrl);
			DroneCommander.instance().moveDrone(pitchCtrl,rollCtrl);
			
			DroneState ds = DroneTelemetry.instance().getDroneState();
			
			// LOAD1
			if(btnLoad1.getModel().isPressed() && ds.load1.enabled == false)
			{
				CmdSetLoad cmd = new CmdSetLoad(0,true,ds.load1.period);
				DroneCommander.instance().addCmd(cmd);
			}
			if(btnLoad1.getModel().isPressed() == false && ds.load1.enabled)
			{
				CmdSetLoad cmd = new CmdSetLoad(0,false,ds.load1.period);
				DroneCommander.instance().addCmd(cmd);
			}
			// LOAD2
			if(btnLoad2.getModel().isPressed() && ds.load2.enabled == false)
			{
				CmdSetLoad cmd = new CmdSetLoad(1,true,ds.load2.period);
				DroneCommander.instance().addCmd(cmd);
			}
			if(btnLoad2.getModel().isPressed() == false && ds.load2.enabled)
			{
				CmdSetLoad cmd = new CmdSetLoad(1,false,ds.load2.period);
				DroneCommander.instance().addCmd(cmd);
			}
			// ANTI TURTLE
			DroneCommander.instance().antiTurtle(btnAntiTurtle.getModel().isPressed(),DroneState.Motors.antiTurtleGas);
			// Camera Move
			if(btnCamAng0.getModel().isPressed())
			{
				DroneCommander.instance().setCameraAngle(0);
			}
			if(btnCamAng45.getModel().isPressed())
			{
				DroneCommander.instance().setCameraAngle(45);
			}
			if(btnCamAng90.getModel().isPressed())
			{
				DroneCommander.instance().setCameraAngle(90);
			}
			// Photo
			if(btnPhoto.getModel().isPressed() && oldBtnPhotoState == false)
			{
				ResBox.sound("PHOTO_TAKEN").play();
				DroneCommander.instance().takePhoto();
				
			}
			oldBtnPhotoState = btnPhoto.getModel().isPressed();
			// Video
			if(btnVideo.isSelected() && oldBtnVideoState == false)
			{
				DroneCommander.instance().startVideo();
			}
			else if(btnVideo.isSelected() == false && oldBtnVideoState == true)
			{
				DroneCommander.instance().stopVideo();
			}
			else
			{
				btnVideo.setSelected(ds.videoState);
			}
			oldBtnVideoState = btnVideo.isSelected();
		}
	}
	
	private class OnWndListener implements WindowListener
	{

		@Override
		public void windowOpened(WindowEvent e) {}

		@Override
		public void windowClosing(WindowEvent e)
		{
			if(tmKeyboard != null)
			{
				tmKeyboard.cancel();
				tmKeyboard = null;
			}
		}

		@Override
		public void windowClosed(WindowEvent e) 
		{
			if(tmKeyboard != null)
			{
				tmKeyboard.cancel();
				tmKeyboard = null;
			}
		}

		@Override
		public void windowIconified(WindowEvent e) {}

		@Override
		public void windowDeiconified(WindowEvent e) {}

		@Override
		public void windowActivated(WindowEvent e)
		{
			lblGamepad.setText("<html>" + ResBox.text("GAMEPAD_NOT_FOUND") + "</html>"); // html is added to automatically wrap text to new line
			Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();
			
			String curGamepad = AppSettings.instance().getInputMap().gamepad;

			for(int i = 0; i < controllers.length; i++)
			{
				Controller controller = controllers[i];
				
				if(	controller.getType() == Controller.Type.GAMEPAD &&
					controller.getName().compareToIgnoreCase(curGamepad) == 0)
				{
					lblGamepad.setText("<html>" + controller.getName() + "</html>");
				}
			}
			
			btnStop.requestFocus();
			
			if(tmKeyboard == null)
			{
				tmKeyboard = new Timer("RcInputTimer");
				tmKeyboard.scheduleAtFixedRate(new OnInputsPoll(), 100, KEYBOARD_PERIOD);
			}
		}

		@Override
		public void windowDeactivated(WindowEvent e) {}
	}

	public RemoteControlGui()
	{
		super(ResBox.text("REMOTE_CONTROL"),253,298);
		createUI();
		rcSettingsGui = new RcSettingsGui();
	}
	
	private void createUI()
	{
		this.setLayout(new MigLayout("","[grow][][]"));
		this.setIconImage(ResBox.icon("REMOTE_CONTROL").getImage());

		mtfAlt = new JTextField();
		mtfAlt.setDocument(new NumericDocument(3,true));
		mtfAlt.setText("0");
		
		JButton btnSendYpr = new JButton(ResBox.text("SEND"));
		btnSendYpr.addActionListener(new OnBtnSend());
		
		JButton btnRcSettings = new JButton(ResBox.icon("SETTINGS"));
		btnRcSettings.addActionListener(new OnBtnSettings());

		btnGetUp = new JButton(ResBox.icon("ARROW_GET_UP"));
		btnGetDown = new JButton(ResBox.icon("ARROW_GET_DOWN"));
		btnLeft = new JButton(ResBox.icon("ARROW_LEFT"));
		btnRight = new JButton(ResBox.icon("ARROW_RIGHT"));
		btnFwd = new JButton(ResBox.icon("ARROW_UP"));
		btnBck = new JButton(ResBox.icon("ARROW_DOWN"));
		btnCw = new JButton(ResBox.icon("ROTATE_CW"));
		btnCcw = new JButton(ResBox.icon("ROTATE_CCW"));
		btnStop = new JButton(ResBox.icon("STOP"));
		
		btnLoad1 = new JButton(ResBox.text("LOAD1"));
		btnLoad2 = new JButton(ResBox.text("LOAD2"));
		//btnLoad3 = new JButton(ResBox.text("LOAD3"));
		btnAntiTurtle = new JButton(ResBox.text("ANTITURTLE"));
		btnCamAng0 = new JButton(ResBox.text("CAMERA_ANGLE_0"));
		btnCamAng45 = new JButton(ResBox.text("CAMERA_ANGLE_45"));
		btnCamAng90 = new JButton(ResBox.text("CAMERA_ANGLE_90"));
		btnPhoto = new JButton(ResBox.text("PHOTO"));
		btnVideo = new JToggleButton(ResBox.text("VIDEO"));
		
		btnStop.requestFocus();
		
		JPanel pnlCtrl = new JPanel(new MigLayout());
		
		pnlCtrl.add(new JLabel(ResBox.text("ALTITUDE") + "(m)"));
		pnlCtrl.add(mtfAlt,"grow");
		pnlCtrl.add(btnSendYpr,"grow,wrap");
		
		lblGamepad = new JLabel();
		lblGamepad.setMaximumSize(new java.awt.Dimension(130,30));
		
		pnlCtrl.add(lblGamepad,"spanx 2, align left");
		pnlCtrl.add(btnRcSettings,"growx,wrap");
		pnlCtrl.add(btnCcw,"growx");
		pnlCtrl.add(btnFwd,"growx");
		pnlCtrl.add(btnCw,"growx,wrap");
		pnlCtrl.add(btnLeft,"growx");
		pnlCtrl.add(btnStop,"growx");
		pnlCtrl.add(btnRight,"growx,wrap");
		pnlCtrl.add(btnGetUp,"growx");
		pnlCtrl.add(btnBck,"growx");
		pnlCtrl.add(btnGetDown,"growx,wrap");
		pnlCtrl.add(btnLoad1,"growx");
		pnlCtrl.add(btnLoad2,"growx");
		//pnlCtrl.add(btnLoad3,"wrap");
		pnlCtrl.add(btnAntiTurtle,"growx,wrap");
		pnlCtrl.add(btnCamAng0,"growx");
		pnlCtrl.add(btnCamAng45,"growx");
		pnlCtrl.add(btnCamAng90,"growx,wrap");
		pnlCtrl.add(btnPhoto,"growx");
		pnlCtrl.add(btnVideo,"growx");
		
		this.add(pnlCtrl,"span,grow");	
		
		this.addWindowListener(new OnWndListener());
	}
	
	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getRcWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setRcWnd(ws);	
	}
	
	public void onEmergencyStop()
	{
		pdl.commands.CmdSwitchMotors cmd = new pdl.commands.CmdSwitchMotors(false);
		
		DroneCommander.instance().addCmd(cmd);
	}
}
