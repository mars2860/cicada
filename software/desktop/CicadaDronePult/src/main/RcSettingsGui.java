package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.JToggleButton;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import helper.NumericDocument;
import main.AppSettings.WndState;
import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;
import net.miginfocom.swing.MigLayout;
import pdl.DroneState;
import pdl.res.Profile;

public class RcSettingsGui extends JSavedFrame
{
	private static final long serialVersionUID = -296860419559339671L;
	
	private JRadioButton rbRotateByRate;
	private JRadioButton rbRotateByHeading;
	private JTextField tfRotateDelta;
	private JRadioButton rbMoveByRate;
	private JRadioButton rbMoveByAng;
	private JRadioButton rbMoveByVel;
	private JTextField tfMoveDelta;
	private JRadioButton rbLiftByGas;
	private JRadioButton rbLiftByForsage;
	private JRadioButton rbLiftByVel;
	private JRadioButton rbLiftByAlt;
	private JTextField tfLiftDelta;
	private JTextField tfForsageUpGas;
	private JTextField tfForsageMiddleGas;
	private JTextField tfForsageMaxMiddleGas;
	private JTextField tfForsageDownGas;
	private JTextField tfTrickAngRate;
	private JTextField tfLiftAccelTime;
	private JTextField tfMoveAccelTime;
	private JTextField tfRotateAccelTime;
	private JComboBox<String> cbGamepad;
	private JCheckBox cbPitchAutoLevel;
	private JCheckBox cbRollAutoLevel;
	
	private CtrlGuiBundle cgb[];
	
	private ArrayList<Controller> foundControllers;
	private ArrayList<Component> foundSticks;
	
	private Object inputSync;
	private Timer inputTimer;
	
	private class CtrlGuiBundle
	{
		public JComboBox<String> cbStick;
		public JProgressBar pbStickTest;
		public JToggleButton mapBtn1;
		public JToggleButton mapBtn2;
		
		private String selStick;
		
		// Clear button mapping when button is pressed
		private class OnMapBtnChanged implements ChangeListener
		{
			@Override
			public void stateChanged(ChangeEvent e)
			{
				Object obj = e.getSource();
				if(obj instanceof JToggleButton)
				{
					JToggleButton btn = (JToggleButton)obj;
					if(btn.isSelected())
						btn.setText("");
				}	
			}
		}
		
		public CtrlGuiBundle(AppSettings.InputCtrl ic)
		{
			cbStick = new JComboBox<String>();
			pbStickTest = new JProgressBar(JProgressBar.HORIZONTAL,0,100);
			mapBtn1 = new JToggleButton(ic.btn1);
			mapBtn2 = new JToggleButton(ic.btn2);
			selStick = ic.stick;
			mapBtn1.addChangeListener(new OnMapBtnChanged());
			mapBtn2.addChangeListener(new OnMapBtnChanged());
		}
		
		public void addToPanel(JPanel pnl, String nameKey)
		{
			pnl.add(new JLabel(ResBox.text(nameKey)));
			pnl.add(mapBtn1,"grow");
			pnl.add(mapBtn2,"grow");
			pnl.add(cbStick,"grow");
			pnl.add(pbStickTest,"grow,wrap");	
		}
		
		public void addSticks(ArrayList<Component> sticks)
		{
			cbStick.removeAllItems();
			
			if(selStick != null && selStick.isEmpty() == false)
			{
				cbStick.addItem(selStick);
			}
			
			for(Component c : sticks)
			{
				if(c.getIdentifier().getName().compareToIgnoreCase(selStick) != 0)	// add only new sticks
				{
					cbStick.addItem(c.getIdentifier().getName());
				}
			}
			
			if(selStick != null && selStick.isEmpty() == false)
			{
				cbStick.setSelectedItem(selStick);
			}
		}
		
		public void stickTest(Component comp)
		{
			if(cbStick.getItemCount() == 0)
				return;
			
			String selStickId = cbStick.getSelectedItem().toString();
			
			if(comp.getIdentifier().getName().compareTo(selStickId) == 0)
			{
				int val = (int)(comp.getPollData()*50.f + 50.f);
				pbStickTest.setValue(val);
			}
		}
		
		public void mapButton(Component comp, boolean isGamepadBtn)
		{
			if(comp.isAnalog())
				return;
			
			String name = "";
			
			if(isGamepadBtn)
			{
				if(comp.getIdentifier() == Component.Identifier.Axis.POV)
				{
					float povPos = comp.getPollData();
					
					if(Float.compare(povPos,Component.POV.UP) == 0)
					{
						name = "G_UP";
					}
					else if(Float.compare(povPos,Component.POV.DOWN) == 0)
					{
						name = "G_DOWN";
					}
					else if(Float.compare(povPos,Component.POV.LEFT) == 0)
					{
						name = "G_LEFT";
					}
					else if(Float.compare(povPos,Component.POV.RIGHT) == 0)
					{
						name = "G_RIGHT";
					}
				}
				else
				{
					name = "G_" + comp.getIdentifier().getName();
				}
			}
			else
			{
				name = comp.getIdentifier().getName();
			}
			
			if(mapBtn1.getModel().isSelected())
			{
				mapBtn1.setText(name);
				mapBtn1.getModel().setSelected(false);
			}
			
			if(mapBtn2.getModel().isSelected())
			{
				mapBtn2.setText(name);
				mapBtn2.getModel().setSelected(false);
			}
		}
	}

	public RcSettingsGui()
	{
		super(ResBox.text("SETTINGS"),1020,560);
		cgb = new CtrlGuiBundle[AppSettings.InputMap.INPUT_CONTROL_COUNT];
		
		for(int i = 0; i < AppSettings.InputMap.INPUT_CONTROL_COUNT;i++)
		{
			cgb[i] = new CtrlGuiBundle(AppSettings.instance().getInputMap().controls[i]);
		}
		
		inputSync = new Object();
		
		this.createUI();
	}
	
	private class OnWndListener implements WindowListener
	{
		@Override
		public void windowOpened(WindowEvent e) {}
		@Override
		public void windowClosing(WindowEvent e)
		{
			if(inputTimer != null)
			{
				inputTimer.cancel();
				inputTimer = null;
			}
		}
		@Override
		public void windowClosed(WindowEvent e) 
		{
			if(inputTimer != null)
			{
				inputTimer.cancel();
				inputTimer = null;
			}
		}
		@Override
		public void windowIconified(WindowEvent e) {}
		@Override
		public void windowDeiconified(WindowEvent e) {}
		@Override
		public void windowActivated(WindowEvent e)
		{
			if(inputTimer == null)
			{
				inputTimer = new Timer("RcSettingsInputTimer");
				inputTimer.scheduleAtFixedRate(new OnInputsPoll(), 100, RemoteControlGui.KEYBOARD_PERIOD);
			}
		}
		@Override
		public void windowDeactivated(WindowEvent e) {}
	}
	
	private class OnInputsPoll extends TimerTask
	{
		@Override
		public void run()
		{
			synchronized(inputSync)
			{
				Event event = new Event();

				Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();

				for(int i = 0; i < controllers.length; i++)
				{
					Controller controller = controllers[i];
					
					if(	(controller.getType() == Controller.Type.KEYBOARD) ||
						(controller.getType() == Controller.Type.GAMEPAD && controller.getName().compareToIgnoreCase(cbGamepad.getSelectedItem().toString()) == 0)
					  )
					{
						controller.poll();
						
					    EventQueue queue = controller.getEventQueue();

					    while (queue.getNextEvent(event))
					    {
					        Component comp = event.getComponent();

					        for(int j = 0; j < AppSettings.InputMap.INPUT_CONTROL_COUNT; j++)
				        	{
					        	if(comp.isAnalog())
					        	{
					        		cgb[j].stickTest(comp);
					        	}
					        	else
					        	{
					        		cgb[j].mapButton(comp,(controller.getType() == Controller.Type.GAMEPAD)?true:false);
					        	}
				        	}
					    }
					}
				}
			}
		}
	}
	
	private class OnBtnApply implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			DroneState.RemoteCtrl rc = grabRemoteCtrlSettings();
			DroneState.rc = rc;
			
			DroneState ds = Profile.instance().getDroneSettings();
			
			if( (rc.liftBy == DroneState.RemoteCtrl.LiftBy.LIFT_BY_GAS_DELTA ||
			     rc.liftBy == DroneState.RemoteCtrl.LiftBy.LIFT_BY_GAS_FORSAGE) && ds.velocityZPid.enabled )
			{
				JOptionPane.showMessageDialog(
						RcSettingsGui.this,
						ResBox.text("TURN_OFF_VELOZ_PID"),
						ResBox.text("ERROR"),
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			if( rc.liftBy == DroneState.RemoteCtrl.LiftBy.LIFT_BY_VELOCITY && !ds.velocityZPid.enabled )
			{
				JOptionPane.showMessageDialog(
						RcSettingsGui.this,
						ResBox.text("TURN_ON_VELOZ_PID"),
						ResBox.text("ERROR"),
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			if( rc.liftBy == DroneState.RemoteCtrl.LiftBy.LIFT_BY_ALT && (!ds.velocityZPid.enabled || !ds.altPid.enabled) )
			{
				JOptionPane.showMessageDialog(
						RcSettingsGui.this,
						ResBox.text("TURN_ON_VELOZ_PID"),
						ResBox.text("ERROR"),
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			if(cbGamepad.getItemCount() > 0)
			{
				AppSettings.instance().getInputMap().gamepad = cbGamepad.getSelectedItem().toString();
			}
			
			for(int i = 0; i < AppSettings.InputMap.INPUT_CONTROL_COUNT; i++)
			{
				AppSettings.InputMap im = AppSettings.instance().getInputMap();
				
				im.controls[i].btn1 = cgb[i].mapBtn1.getText();
				im.controls[i].btn2 = cgb[i].mapBtn2.getText();
				if(cgb[i].cbStick.getItemCount() > 0)
				{
					im.controls[i].stick = cgb[i].cbStick.getSelectedItem().toString();
				}
				else
				{
					im.controls[i].stick = "";
				}
			}
			
			AppSettings.instance().save();
			Profile.instance().save();
		}
	}

	protected void createUI()
	{
		this.setLayout(new MigLayout("","[][][grow]"));
		this.setTitle(ResBox.text("SETTINGS"));
		this.setIconImage(ResBox.icon("SETTINGS").getImage());
		// ---------------------------------------------------------------------
		NumericDocument numDoc1 = new NumericDocument(3, false);
		NumericDocument numDoc2 = new NumericDocument(3, false);
		NumericDocument numDoc3 = new NumericDocument(3, false);
		
		// ---------------------------------------------------------------------
		// Rotate ctrl

		JPanel pnlRotateCtrl = new JPanel(new MigLayout("", "[][][grow][][]"));
		pnlRotateCtrl.setBorder(new TitledBorder(ResBox.text("ROTATE_BY")));

		rbRotateByRate = new JRadioButton(ResBox.text("YAW_RATE"));
		rbRotateByHeading = new JRadioButton(ResBox.text("HEADING"));
		tfRotateDelta = new JTextField();

		ButtonGroup grp1 = new ButtonGroup();
		grp1.add(rbRotateByRate);
		grp1.add(rbRotateByHeading);
		
		tfRotateDelta.setDocument(numDoc1);

		pnlRotateCtrl.add(rbRotateByRate);
		pnlRotateCtrl.add(rbRotateByHeading);
		pnlRotateCtrl.add(new JLabel(), "grow");
		pnlRotateCtrl.add(new JLabel(ResBox.text("DELTA")));
		pnlRotateCtrl.add(tfRotateDelta, "w 40px");

		this.add(pnlRotateCtrl, "spanx 2,grow");
		
		//----------------------------------------------------------------------
		// Control settings
			
		JPanel pnlCtrlSettings = new JPanel(new MigLayout("","[][90!][90!][grow][grow]"));
		pnlCtrlSettings.setBorder(new TitledBorder(ResBox.text("CONTROL_SETTINGS")));
				
		cbGamepad = new JComboBox<String>();
		
		pnlCtrlSettings.add(new JLabel(ResBox.text("GAMEPAD")));
		pnlCtrlSettings.add(cbGamepad,"growx,spanx,wrap");
		pnlCtrlSettings.add(new JLabel());
		pnlCtrlSettings.add(new JLabel());
		pnlCtrlSettings.add(new JLabel());
		pnlCtrlSettings.add(new JLabel(ResBox.text("STICK")),"align center");
		pnlCtrlSettings.add(new JLabel(ResBox.text("STICK_TEST")),"align center,wrap");
		
		cgb[AppSettings.InputMap.UP].addToPanel(pnlCtrlSettings,"UP");
		cgb[AppSettings.InputMap.DOWN].addToPanel(pnlCtrlSettings,"DOWN");
		cgb[AppSettings.InputMap.FWD].addToPanel(pnlCtrlSettings,"FORWARD");
		cgb[AppSettings.InputMap.BACK].addToPanel(pnlCtrlSettings,"BACK");
		cgb[AppSettings.InputMap.LEFT].addToPanel(pnlCtrlSettings,"LEFT");
		cgb[AppSettings.InputMap.RIGHT].addToPanel(pnlCtrlSettings,"RIGHT");
		cgb[AppSettings.InputMap.TURN_CW].addToPanel(pnlCtrlSettings,"TURN_CW");
		cgb[AppSettings.InputMap.TURN_CCW].addToPanel(pnlCtrlSettings,"TURN_CCW");
		cgb[AppSettings.InputMap.ARM_DISARM].addToPanel(pnlCtrlSettings,"ARM_DISARM");
		cgb[AppSettings.InputMap.TRICK_MODE].addToPanel(pnlCtrlSettings,"TRICK_MODE");
		cgb[AppSettings.InputMap.LOAD1].addToPanel(pnlCtrlSettings,"LOAD1");
		cgb[AppSettings.InputMap.LOAD2].addToPanel(pnlCtrlSettings,"LOAD2");
		cgb[AppSettings.InputMap.ANTITURTLE].addToPanel(pnlCtrlSettings,"ANTITURTLE");
		cgb[AppSettings.InputMap.CAMERA_ANGLE_0].addToPanel(pnlCtrlSettings,"CAMERA_ANGLE_0");
		cgb[AppSettings.InputMap.CAMERA_ANGLE_45].addToPanel(pnlCtrlSettings,"CAMERA_ANGLE_45");
		cgb[AppSettings.InputMap.CAMERA_ANGLE_90].addToPanel(pnlCtrlSettings,"CAMERA_ANGLE_90");
		cgb[AppSettings.InputMap.PHOTO].addToPanel(pnlCtrlSettings,"PHOTO");
		cgb[AppSettings.InputMap.VIDEO].addToPanel(pnlCtrlSettings,"VIDEO");
		
		// disable to map sticks for commands
		for(int i = AppSettings.InputMap.ARM_DISARM; i < AppSettings.InputMap.INPUT_CONTROL_COUNT; i++)
		{
			cgb[i].cbStick.setSelectedIndex(-1);
			cgb[i].cbStick.removeAllItems();
			cgb[i].cbStick.setEnabled(false);
			cgb[i].pbStickTest.setEnabled(false);
		}
				
		JScrollPane pnlCtrlScrollPane = new JScrollPane(pnlCtrlSettings);
		this.add(pnlCtrlScrollPane,"spany 5,grow,wrap");

		// ---------------------------------------------------------------------
		// Move ctrl

		JPanel pnlMoveCtrl = new JPanel(new MigLayout("", "[][][][][][grow][][]"));
		pnlMoveCtrl.setBorder(new TitledBorder(ResBox.text("MOVE_BY")));

		rbMoveByRate = new JRadioButton(ResBox.text("PITCH_ROLL_RATE"));
		rbMoveByAng = new JRadioButton(ResBox.text("PITCH_ROLL"));
		rbMoveByVel = new JRadioButton(ResBox.text("VELOCITY"));

		ButtonGroup grp2 = new ButtonGroup();

		grp2.add(rbMoveByAng);
		grp2.add(rbMoveByRate);
		grp2.add(rbMoveByVel);

		tfMoveDelta = new JTextField();
		tfMoveDelta.setDocument(numDoc2);
		
		cbPitchAutoLevel = new JCheckBox(ResBox.text("PITCH_AUTO_LEVEL"));
		cbRollAutoLevel = new JCheckBox(ResBox.text("ROLL_AUTO_LEVEL"));
		
		pnlMoveCtrl.add(rbMoveByRate);
		pnlMoveCtrl.add(rbMoveByAng);
		pnlMoveCtrl.add(rbMoveByVel);
		pnlMoveCtrl.add(cbPitchAutoLevel);
		pnlMoveCtrl.add(cbRollAutoLevel);
		pnlMoveCtrl.add(new JLabel(), "grow");
		pnlMoveCtrl.add(new JLabel(ResBox.text("DELTA")));
		pnlMoveCtrl.add(tfMoveDelta, "w 40px");

		this.add(pnlMoveCtrl, "spanx 2,grow,wrap");

		// ---------------------------------------------------------------------
		// Lift ctrl

		JPanel pnlLiftCtrl = new JPanel(new MigLayout("", "[][][][grow][][]"));
		pnlLiftCtrl.setBorder(new TitledBorder(ResBox.text("LIFT_BY")));

		rbLiftByGas = new JRadioButton(ResBox.text("GAS"));
		rbLiftByForsage = new JRadioButton(ResBox.text("FORSAGE"));
		rbLiftByVel = new JRadioButton(ResBox.text("VELOCITY"));
		rbLiftByAlt = new JRadioButton(ResBox.text("ALT"));

		ButtonGroup grp3 = new ButtonGroup();
		grp3.add(rbLiftByGas);
		grp3.add(rbLiftByForsage);
		grp3.add(rbLiftByVel);
		grp3.add(rbLiftByAlt);

		tfLiftDelta = new JTextField();
		tfLiftDelta.setDocument(numDoc3);
		
		pnlLiftCtrl.add(rbLiftByGas);
		pnlLiftCtrl.add(rbLiftByForsage);
		pnlLiftCtrl.add(rbLiftByVel);
		pnlLiftCtrl.add(rbLiftByAlt);
		pnlLiftCtrl.add(new JLabel(), "grow");
		pnlLiftCtrl.add(new JLabel(ResBox.text("DELTA")));
		pnlLiftCtrl.add(tfLiftDelta, "w 40px");

		this.add(pnlLiftCtrl, "spanx 2,grow,wrap");

		// ---------------------------------------------------------------------
		// Forsage ctrl
		
		NumericDocument numDoc4 = new NumericDocument(0, false);
		NumericDocument numDoc5 = new NumericDocument(0, false);
		NumericDocument numDoc55 = new NumericDocument(0, false);
		NumericDocument numDoc6 = new NumericDocument(0, false);
		NumericDocument numDoc66 = new NumericDocument(0, false);

		JPanel pnlForsageCtrl = new JPanel(new MigLayout("", "[][][][][][][][][]"));
		pnlForsageCtrl.setBorder(new TitledBorder(ResBox.text("FORSAGE")));

		tfForsageUpGas = new JTextField();
		tfForsageUpGas.setDocument(numDoc4);
		
		tfForsageMiddleGas = new JTextField();
		tfForsageMiddleGas.setDocument(numDoc5);
		
		tfForsageMaxMiddleGas = new JTextField();
		tfForsageMaxMiddleGas.setDocument(numDoc55);

		tfForsageDownGas = new JTextField();
		tfForsageDownGas.setDocument(numDoc6);
		
		tfTrickAngRate = new JTextField();
		tfTrickAngRate.setDocument(numDoc66);

		pnlForsageCtrl.add(new JLabel(), "grow");
		pnlForsageCtrl.add(new JLabel(ResBox.text("FORSAGE_UP_GAS")));
		pnlForsageCtrl.add(tfForsageUpGas, "w 40px");
		pnlForsageCtrl.add(new JLabel(ResBox.text("FORSAGE_MIDDLE_GAS")));
		pnlForsageCtrl.add(tfForsageMiddleGas, "w 40px");
		pnlForsageCtrl.add(new JLabel(ResBox.text("FORSAGE_MAX_MIDDLE_GAS")));
		pnlForsageCtrl.add(tfForsageMaxMiddleGas, "w 40px");
		pnlForsageCtrl.add(new JLabel(ResBox.text("FORSAGE_DOWN_GAS")));
		pnlForsageCtrl.add(tfForsageDownGas, "w 40px");
		pnlForsageCtrl.add(new JLabel(ResBox.text("TRICK_ANG_RATE")));
		pnlForsageCtrl.add(tfTrickAngRate, "w 40px");

		this.add(pnlForsageCtrl, "spanx 2,grow,wrap");
		
		// ---------------------------------------------------------------------
		// Acceleration
		
		NumericDocument numDoc7 = new NumericDocument(3, false);
		NumericDocument numDoc8 = new NumericDocument(3, false);
		NumericDocument numDoc9 = new NumericDocument(3, false);

		JPanel pnlAccelCtrl = new JPanel(new MigLayout("", "[][][][][][]"));
		pnlAccelCtrl.setBorder(new TitledBorder(ResBox.text("ACCELERATION_TIME") + " (" + ResBox.text("SECONDS") + ")"));

		tfLiftAccelTime = new JTextField();
		tfLiftAccelTime.setDocument(numDoc7);
		
		tfMoveAccelTime = new JTextField();
		tfMoveAccelTime.setDocument(numDoc8);
		
		tfRotateAccelTime = new JTextField();
		tfRotateAccelTime.setDocument(numDoc9);
		
		pnlAccelCtrl.add(new JLabel(ResBox.text("LIFT")));
		pnlAccelCtrl.add(tfLiftAccelTime,"w 40px");
		pnlAccelCtrl.add(new JLabel(ResBox.text("MOVE")));
		pnlAccelCtrl.add(tfMoveAccelTime,"w 40px");
		pnlAccelCtrl.add(new JLabel(ResBox.text("ROTATE")));
		pnlAccelCtrl.add(tfRotateAccelTime,"w 40px");

		this.add(pnlAccelCtrl,"spanx 2,grow,wrap");
		
		//---------------------------------------------------------------------
		//

		JButton btnApply = new JButton(ResBox.text("APPLY"));
		btnApply.addActionListener(new OnBtnApply());

		this.add(new JLabel(), "grow,spanx 2");
		this.add(btnApply,"align right");
		
		this.addWindowListener(new OnWndListener());
		
		updateUI();
	}
	
	protected void updateUI()
	{
		DroneState.RemoteCtrl rc = DroneState.rc;
		
		NumberFormat numFmt = new DecimalFormat();
		numFmt.setMaximumFractionDigits(3);
		numFmt.setGroupingUsed(false);
		
		switch(rc.rotateBy)
		{
			case ROTATE_BY_HEADING:
				rbRotateByHeading.setSelected(true);
				break;
			default:
				rbRotateByRate.setSelected(true);
				break;
		}

		switch(rc.moveBy)
		{
			case MOVE_BY_PITCH_ROLL:
				rbMoveByAng.setSelected(true);
				break;
			case MOVE_BY_VELOCITY:
				rbMoveByVel.setSelected(true);
				break;
			default:
				rbMoveByRate.setSelected(true);
				break;
		}

		switch(rc.liftBy)
		{
			case LIFT_BY_VELOCITY:
				rbLiftByVel.setSelected(true);
				break;
			case LIFT_BY_ALT:
				rbLiftByAlt.setSelected(true);
				break;
			case LIFT_BY_GAS_FORSAGE:
				rbLiftByForsage.setSelected(true);
				break;
			default:
				rbLiftByGas.setSelected(true);
				break;
		}
		
		tfRotateDelta.setText(numFmt.format(rc.rotateDelta));		
		tfMoveDelta.setText(numFmt.format(rc.moveDelta));
		tfLiftDelta.setText(numFmt.format(rc.liftDelta));
		
		tfForsageUpGas.setText(numFmt.format(rc.maxGas));
		tfForsageMiddleGas.setText(numFmt.format(rc.middleGas));
		tfForsageMaxMiddleGas.setText(numFmt.format(rc.maxMiddleGas));
		tfForsageDownGas.setText(numFmt.format(rc.minGas));
		
		tfTrickAngRate.setText(numFmt.format(rc.trickDelta));
		
		tfLiftAccelTime.setText(numFmt.format(rc.liftAccelTime));
		tfMoveAccelTime.setText(numFmt.format(rc.moveAccelTime));
		tfRotateAccelTime.setText(numFmt.format(rc.rotateAccelTime));
		
		cbPitchAutoLevel.setSelected(rc.pitchAutoLevel);
		cbRollAutoLevel.setSelected(rc.rollAutoLevel);
		
		searchForGamepads();
		
		for(int i = 0; i < AppSettings.InputMap.INPUT_CONTROL_COUNT; i++)
		{
			AppSettings.InputMap im = AppSettings.instance().getInputMap();
			
			cgb[i].mapBtn1.setText(im.controls[i].btn1); 
			cgb[i].mapBtn2.setText(im.controls[i].btn2);
			if(cgb[i].cbStick.isEnabled())
			{
				cgb[i].cbStick.setSelectedItem(im.controls[i].stick); 
			}
			else
			{
				cgb[i].cbStick.setSelectedIndex(-1);
				cgb[i].cbStick.removeAllItems();
			}
		}
	}
	
	protected DroneState.RemoteCtrl grabRemoteCtrlSettings()
	{
		DroneState.RemoteCtrl rc = new DroneState.RemoteCtrl();
		
		if(rbRotateByHeading.isSelected())
			rc.rotateBy = DroneState.RemoteCtrl.RotateBy.ROTATE_BY_HEADING;
		else
			rc.rotateBy = DroneState.RemoteCtrl.RotateBy.ROTATE_BY_YAW_RATE;
		
		rc.rotateDelta = Float.parseFloat(tfRotateDelta.getText());
		
		if(rbMoveByAng.isSelected())
			rc.moveBy = DroneState.RemoteCtrl.MoveBy.MOVE_BY_PITCH_ROLL;
		else if(rbMoveByVel.isSelected())
			rc.moveBy = DroneState.RemoteCtrl.MoveBy.MOVE_BY_VELOCITY;
		else
			rc.moveBy = DroneState.RemoteCtrl.MoveBy.MOVE_BY_PITCH_ROLL_RATE;
		
		rc.moveDelta = Float.parseFloat(tfMoveDelta.getText());
		
		if(rbLiftByVel.isSelected())
			rc.liftBy = DroneState.RemoteCtrl.LiftBy.LIFT_BY_VELOCITY;
		else if(rbLiftByAlt.isSelected())
			rc.liftBy = DroneState.RemoteCtrl.LiftBy.LIFT_BY_ALT;
		else if(rbLiftByForsage.isSelected())
			rc.liftBy = DroneState.RemoteCtrl.LiftBy.LIFT_BY_GAS_FORSAGE;
		else
			rc.liftBy = DroneState.RemoteCtrl.LiftBy.LIFT_BY_GAS_DELTA;
		
		rc.liftDelta = Float.parseFloat(tfLiftDelta.getText());
		
		rc.maxGas = Integer.parseInt(tfForsageUpGas.getText());
		rc.middleGas = Integer.parseInt(tfForsageMiddleGas.getText());
		rc.maxMiddleGas = Integer.parseInt(tfForsageMaxMiddleGas.getText());
		rc.minGas = Integer.parseInt(tfForsageDownGas.getText());
		
		rc.trickDelta = Integer.parseInt(tfTrickAngRate.getText());
		
		rc.liftAccelTime = Float.parseFloat(tfLiftAccelTime.getText());
		rc.moveAccelTime = Float.parseFloat(tfMoveAccelTime.getText());
		rc.rotateAccelTime = Float.parseFloat(tfRotateAccelTime.getText());
		
		rc.pitchAutoLevel = cbPitchAutoLevel.isSelected();
		rc.rollAutoLevel = cbRollAutoLevel.isSelected();
		
		return rc;
	}
	
	private void searchForGamepads()
	{
		synchronized(inputSync)
		{
			foundControllers = new ArrayList<Controller>();
			foundSticks = new ArrayList<Component>();
			cbGamepad.removeAllItems();
			
			String curGamepad = AppSettings.instance().getInputMap().gamepad;
			
			// add gamepad from settings
			if( curGamepad != null &&
				curGamepad.isEmpty() == false )
			{
				cbGamepad.addItem(curGamepad);
			}
		
			Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();

			for(int i = 0; i < controllers.length; i++)
			{
				Controller controller = controllers[i];
	            
				if( controller.getType() == Controller.Type.GAMEPAD )       
				{
					foundControllers.add(controller);
					
					if(controller.getName().compareToIgnoreCase(curGamepad) != 0)	// add only new gamepads are connected
					{
						cbGamepad.addItem(controller.getName());
					}
	            
					Component cmps[] = controller.getComponents();
	            
					for(Component c : cmps)
					{
						if(c.isAnalog())
						{
							foundSticks.add(c);
						}
					}
				}
			}
		
			if(cbGamepad.getItemCount() > 0)
			{
				cbGamepad.setSelectedItem(curGamepad);
			}
			
			for(int i = 0; i < AppSettings.InputMap.INPUT_CONTROL_COUNT - 2; i++)
			{
				cgb[i].addSticks(foundSticks);
			}
		}
	}

	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getRcSettingsWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setRcSettingsWnd(ws);	
	}
	
	@Override
	public void setVisible(boolean b)
	{
		super.setVisible(b);
		updateUI();
	}
}
