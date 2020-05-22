package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Observable;
import java.util.Observer;

import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComponent;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.KeyStroke;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import ChartDirector.ChartViewer;
import ChartDirector.XYChart;
import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.CopterTelemetry.DroneState;
import copter.commands.CmdSetAltPid;
import copter.commands.CmdSetAltitude;
import copter.commands.CmdSetBaseGas;
import copter.commands.CmdSetPitchPid;
import copter.commands.CmdSetRollPid;
import copter.commands.CmdSetYPR;
import copter.commands.CmdSetYawPid;
import copter.commands.TakeOff;
import helper.NumericDocument;
import net.miginfocom.swing.MigLayout;

public class PIDlg extends JDialog
{
	private static final long serialVersionUID = -2092334500542560656L;
	
	private class PidPanel extends JPanel
	{
		private static final long serialVersionUID = -7506067983673078938L;
		
		private JTextField mtfKp;
		private JTextField mtfKi;
		private JTextField mtfKd;
		private JSlider mslTarget;
		private JLabel mlbTarget;
		private JCheckBox mcbEnabled;
		private JLabel mlbEnabled;
		private JLabel mlbKp;
		private JLabel mlbKi;
		private JLabel mlbKd;
		private JButton mbtnSet;
		private DecimalFormat mKoefFmt;
		
		private class OnTargetChanged implements ChangeListener
		{
			@Override
			public void stateChanged(ChangeEvent e)
			{
				//if(!mslTarget.getValueIsAdjusting())
				{
					mlbTarget.setText(Integer.toString(mslTarget.getValue()));
				}
			}	
		}
		
		public PidPanel(String title, int minTarget, int maxTarget)
		{
			super();
			
			mKoefFmt = new DecimalFormat();
			mKoefFmt.setMaximumFractionDigits(3);
			mKoefFmt.setGroupingUsed(false);
			
			this.setBorder(new TitledBorder(title));
			this.setLayout(new MigLayout("","[]10[][grow][]",""));

			mtfKp = new JTextField();
			mtfKi = new JTextField();
			mtfKd = new JTextField();
			mlbKp = new JLabel("0");
			mlbKi = new JLabel("0");
			mlbKd = new JLabel("0");		
			mlbEnabled = new JLabel();
			mcbEnabled = new JCheckBox(Text.get("PID_ENABLED"));
			mslTarget = new JSlider(JSlider.HORIZONTAL, minTarget, maxTarget, 0);
			mslTarget.setMinorTickSpacing(1);
			mlbTarget = new JLabel(Integer.toString(mslTarget.getValue()));
			
			mslTarget.addChangeListener(new OnTargetChanged());
			
			mtfKp.setHorizontalAlignment(JTextField.RIGHT);
			mtfKi.setHorizontalAlignment(JTextField.RIGHT);
			mtfKd.setHorizontalAlignment(JTextField.RIGHT);
			mlbKp.setHorizontalAlignment(JTextField.CENTER);
			mlbKi.setHorizontalAlignment(JTextField.CENTER);
			mlbKd.setHorizontalAlignment(JTextField.CENTER);
			
			mtfKp.setDocument(new NumericDocument(3,false));
			mtfKi.setDocument(new NumericDocument(3,false));
			mtfKd.setDocument(new NumericDocument(3,false));
			
			mbtnSet = new JButton(Text.get("SET"));
			
			JPanel pnlKoef = new JPanel(new MigLayout("insets 0 0 0 0"));
			
			pnlKoef.add(new JLabel("Kp"));
			pnlKoef.add(mtfKp,"w 60!");
			pnlKoef.add(new JLabel("Ki"));
			pnlKoef.add(mtfKi,"w 60!");
			pnlKoef.add(new JLabel("Kd"));
			pnlKoef.add(mtfKd,"w 60!");
			pnlKoef.add(mbtnSet,"wrap");
			pnlKoef.add(new JLabel("Cur"));
			pnlKoef.add(mlbKp,"grow");
			pnlKoef.add(new JPanel());
			pnlKoef.add(mlbKi,"grow");
			pnlKoef.add(new JPanel());
			pnlKoef.add(mlbKd,"grow");
			pnlKoef.add(mlbEnabled);

			this.add(mcbEnabled);
			this.add(new JLabel(Text.get("TARGET")));
			this.add(mslTarget,"grow");
			this.add(mlbTarget,"w 30!,wrap");
			this.add(pnlKoef,"spanx,grow");	
		}
		
		public void initBySettings(boolean enabled, float kp, float ki, float kd)
		{
			mcbEnabled.setSelected(enabled);
			mtfKp.setText(mKoefFmt.format(kp));
			mtfKi.setText(mKoefFmt.format(ki));
			mtfKd.setText(mKoefFmt.format(kd));
		}
		
		public boolean getPidEnabled()
		{
			return mcbEnabled.isSelected();
		}
		
		public void setLbKp(double kp)
		{
			mlbKp.setText(mKoefFmt.format(kp));
		}
		
		public float getKp()
		{
			return Float.parseFloat(mtfKp.getText());
		}
		
		public void setLbKi(double ki)
		{
			mlbKi.setText(mKoefFmt.format(ki));
		}
		
		public float getKi()
		{
			return Float.parseFloat(mtfKi.getText());
		}
		
		public void setLbKd(double kd)
		{
			mlbKd.setText(mKoefFmt.format(kd));
		}
		
		public float getKd()
		{
			return Float.parseFloat(mtfKd.getText());
		}
		
		public void setLbEnabled(boolean enabled)
		{
			mlbEnabled.setText("(" + Boolean.toString(enabled) + ")");
		}
		
		public float getTarget()
		{
			return mslTarget.getValue();
		}
		
		public void onSet(ActionListener ls)
		{
			mbtnSet.addActionListener(ls);
		}
		
		public void onTarget(ChangeListener ls)
		{
			mslTarget.addChangeListener(ls);
		}
	}
	
	private class GasChanged implements ChangeListener
	{
		@Override
		public void stateChanged(ChangeEvent e)
		{
			JSlider slider = (JSlider)e.getSource();
			
			if(!slider.getValueIsAdjusting())
			{
				int gas = slider.getValue();
				CopterCommander.instance().addCmd(new CmdSetBaseGas(gas));
			}
		}
	}
	
	private class OnAddGasKey extends AbstractAction
	{
		private static final long serialVersionUID = 1L;
		
		private long timer = System.currentTimeMillis();
		private int addval;
		
		public OnAddGasKey(int val)
		{
			addval = val;
		}

		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(System.currentTimeMillis() - timer >= 50)
			{
				timer = System.currentTimeMillis();
				mSetAllGas.addGas(addval);
			}	
		}
	}
	
	private class OnTakeOffKey extends AbstractAction
	{
		private static final long serialVersionUID = 1L;

		@Override
		public void actionPerformed(ActionEvent e)
		{
			TakeOff cmd = new copter.commands.TakeOff();
			cmd.run();
			mSetAllGas.setGas(cmd.getEndGas(), false);
		}
	}
	
	private class OnTurnKey extends AbstractAction
	{
		private static final long serialVersionUID = 1L;
		private float da;
		
		public OnTurnKey(float angle)
		{
			da = angle;
		}

		@Override
		public void actionPerformed(ActionEvent e)
		{
			CopterTelemetry.DroneState droneState = CopterTelemetry.instance().getDroneState();
			double yaw = droneState.yawRadRatePid.target;
			double pitch = droneState.pitchRadPid.target;
			double roll = droneState.rollRadPid.target;
			
			yaw += da;
			
			yaw = Math.toRadians(yaw);
			pitch = Math.toRadians(pitch);
			roll = Math.toRadians(roll);
			
			CmdSetYPR cmd = new CmdSetYPR((float)yaw,(float)pitch,(float)roll);
			CopterCommander.instance().addCmd(cmd);
		}
	}
	
	private class OnPitchKey extends AbstractAction
	{
		private static final long serialVersionUID = 1L;
		private float da;
		
		public OnPitchKey(float angle)
		{
			da = angle;
		}

		@Override
		public void actionPerformed(ActionEvent e)
		{
			CopterTelemetry.DroneState droneState = CopterTelemetry.instance().getDroneState();
			double yaw = droneState.yawRadRatePid.target;
			double pitch = droneState.pitchRadPid.target;
			double roll = droneState.rollRadPid.target;
			
			if( Math.abs(pitch - da) > 0.1f)
			{
				pitch = da;
			
				yaw = (float)Math.toRadians(yaw);
				pitch = (float)Math.toRadians(pitch);
				roll = (float)Math.toRadians(roll);
			
				CmdSetYPR cmd = new CmdSetYPR((float)yaw,(float)pitch,(float)roll);
				CopterCommander.instance().addCmd(cmd);
			}
		}
	}
	
	private class OnRollKey extends AbstractAction
	{
		private static final long serialVersionUID = 1L;
		private float da;
		
		public OnRollKey(float angle)
		{
			da = angle;
		}

		@Override
		public void actionPerformed(ActionEvent e)
		{
			CopterTelemetry.DroneState droneState = CopterTelemetry.instance().getDroneState();
			double yaw = droneState.yawRadRatePid.target;
			double pitch = droneState.pitchRadPid.target;
			double roll = droneState.rollRadPid.target;
			
			if( Math.abs(roll - da) > 0.1f)
			{
				roll = da;
			
				yaw = Math.toRadians(yaw);
				pitch = Math.toRadians(pitch);
				roll = Math.toRadians(roll);
			
				CmdSetYPR cmd = new CmdSetYPR((float)yaw,(float)pitch,(float)roll);
				CopterCommander.instance().addCmd(cmd);
			}
		}
	}
	
	private class OnStopKey extends AbstractAction
	{
		private static final long serialVersionUID = 1L;
		
		public OnStopKey() {}

		@Override
		public void actionPerformed(ActionEvent e)
		{
			CopterTelemetry.DroneState droneState = CopterTelemetry.instance().getDroneState();
			double yaw = droneState.yawRadRatePid.target;
			double pitch = droneState.pitchRadPid.target;
			double roll = droneState.rollRadPid.target;
			
			pitch = 0;
			roll = 0;
			
			yaw = Math.toRadians(yaw);
			pitch = Math.toRadians(pitch);
			roll = Math.toRadians(roll);
			
			CmdSetYPR cmd = new CmdSetYPR((float)yaw,(float)pitch,(float)roll);
			CopterCommander.instance().addCmd(cmd);
			CopterCommander.instance().addCmd(cmd);
			CopterCommander.instance().addCmd(cmd);
		}
	}
	
	private class YprTargetChanged implements ChangeListener
	{
		@Override
		public void stateChanged(ChangeEvent e)
		{
			JSlider sl = (JSlider)e.getSource();
			
			if(!sl.getValueIsAdjusting())
			{
				float yaw = (float)Math.toRadians(mYawPid.getTarget());
				float pitch = (float)Math.toRadians(mPitchPid.getTarget());
				float roll = (float)Math.toRadians(mRollPid.getTarget());
				CmdSetYPR cmd = new CmdSetYPR(yaw,pitch,roll);
				CopterCommander.instance().addCmd(cmd);
			}
		}
	}
	
	private class AltitudeTargetChanged implements ChangeListener
	{
		@Override
		public void stateChanged(ChangeEvent e)
		{
			JSlider sl = (JSlider)e.getSource();
			
			if(!sl.getValueIsAdjusting())
			{
				float alt = ((float)sl.getValue()) / 100.f;
				CmdSetAltitude cmd = new CmdSetAltitude(alt);
				CopterCommander.instance().addCmd(cmd);
			}			
		}	
	}
	
	private class OnTelemetryUpdate implements Observer
	{
		private void updatePidPanel(PidPanel panel, CopterTelemetry.DroneState.Pid pid)
		{
			panel.setLbEnabled(pid.enabled);
			panel.setLbKp(pid.kp);
			panel.setLbKi(pid.ki);
			panel.setLbKd(pid.kd);
		}

		@Override
		public void update(Observable o, Object arg)
		{
			CopterTelemetry.DroneState droneState = CopterTelemetry.instance().getDroneState();
			
			updatePidPanel(mYawPid, droneState.yawRadRatePid);
			updatePidPanel(mPitchPid, droneState.pitchRadPid);
			updatePidPanel(mRollPid, droneState.rollRadPid);
			updatePidPanel(mAltPid, droneState.altPid);
			
			if(mcbCollectData.isSelected())
			{
				if(mDataCount < MAX_DATA_COUNT)
				{
					mTimeData[mDataCount] = System.currentTimeMillis() - mCollectDataStartTime; 
					mYaw[mDataCount] = (float)Math.toDegrees(droneState.yawRad);
					mPitch[mDataCount] = (float)Math.toDegrees(droneState.pitchRad);
					mRoll[mDataCount] = (float)Math.toDegrees(droneState.rollRad);
					mAltitude[mDataCount] = (float)Math.toDegrees(droneState.altitude);
					mHeading[mDataCount] = (float)Math.toDegrees(droneState.headingRad);
					mYawOutput[mDataCount] = (float)Math.toDegrees(droneState.yawRadRatePid.out);
					mPitchOutput[mDataCount] = (float)Math.toDegrees(droneState.pitchRadPid.out);
					mRollOutput[mDataCount] = (float)Math.toDegrees(droneState.rollRadPid.out);
					mAltitudeOutput[mDataCount] = (float)Math.toDegrees(droneState.altPid.out);
					mGyroX[mDataCount] = droneState.gyroRad.filteredX;
					mGyroY[mDataCount] = droneState.gyroRad.filteredY;
					mGyroZ[mDataCount] = droneState.gyroRad.filteredZ;
					mAccelX[mDataCount] = droneState.accel.filteredX;
					mAccelY[mDataCount] = droneState.accel.filteredY;
					mAccelZ[mDataCount] = droneState.accel.filteredZ;
					mM0[mDataCount] = droneState.motorGas0;
					mM1[mDataCount] = droneState.motorGas1;
					mM2[mDataCount] = droneState.motorGas2;
					mM3[mDataCount] = droneState.motorGas3;
					mDataCount++;
					mlbDataCount.setText(Integer.toString(mDataCount));
				}
				else
				{
					mcbCollectData.setSelected(false);
				}
			}
		}
	}
	
	private class OnSetYawPid implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			CmdSetYawPid cmd = new CmdSetYawPid( mYawPid.getPidEnabled(),
												 mYawPid.getKp(),
												 mYawPid.getKi(),
												 mYawPid.getKd());
			
			CopterCommander.instance().addCmd(cmd);
		}
	}
	
	private class OnSetPitchPid implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			CmdSetPitchPid cmd = new CmdSetPitchPid( mPitchPid.getPidEnabled(),
												 mPitchPid.getKp(),
												 mPitchPid.getKi(),
												 mPitchPid.getKd());
			
			CopterCommander.instance().addCmd(cmd);
		}
	}
	
	private class OnSetRollPid implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			CmdSetRollPid cmd = new CmdSetRollPid( mRollPid.getPidEnabled(),
												 mRollPid.getKp(),
												 mRollPid.getKi(),
												 mRollPid.getKd());
			
			CopterCommander.instance().addCmd(cmd);
		}
	}
	
	private class OnSetAltPid implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			CmdSetAltPid cmd = new CmdSetAltPid( mAltPid.getPidEnabled(),
												 mAltPid.getKp(),
												 mAltPid.getKi(),
												 mAltPid.getKd());
			
			CopterCommander.instance().addCmd(cmd);
		}
	}
	
	private class OnCollectData implements ItemListener
	{
		@Override
		public void itemStateChanged(ItemEvent e)
		{
			if(e.getStateChange() == ItemEvent.SELECTED)
			{
				mDataCount = 0;
				mCollectDataStartTime = System.currentTimeMillis();
			}
			else
			{
				PIDlg.this.drawData();
			}
		}
	}
	
	private class OnDrawData implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			PIDlg.this.drawData();
		}
	}
	
	private OnTelemetryUpdate mObserver;
	
	private JCheckBox mcbMotorsEnabled;
	private MotorGasSlider mSetAllGas;
	private JCheckBox mcbCollectData;
	private JLabel mlbDataCount;
	private JCheckBox mcbDrawYaw;
	private JCheckBox mcbDrawPitch;
	private JCheckBox mcbDrawRoll;
	private JCheckBox mcbDrawHeading;
	private JCheckBox mcbDrawAlt;
	private JCheckBox mcbDrawYawOutput;
	private JCheckBox mcbDrawPitchOutput;
	private JCheckBox mcbDrawRollOutput;
	private JCheckBox mcbDrawAltOutput;
	private JCheckBox mcbDrawGyroX;
	private JCheckBox mcbDrawGyroY;
	private JCheckBox mcbDrawGyroZ;
	private JCheckBox mcbDrawAccelX;
	private JCheckBox mcbDrawAccelY;
	private JCheckBox mcbDrawAccelZ;
	private JCheckBox mcbDrawM0;
	private JCheckBox mcbDrawM1;
	private JCheckBox mcbDrawM2;
	private JCheckBox mcbDrawM3;
	private PidPanel mYawPid;
	private PidPanel mPitchPid;
	private PidPanel mRollPid;
	private PidPanel mAltPid;
	private ChartViewer mChartViewer;
	private static int MAX_DATA_COUNT = 1024;
	private int mDataCount;
	private double mYaw[];
	private double mPitch[];
	private double mRoll[];
	private double mAltitude[];
	private double mHeading[];
	private double mYawOutput[];
	private double mPitchOutput[];
	private double mRollOutput[];
	private double mAltitudeOutput[];
	private double mTimeData[];
	private double mGyroX[];
	private double mGyroY[];
	private double mGyroZ[];
	private double mAccelX[];
	private double mAccelY[];
	private double mAccelZ[];
	private double mM0[];
	private double mM1[];
	private double mM2[];
	private double mM3[];
	private long mCollectDataStartTime;

	public PIDlg(JFrame owner)
	{
		super(owner,true);
		
		mYaw = new double[MAX_DATA_COUNT];
		mPitch = new double[MAX_DATA_COUNT];
		mRoll = new double[MAX_DATA_COUNT];
		mHeading = new double[MAX_DATA_COUNT];
		mYawOutput = new double[MAX_DATA_COUNT];
		mPitchOutput = new double[MAX_DATA_COUNT];
		mRollOutput = new double[MAX_DATA_COUNT];
		mAltitude = new double[MAX_DATA_COUNT];
		mAltitudeOutput = new double[MAX_DATA_COUNT];
		mGyroX = new double[MAX_DATA_COUNT];
		mGyroY = new double[MAX_DATA_COUNT];
		mGyroZ = new double[MAX_DATA_COUNT];
		mAccelX = new double[MAX_DATA_COUNT];
		mAccelY = new double[MAX_DATA_COUNT];
		mAccelZ = new double[MAX_DATA_COUNT];
		mM0 = new double[MAX_DATA_COUNT];
		mM1 = new double[MAX_DATA_COUNT];
		mM2 = new double[MAX_DATA_COUNT];
		mM3 = new double[MAX_DATA_COUNT];
		mTimeData = new double[MAX_DATA_COUNT];
		
		mObserver = new OnTelemetryUpdate();
		
		this.setResizable(true);
		this.setSize(1060,660);
		this.setLocationRelativeTo(null);
		this.setTitle(Text.get("PID"));
		this.setLayout(new MigLayout("","[]10[]","[center]"));
		
		DroneState droneState = CopterTelemetry.instance().getDroneState();
		
		mcbMotorsEnabled = new JCheckBox(Text.get("MOTORS_ENABLED") + "(Q)");
		mcbMotorsEnabled.setSelected(droneState.motorsEnabled);
		//mcbMotorsEnabled.addActionListener(new CopterCtrlPanel.OnMotorsEnabled());
		mcbMotorsEnabled.setMnemonic('Q');
		
		mSetAllGas = new MotorGasSlider(Text.get("GAS"),true);
		mYawPid = new PidPanel(Text.get("YAW_PID"),-180,180);
		mPitchPid = new PidPanel(Text.get("PITCH_PID"),-180,180);
		mRollPid = new PidPanel(Text.get("ROLL_PID"),-180,180);
		mAltPid = new PidPanel(Text.get("ALT_PID"),0,200);
		
		mSetAllGas.addChangeListener(new GasChanged());
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("Z"),"addGas");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("X"),"subGas");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("typed c"),"takeOff");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("LEFT"),"turnLeft");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("released LEFT"),"stopLeft");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("RIGHT"),"turnRight");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("released RIGHT"),"stopRight");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("UP"),"fwd");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("released UP"),"stopFwd");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("DOWN"),"back");
		mcbMotorsEnabled.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke("released DOWN"),"stopBack");
		mcbMotorsEnabled.getActionMap().put("addGas", new OnAddGasKey(30));
		mcbMotorsEnabled.getActionMap().put("subGas", new OnAddGasKey(-30));
		mcbMotorsEnabled.getActionMap().put("takeOff", new OnTakeOffKey());
		mcbMotorsEnabled.getActionMap().put("turnLeft", new OnRollKey(-8));
		mcbMotorsEnabled.getActionMap().put("turnRight", new OnRollKey(8));
		mcbMotorsEnabled.getActionMap().put("fwd", new OnPitchKey(8));
		mcbMotorsEnabled.getActionMap().put("back", new OnPitchKey(-8));
		mcbMotorsEnabled.getActionMap().put("stopFwd", new OnStopKey());
		mcbMotorsEnabled.getActionMap().put("stopBack", new OnStopKey());
		mcbMotorsEnabled.getActionMap().put("stopLeft", new OnStopKey());
		mcbMotorsEnabled.getActionMap().put("stopRight", new OnStopKey());
		
		mYawPid.onSet(new OnSetYawPid());
		mYawPid.onTarget(new YprTargetChanged());
		mPitchPid.onSet(new OnSetPitchPid());
		mPitchPid.onTarget(new YprTargetChanged());
		mRollPid.onSet(new OnSetRollPid());
		mRollPid.onTarget(new YprTargetChanged());
		mAltPid.onSet(new OnSetAltPid());
		mAltPid.onTarget(new AltitudeTargetChanged());
		
		mcbCollectData = new JCheckBox(Text.get("COLLECT_DATA"));
		mcbCollectData.addItemListener(new OnCollectData());
		mlbDataCount = new JLabel("0");
		
		mcbDrawYaw = new JCheckBox(Text.get("YAW"));
		mcbDrawPitch = new JCheckBox(Text.get("PITCH"));
		mcbDrawRoll = new JCheckBox(Text.get("ROLL"));
		mcbDrawHeading = new JCheckBox(Text.get("HEADING"));
		mcbDrawYawOutput = new JCheckBox(Text.get("YAW_OUTPUT"));
		mcbDrawPitchOutput = new JCheckBox(Text.get("PITCH_OUTPUT"));
		mcbDrawRollOutput = new JCheckBox(Text.get("ROLL_OUTPUT"));
		mcbDrawAlt = new JCheckBox(Text.get("ALTITUDE"));
		mcbDrawAltOutput = new JCheckBox(Text.get("ALT_OUTPUT"));
		mcbDrawGyroX = new JCheckBox("GyroX");
		mcbDrawGyroY = new JCheckBox("GyroY");
		mcbDrawGyroZ = new JCheckBox("GyroZ");
		mcbDrawAccelX = new JCheckBox("AccelX");
		mcbDrawAccelY = new JCheckBox("AccelY");
		mcbDrawAccelZ = new JCheckBox("AccelZ");
		mcbDrawM0 = new JCheckBox("M1");
		mcbDrawM1 = new JCheckBox("M2");
		mcbDrawM2 = new JCheckBox("M3");
		mcbDrawM3 = new JCheckBox("M4");
		
		mcbDrawYaw.addActionListener(new OnDrawData());
		mcbDrawPitch.addActionListener(new OnDrawData());
		mcbDrawRoll.addActionListener(new OnDrawData());
		mcbDrawHeading.addActionListener(new OnDrawData());
		mcbDrawYawOutput.addActionListener(new OnDrawData());
		mcbDrawPitchOutput.addActionListener(new OnDrawData());
		mcbDrawRollOutput.addActionListener(new OnDrawData());
		mcbDrawAlt.addActionListener(new OnDrawData());
		mcbDrawAltOutput.addActionListener(new OnDrawData());
		mcbDrawGyroX.addActionListener(new OnDrawData());
		mcbDrawGyroY.addActionListener(new OnDrawData());
		mcbDrawGyroZ.addActionListener(new OnDrawData());
		mcbDrawAccelX.addActionListener(new OnDrawData());
		mcbDrawAccelY.addActionListener(new OnDrawData());
		mcbDrawAccelZ.addActionListener(new OnDrawData());
		mcbDrawM0.addActionListener(new OnDrawData());
		mcbDrawM1.addActionListener(new OnDrawData());
		mcbDrawM2.addActionListener(new OnDrawData());
		mcbDrawM3.addActionListener(new OnDrawData());
		
		mChartViewer = new ChartViewer();
		
		JPanel pnlDraw = new JPanel(new MigLayout());
		pnlDraw.setBorder(new TitledBorder(Text.get("PLOT")));
		pnlDraw.add(mcbDrawYaw);
		pnlDraw.add(mcbDrawPitch);
		pnlDraw.add(mcbDrawRoll);
		pnlDraw.add(mcbDrawAlt);
		pnlDraw.add(mcbDrawGyroX);
		pnlDraw.add(mcbDrawGyroY);
		pnlDraw.add(mcbDrawGyroZ);
		pnlDraw.add(mcbDrawHeading,"wrap");
		pnlDraw.add(mcbDrawYawOutput);
		pnlDraw.add(mcbDrawPitchOutput);
		pnlDraw.add(mcbDrawRollOutput);
		pnlDraw.add(mcbDrawAltOutput);
		pnlDraw.add(mcbDrawM0);
		pnlDraw.add(mcbDrawM1);
		pnlDraw.add(mcbDrawM2);
		pnlDraw.add(mcbDrawM3,"wrap");
		pnlDraw.add(mcbDrawAccelX);
		pnlDraw.add(mcbDrawAccelY);
		pnlDraw.add(mcbDrawAccelZ);
		
		this.add(mcbMotorsEnabled);
		this.add(mSetAllGas,"spanx 3,growx");
		this.add(mChartViewer,"spany 7,grow,wrap");
		this.add(mcbCollectData);
		this.add(new JLabel(Text.get("DATA_COUNT") + ":"));
		this.add(mlbDataCount);
		this.add(new JLabel(Text.get("TELEMETRY") + ":  " + CopterTelemetry.instance().getTelemetryPeriod() + "ms"),"grow,wrap");
		this.add(pnlDraw,"spanx 4,grow,wrap");
		this.add(mYawPid,"spanx 4,grow,wrap");
		this.add(mPitchPid,"spanx 4,grow,wrap");
		this.add(mRollPid,"spanx 4,grow,wrap");
		this.add(mAltPid,"spanx 4,grow,wrap");
		
		mYawPid.initBySettings( Settings.instance().getYawPidEnabled(),
								Settings.instance().getYawPidKp(),
								Settings.instance().getYawPidKi(),
								Settings.instance().getYawPidKd());
		
		mPitchPid.initBySettings(	Settings.instance().getPitchPidEnabled(),
									Settings.instance().getPitchPidKp(),
									Settings.instance().getPitchPidKi(),
									Settings.instance().getPitchPidKd());
		
		mRollPid.initBySettings(	Settings.instance().getRollPidEnabled(),
									Settings.instance().getRollPidKp(),
									Settings.instance().getRollPidKi(),
									Settings.instance().getRollPidKd());
		
		mAltPid.initBySettings(	Settings.instance().getAltPidEnabled(),
								Settings.instance().getAltPidKp(),
								Settings.instance().getAltPidKi(),
								Settings.instance().getAltPidKd());
		
		this.drawData();
	}
	
	private void drawData()
	{
        // Create a XYChart object of size 250 x 250 pixels
        XYChart c = new XYChart(500, 600);
        // Set the plotarea at (30, 20) and of size 200 x 200 pixels
        c.setPlotArea(60, 45, 410, 510, -1, -1, 0xc0c0c0, 0xc0c0c0, -1);
        c.addLegend(60, 5, false);
        // Set scale for axis
        c.xAxis().setTitle("Step");
        c.xAxis().setMargin(0,0);
        
        c.yAxis().setTitle("YPR");
        c.yAxis().setMargin(0,0);

       	//c.yAxis().setLinearScale(-180, 180, 10);
        
       	int yawColor = 0x800000;
       	int pitchColor = 0x008000;
       	int rollColor = 0x000080;
       	int headingColor = 0x00FFFF;
       	int altColor = 0x7FFFD4;
       	
       	if(mDataCount > 0)
       	{
       		DroneState droneState = CopterTelemetry.instance().getDroneState();
       		double yawTarget = Math.toDegrees(droneState.yawRadRatePid.target);
       		double pitchTarget = Math.toDegrees(droneState.pitchRadPid.target);
       		double rollTarget = Math.toDegrees(droneState.rollRadPid.target);
       		double altTarget = droneState.altPid.target;
       		
       		if(mcbDrawYaw.isSelected())
       		{
       			c.yAxis().addMark(yawTarget, yawColor, "Yaw").setLineWidth(2);
       			c.addLineLayer(Arrays.copyOf(mYaw, mDataCount), yawColor, "Yaw").setXData(mTimeData);
       		}
       		
       		if(mcbDrawPitch.isSelected())
       		{
       			c.yAxis().addMark(pitchTarget, pitchColor, "Pitch").setLineWidth(2);
       			c.addLineLayer(Arrays.copyOf(mPitch, mDataCount), pitchColor, "Pitch").setXData(mTimeData);
       		}
       		
       		if(mcbDrawRoll.isSelected())
       		{
       			c.yAxis().addMark(rollTarget, rollColor, "Roll").setLineWidth(2);
       			c.addLineLayer(Arrays.copyOf(mRoll, mDataCount), rollColor, "Roll").setXData(mTimeData);
       		}
       		
       		if(mcbDrawHeading.isSelected())
       		{
       			c.yAxis().addMark(yawTarget, yawColor, "Yaw").setLineWidth(2);
       			c.addLineLayer(Arrays.copyOf(mHeading, mDataCount), headingColor, "Heading").setXData(mTimeData);
       		}
       		
       		if(mcbDrawYawOutput.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mYawOutput, mDataCount), -1, "Y-out").setXData(mTimeData);
       		}
       		
       		if(mcbDrawPitchOutput.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mPitchOutput, mDataCount), -1, "P-out").setXData(mTimeData);
       		}
       		
       		if(mcbDrawRollOutput.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mRollOutput, mDataCount), -1, "R-out").setXData(mTimeData);
       		}
       		
       		if(mcbDrawAlt.isSelected())
       		{
       			c.yAxis().addMark(altTarget, altColor, "Alt").setLineWidth(2);
       			c.addLineLayer(Arrays.copyOf(mAltitude, mDataCount), altColor, "Alt").setXData(mTimeData);
       		}
       		
       		if(mcbDrawAltOutput.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mAltitudeOutput, mDataCount), -1, "A-out").setXData(mTimeData);
       		}
       		
       		if(mcbDrawGyroX.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mGyroX, mDataCount), -1, "GyroX").setXData(mTimeData);
       		}
       		
       		if(mcbDrawGyroY.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mGyroY, mDataCount), -1, "GyroY").setXData(mTimeData);
       		}
       		
       		if(mcbDrawGyroZ.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mGyroZ, mDataCount), -1, "GyroZ").setXData(mTimeData);
       		}
       		
       		if(mcbDrawAccelX.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mAccelX, mDataCount), -1, "AccelX").setXData(mTimeData);
       		}
       		
       		if(mcbDrawAccelY.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mAccelY, mDataCount), -1, "AccelY").setXData(mTimeData);
       		}
       		
       		if(mcbDrawAccelZ.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mAccelZ, mDataCount), -1, "AccelZ").setXData(mTimeData);
       		}
       		
       		if(mcbDrawM0.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mM0, mDataCount), -1, "M0").setXData(mTimeData);
       		}
       		
       		if(mcbDrawM1.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mM1, mDataCount), -1, "M1").setXData(mTimeData);
       		}
       		
       		if(mcbDrawM2.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mM2, mDataCount), -1, "M2").setXData(mTimeData);
       		}
       		
       		if(mcbDrawM3.isSelected())
       		{
       			c.addLineLayer(Arrays.copyOf(mM3, mDataCount), -1, "M3").setXData(mTimeData);
       		}

           	//c.xAxis().setAutoScale();//(0,0);
       		c.xAxis().setLinearScale(0, mTimeData[mDataCount - 1], 500);
           	c.yAxis().setAutoScale();//(0,0);
       	}

        mChartViewer.setChart(c);
	}
	
	@Override
	public void setVisible(boolean b)
	{
		if(b)
		{
			CopterTelemetry.instance().addObserver(mObserver);
		}
		else
		{
			int result = JOptionPane.showConfirmDialog(	this,
														Text.get("CONFIRM_SAVE_SETTINGS"),
														"",
														JOptionPane.YES_NO_OPTION,
														JOptionPane.QUESTION_MESSAGE);
		
			if(result == JOptionPane.YES_OPTION)
			{
				Settings.instance().setYawPidEnabled(mYawPid.getPidEnabled());
				Settings.instance().setYawPidKp(mYawPid.getKp());
				Settings.instance().setYawPidKi(mYawPid.getKi());
				Settings.instance().setYawPidKd(mYawPid.getKd());
				
				Settings.instance().setPitchPidEnabled(mPitchPid.getPidEnabled());
				Settings.instance().setPitchPidKp(mPitchPid.getKp());
				Settings.instance().setPitchPidKi(mPitchPid.getKi());
				Settings.instance().setPitchPidKd(mPitchPid.getKd());
				
				Settings.instance().setRollPidEnabled(mRollPid.getPidEnabled());
				Settings.instance().setRollPidKp(mRollPid.getKp());
				Settings.instance().setRollPidKi(mRollPid.getKi());
				Settings.instance().setRollPidKd(mRollPid.getKd());
				
				Settings.instance().setAltPidEnabled(mAltPid.getPidEnabled());
				Settings.instance().setAltPidKp(mAltPid.getKp());
				Settings.instance().setAltPidKi(mAltPid.getKi());
				Settings.instance().setAltPidKd(mAltPid.getKd());
				
				Settings.instance().save();
			}
			
			CopterTelemetry.instance().deleteObserver(mObserver);
		}
		
		super.setVisible(b);
	}
}
