package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JButton;
import javax.swing.JCheckBox;
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

import ChartDirector.ChartViewer;
import ChartDirector.XYChart;
import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.commands.CmdSetAltPid;
import copter.commands.CmdSetMotorsGas;
import copter.commands.CmdSetPitchPid;
import copter.commands.CmdSetRollPid;
import copter.commands.CmdSetYPR;
import copter.commands.CmdSetYawPid;
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
		
		public PidPanel(String title)
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
			mslTarget = new JSlider(JSlider.HORIZONTAL, -180, 180, 0);
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
		
		public void setLbKp(float kp)
		{
			mlbKp.setText(mKoefFmt.format(kp));
		}
		
		public float getKp()
		{
			return Float.parseFloat(mtfKp.getText());
		}
		
		public void setLbKi(float ki)
		{
			mlbKi.setText(mKoefFmt.format(ki));
		}
		
		public float getKi()
		{
			return Float.parseFloat(mtfKi.getText());
		}
		
		public void setLbKd(float kd)
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
				CopterCommander.instance().addCmd(new CmdSetMotorsGas(gas,gas,gas,gas));
			}
		}
	}
	
	private class TargetChanged implements ChangeListener
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
	
	private class OnTelemetryUpdate implements Observer
	{

		@Override
		public void update(Observable o, Object arg)
		{
			mYawPid.setLbEnabled(CopterTelemetry.instance().getYawPidEnabled());
			mYawPid.setLbKp(CopterTelemetry.instance().getYawPidKp());
			mYawPid.setLbKi(CopterTelemetry.instance().getYawPidKi());
			mYawPid.setLbKd(CopterTelemetry.instance().getYawPidKd());
			
			mPitchPid.setLbEnabled(CopterTelemetry.instance().getPitchPidEnabled());
			mPitchPid.setLbKp(CopterTelemetry.instance().getPitchPidKp());
			mPitchPid.setLbKi(CopterTelemetry.instance().getPitchPidKi());
			mPitchPid.setLbKd(CopterTelemetry.instance().getPitchPidKd());
			
			mRollPid.setLbEnabled(CopterTelemetry.instance().getRollPidEnabled());
			mRollPid.setLbKp(CopterTelemetry.instance().getRollPidKp());
			mRollPid.setLbKi(CopterTelemetry.instance().getRollPidKi());
			mRollPid.setLbKd(CopterTelemetry.instance().getRollPidKd());
			
			mAltPid.setLbEnabled(CopterTelemetry.instance().getAltPidEnabled());
			mAltPid.setLbKp(CopterTelemetry.instance().getAltPidKp());
			mAltPid.setLbKi(CopterTelemetry.instance().getAltPidKi());
			mAltPid.setLbKd(CopterTelemetry.instance().getAltPidKd());
			
			if(mcbCollectData.isSelected())
			{
				if(mDataCount < MAX_DATA_COUNT)
				{
					mTimeData[mDataCount] = System.currentTimeMillis() - mCollectDataStartTime; 
					mYaw[mDataCount] = CopterTelemetry.instance().getYaw();
					mPitch[mDataCount] = CopterTelemetry.instance().getPitch();
					mRoll[mDataCount] = CopterTelemetry.instance().getRoll();
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
	
	private OnTelemetryUpdate mObserver;
	
	private JCheckBox mcbMotorsEnabled;
	private MotorGasSlider mSetAllGas;
	private JCheckBox mcbCollectData;
	private JLabel mlbDataCount;
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
	private double mTimeData[];
	private long mCollectDataStartTime;

	public PIDlg(JFrame owner)
	{
		super(owner,true);
		
		mYaw = new double[MAX_DATA_COUNT];
		mPitch = new double[MAX_DATA_COUNT];
		mRoll = new double[MAX_DATA_COUNT];
		mTimeData = new double[MAX_DATA_COUNT];
		
		mObserver = new OnTelemetryUpdate();
		
		this.setResizable(true);
		this.setSize(975,560);
		this.setLocationRelativeTo(null);
		this.setTitle(Text.get("PID"));
		this.setLayout(new MigLayout("","[]10[]","[center]"));
		
		mcbMotorsEnabled = new JCheckBox(Text.get("MOTORS_ENABLED") + "(Q)");
		mcbMotorsEnabled.setSelected(CopterTelemetry.instance().getMotorsEnabled());
		mcbMotorsEnabled.addActionListener(new CopterCtrlPanel.OnMotorsEnabled());
		mcbMotorsEnabled.setMnemonic('Q');
		
		mSetAllGas = new MotorGasSlider(Text.get("GAS"),true);
		mYawPid = new PidPanel(Text.get("YAW_PID"));
		mPitchPid = new PidPanel(Text.get("PITCH_PID"));
		mRollPid = new PidPanel(Text.get("ROLL_PID"));
		mAltPid = new PidPanel(Text.get("ALT_PID"));
		
		mSetAllGas.addChangeListener(new GasChanged());
		mYawPid.onSet(new OnSetYawPid());
		mYawPid.onTarget(new TargetChanged());
		mPitchPid.onSet(new OnSetPitchPid());
		mPitchPid.onTarget(new TargetChanged());
		mRollPid.onSet(new OnSetRollPid());
		mRollPid.onTarget(new TargetChanged());
		mAltPid.onSet(new OnSetAltPid());
		
		mcbCollectData = new JCheckBox(Text.get("COLLECT_DATA"));
		mcbCollectData.addItemListener(new OnCollectData());
		mlbDataCount = new JLabel("0");
		
		mChartViewer = new ChartViewer();
		
		this.add(mcbMotorsEnabled);
		this.add(mSetAllGas,"spanx 3,growx");
		this.add(mChartViewer,"spany 6,grow,wrap");
		this.add(mcbCollectData);
		this.add(new JLabel(Text.get("DATA_COUNT") + ":"));
		this.add(mlbDataCount);
		this.add(new JLabel(Text.get("TELEMETRY") + ":  " + CopterTelemetry.instance().getTelemetryPeriod() + "ms " + 
				Text.get("PID") + ":  " + CopterTelemetry.instance().getPidPeriod()  + "ms"),"spanx,wrap");
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
        XYChart c = new XYChart(500, 500);
        // Set the plotarea at (30, 20) and of size 200 x 200 pixels
        c.setPlotArea(60, 45, 410, 410, -1, -1, 0xc0c0c0, 0xc0c0c0, -1);
        c.addLegend(60, 5, false);
        // Set scale for axis
        c.xAxis().setTitle("Step");
        c.xAxis().setMargin(0,0);
        
        c.yAxis().setTitle("YPR");
        c.yAxis().setMargin(0,0);

       	c.yAxis().setLinearScale(-180, 180, 10);
       	
       	int yawColor = 0x800000;
       	int pitchColor = 0x008000;
       	int rollColor = 0x000080;
       	
       	if(mDataCount > 0)
       	{
       		float yawTarget = CopterTelemetry.instance().getYawPidTarget();
       		float pitchTarget = CopterTelemetry.instance().getPitchPidTarget();
       		float rollTarget = CopterTelemetry.instance().getRollPidTarget();
       		
       		c.yAxis().addMark(yawTarget, yawColor, "Yaw").setLineWidth(2);
       		c.yAxis().addMark(pitchTarget, pitchColor, "Pitch").setLineWidth(2);
       		c.yAxis().addMark(rollTarget, rollColor, "Roll").setLineWidth(2);
       	
       		c.addLineLayer(Arrays.copyOf(mYaw, mDataCount), yawColor, "Yaw").setXData(mTimeData);
       		c.addLineLayer(Arrays.copyOf(mPitch, mDataCount), pitchColor, "Pitch").setXData(mTimeData);
       		c.addLineLayer(Arrays.copyOf(mRoll, mDataCount), rollColor, "Roll").setXData(mTimeData);

           	c.xAxis().setAutoScale();
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
