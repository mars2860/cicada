package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentAdapter;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.text.DecimalFormat;
import java.util.Collections;
import java.util.List;
import java.util.Observable;
import java.util.Observer;

import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;

import ChartDirector.Chart;
import ChartDirector.ChartViewer;
import ChartDirector.XYChart;
import helper.ArrayHelper;
import main.Settings.WndState;
import net.miginfocom.swing.MigLayout;
import pdl.DroneCommander;
import pdl.DroneTelemetry;
import pdl.DroneState;

import pdl.commands.CmdSetAccel;
import pdl.commands.CmdSetGyro;
import pdl.commands.CmdSetMagneto;

public class SensorsGui extends JSavedFrame
{
	private static final long serialVersionUID = -4310119839211305793L;

	private class OnTelemetryUpdate implements Observer
	{
		private long timestamp;
		
		@Override
		public void update(Observable o, Object arg)
		{
			// update data in this gui no faster 50 ms
			if(System.currentTimeMillis() - timestamp < 50)
				return;
			timestamp = System.currentTimeMillis();
			
			String text;
			DecimalFormat fmt = new DecimalFormat();
			fmt.setMaximumFractionDigits(2);
			
			DroneState droneState = DroneTelemetry.instance().getDroneState();
			
			text = ": " + Integer.toString((int)droneState.accel.rawX) + "/" +
							Integer.toString((int)droneState.accel.offsetX);
			mlbAx.setText(text);
			mlbAx.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.accel.rawY) + "/" +
					Integer.toString((int)droneState.accel.offsetY);
			mlbAy.setText(text);
			mlbAy.setToolTipText(text);
	
			text = ": " + Integer.toString((int)droneState.accel.rawZ) + "/" +
					Integer.toString((int)droneState.accel.offsetZ);
			mlbAz.setText(text);
			mlbAz.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.gyro.rawX) + "/" +
					Integer.toString((int)droneState.gyro.offsetX);
			mlbGx.setText(text);
			mlbGx.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.gyro.rawY) + "/" +
					Integer.toString((int)droneState.gyro.offsetY);
			mlbGy.setText(text);
			mlbGy.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.gyro.rawZ) + "/" +
					Integer.toString((int)droneState.gyro.offsetZ);
			mlbGz.setText(text);
			mlbGz.setToolTipText(text);
			
			/* debug compass tilt compensation
			double roll = CopterTelemetry.instance().getRoll();
			double pitch = CopterTelemetry.instance().getPitch();
			double fx = CopterTelemetry.instance().getMagnetX();
			double fy = CopterTelemetry.instance().getMagnetY();
			double fz = CopterTelemetry.instance().getMagnetZ();
			double cor = Math.cos(Math.toRadians(roll));
			double sir = Math.sin(Math.toRadians(roll));
			double cop = Math.cos(Math.toRadians(pitch));
			double sip = Math.sin(Math.toRadians(pitch));
			
			DecimalFormat fmt1 = new DecimalFormat();
			fmt1.setMaximumFractionDigits(0);
			
			double fx1 = fx*cop + fy*sir*sip + fz*cor*sip;
			double fy1 = fy*cor - fz*sir;
			double fz1 = -fx*sip + fy*cop*sir + fz*cor*cop;
			
			double heading = 180.0 - Math.toDegrees(Math.atan2(fy1, fx1));
			
			mlbDataCount.setText(fmt1.format(heading));
			*/
			
			text = ": " + Integer.toString((int)droneState.magneto.rawX) + "/" +
					Integer.toString((int)droneState.magneto.offsetX);
			mlbMx.setText(text);
			mlbMx.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.magneto.rawY) + "/" +
					Integer.toString((int)droneState.magneto.offsetY);
			mlbMy.setText(text);
			mlbMy.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.magneto.rawZ) + "/" +
					Integer.toString((int)droneState.magneto.offsetZ);
			mlbMz.setText(text);
			mlbMz.setToolTipText(text);
			
			if(mcbCollectData.isSelected())
			{
				if(mDataCount < MAX_DATA_COUNT)
				{
					mAccelRawX[mDataCount] = droneState.accel.rawX;
					mAccelRawY[mDataCount] = droneState.accel.rawY;
					mAccelRawZ[mDataCount] = droneState.accel.rawZ;
					
					mGyroRawX[mDataCount] = droneState.gyro.rawX;
					mGyroRawY[mDataCount] = droneState.gyro.rawY;
					mGyroRawZ[mDataCount] = droneState.gyro.rawZ;
				
					mMagnetRawX[mDataCount] = droneState.magneto.rawX;
					mMagnetRawY[mDataCount] = droneState.magneto.rawY;
					mMagnetRawZ[mDataCount] = droneState.magneto.rawZ;
					
					mMagnetPureX[mDataCount] = droneState.magneto.pureX;
					mMagnetPureY[mDataCount] = droneState.magneto.pureY;
					mMagnetPureZ[mDataCount] = droneState.magneto.pureZ;
				}
				
				mDataCount++;
				if(mDataCount >= MAX_DATA_COUNT)
					mcbCollectData.setSelected(false);
			}
			
			text = ": " + Integer.toString(mDataCount);
			mlbDataCount.setText(text);
		}
	}
	
	private class OnChartResize extends ComponentAdapter
	{
		public void componentResized(java.awt.event.ComponentEvent e)
		{
			SensorsGui.this.drawData();
		};
	}
	
	private class OnCollectData implements ItemListener
	{
		@Override
		public void itemStateChanged(ItemEvent e)
		{
			if(mcbCollectData.isSelected())
			{
				mDataCount = 0;
			}
			else
			{
				SensorsGui.this.drawData();
			}	
		}
	}
	
	private class OnChangeShowedData implements ItemListener
	{
		@Override
		public void itemStateChanged(ItemEvent e)
		{
			if(e.getStateChange() == ItemEvent.SELECTED)
				SensorsGui.this.drawData();	
		}
	}
		
	private class OnBtnCalibrateMagnet implements ActionListener
	{

		@Override
		public void actionPerformed(ActionEvent e)
		{
			// https://appelsiini.net/2018/calibrate-magnetometer/
			
			/*DoubleSummaryStatistics dss = Arrays.stream(mMagnetX).summaryStatistics();
			double maxMx = dss.getMax();
			double minMx = dss.getMin();
			
			dss = Arrays.stream(mMagnetY).summaryStatistics();
			double maxMy = dss.getMax();
			double minMy = dss.getMin();
			
			dss = Arrays.stream(mMagnetZ).summaryStatistics();
			double maxMz = dss.getMax();
			double minMz = dss.getMin();
			
			double avgDeltaX = (maxMx - minMx)/2.0;
			double avgDeltaY = (maxMy - minMx)/2.0;
			double avgDeltaZ = (maxMz - minMz)/2.0;
			
			double avgDelta = (avgDeltaX + avgDeltaY + avgDeltaZ)/3.0;
			
			double sx = avgDelta/avgDeltaX;
			double sy = avgDelta/avgDeltaY;
			double sz = avgDelta/avgDeltaZ;
			
			double dx = (maxMx + minMx)/2.0;
			double dy = (maxMy + minMy)/2.0;
			double dz = (maxMz + minMz)/2.0;
			
			if(dx > 0)
				dx += 0.5;
			else
				dx -= 0.5;
			
			if(dy > 0)
				dy += 0.5;
			else
				dy -= 0.5;
			
			if(dz > 0)
				dz += 0.5;
			else
				dz -= 0.5;
			
			CopterCommander.instance().addCmd(
					new CmdCalibrateMagnet( (int)(dx),
											(int)(dy),
											(int)(dz),
											(float)sx,
											(float)sy,
											(float)sz ));
			 */
			
			// AN2272
			// This work only on Java 8
			/*
			DoubleSummaryStatistics dss = Arrays.stream(mMagnetX).summaryStatistics();
			double maxMx = dss.getMax();
			double minMx = dss.getMin();
			
			dss = Arrays.stream(mMagnetY).summaryStatistics();
			double maxMy = dss.getMax();
			double minMy = dss.getMin();
			
			dss = Arrays.stream(mMagnetZ).summaryStatistics();
			double maxMz = dss.getMax();
			double minMz = dss.getMin();
			*/
						
			DroneState.TripleAxisSensor sens = calcSensOffsets(mMagnetRawX,mMagnetRawY,mMagnetRawZ);

			DroneCommander.instance().sendSetupCmd(new CmdSetMagneto(sens,1.f,1.f,1.f));
			
			mModified = true;
		}
	}
	
	private class OnBtnResetCalibration implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			DroneState.TripleAxisSensor sens = new DroneState.TripleAxisSensor();
			DroneCommander.instance().addCmd(new CmdSetAccel(sens));
			DroneCommander.instance().addCmd(new CmdSetGyro(sens));
			DroneCommander.instance().addCmd(new CmdSetMagneto(sens,1.f,1.f,1.f));
			
			mModified = true;
		}
	}
	
	private class OnBtnResetMagnetCalibration implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			DroneState.TripleAxisSensor sens = new DroneState.TripleAxisSensor();
			DroneCommander.instance().addCmd(new CmdSetMagneto(sens,1.f,1.f,1.f));
			
			mModified = true;
		}
	}
	
	private class OnBtnCalibrateAccel implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			//DroneCommander.instance().addCmd(new CmdSelfCalibrateAccel());
			//JOptionPane.showMessageDialog(SensorsGui.this, Text.get("WAIT_CALIBRATION"),"",JOptionPane.INFORMATION_MESSAGE);

			DroneState.TripleAxisSensor sens = calcSensOffsets(mAccelRawX,mAccelRawY,mAccelRawZ);
			
			DroneCommander.instance().sendSetupCmd(new CmdSetAccel(sens));
			
			mModified = true;
		}
	}
	
	private class OnBtnCalibrateGyro implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			//DroneCommander.instance().addCmd(new CmdSelfCalibrateGyro());
			//JOptionPane.showMessageDialog(SensorsGui.this, Text.get("WAIT_CALIBRATION"),"",JOptionPane.INFORMATION_MESSAGE);
			
			DroneState.TripleAxisSensor sens = calcSensOffsets(mGyroRawX,mGyroRawY,mGyroRawZ);
			
			DroneCommander.instance().sendSetupCmd(new CmdSetGyro(sens));
			
			mModified = true;
		}
	}
	
	private OnTelemetryUpdate mObserver;
	
	private JLabel mlbAx;
	private JLabel mlbAy;
	private JLabel mlbAz;
	private JLabel mlbGx;
	private JLabel mlbGy;
	private JLabel mlbGz;
	private JLabel mlbMx;
	private JLabel mlbMy;
	private JLabel mlbMz;
	private ChartViewer mChartViewer;
	private JCheckBox mcbCollectData;
	private JLabel mlbDataCount;
	private JRadioButton mrbMagnetRaw;
	private JRadioButton mrbMagnetPure;
	
	private static final int MAX_DATA_COUNT = 1024;
	
	private int mDataCount;
	private double mAccelRawX[];
	private double mAccelRawY[];
	private double mAccelRawZ[];
	private double mGyroRawX[];
	private double mGyroRawY[];
	private double mGyroRawZ[];
	private double mMagnetRawX[];
	private double mMagnetRawY[];
	private double mMagnetRawZ[];
	private double mMagnetPureX[];
	private double mMagnetPureY[];
	private double mMagnetPureZ[];
	
	private boolean mModified = false;
	
	public SensorsGui()
	{
		super("Sensors",680,568);
		
		this.setTitle(ResBox.text("SENSORS"));
		this.setIconImage(ResBox.icon("SENSORS").getImage());
		
		mObserver = new OnTelemetryUpdate();
		mAccelRawX = new double[MAX_DATA_COUNT];
		mAccelRawY = new double[MAX_DATA_COUNT];
		mAccelRawZ = new double[MAX_DATA_COUNT];
		mGyroRawX = new double[MAX_DATA_COUNT];
		mGyroRawY = new double[MAX_DATA_COUNT];
		mGyroRawZ = new double[MAX_DATA_COUNT];
		mMagnetRawX = new double[MAX_DATA_COUNT];
		mMagnetRawY = new double[MAX_DATA_COUNT];
		mMagnetRawZ = new double[MAX_DATA_COUNT];
		mMagnetPureX = new double[MAX_DATA_COUNT];
		mMagnetPureY = new double[MAX_DATA_COUNT];
		mMagnetPureZ = new double[MAX_DATA_COUNT];
		
		createUI();
	}
	
	private void createUI()
	{
		this.setLayout(new MigLayout("","[][80!][center,grow]"));
		
		mlbAx = new JLabel();
		mlbAx.setHorizontalAlignment(JLabel.LEFT);
		mlbAy = new JLabel();
		mlbAy.setHorizontalAlignment(JLabel.LEFT);
		mlbAz = new JLabel();
		mlbAz.setHorizontalAlignment(JLabel.LEFT);
		mlbGx = new JLabel();
		mlbGx.setHorizontalAlignment(JLabel.LEFT);
		mlbGy = new JLabel();
		mlbGy.setHorizontalAlignment(JLabel.LEFT);
		mlbGz = new JLabel();
		mlbGz.setHorizontalAlignment(JLabel.LEFT);
		mlbMx = new JLabel();
		mlbMx.setHorizontalAlignment(JLabel.LEFT);
		mlbMy = new JLabel();
		mlbMy.setHorizontalAlignment(JLabel.LEFT);
		mlbMz = new JLabel();
		mlbMz.setHorizontalAlignment(JLabel.LEFT);
		
		mChartViewer = new ChartViewer();
		mcbCollectData = new JCheckBox(ResBox.text("COLLECT_DATA"));
		mlbDataCount = new JLabel();
		mrbMagnetRaw = new JRadioButton(ResBox.text("SHOW_MAGNET_RAW"));
		mrbMagnetPure = new JRadioButton(ResBox.text("SHOW_MAGNET_PURE"));
		JButton btnCalibrateAccel = new JButton(ResBox.text("CALIBRATE_ACCEL"));
		JButton btnCalibrateGyro = new JButton(ResBox.text("CALIBRATE_GYRO"));
		JButton btnCalibrateMagnet = new JButton(ResBox.text("CALIBRATE_MAGNET"));
		JButton btnResetCalibration = new JButton(ResBox.text("RESET_CALIBRATION"));
		JButton btnResetMagnetCalibration = new JButton(ResBox.text("RESET_MAGNET_CALIBRATION"));
		
		ButtonGroup group = new ButtonGroup();
		group.add(mrbMagnetRaw);
		group.add(mrbMagnetPure);
		
		mrbMagnetRaw.setSelected(true);
		
		this.addComponentListener(new OnChartResize());
		mcbCollectData.addItemListener(new OnCollectData());
		mrbMagnetRaw.addItemListener(new OnChangeShowedData());
		mrbMagnetPure.addItemListener(new OnChangeShowedData());
		btnCalibrateAccel.addActionListener(new OnBtnCalibrateAccel());
		btnCalibrateGyro.addActionListener(new OnBtnCalibrateGyro());
		btnCalibrateMagnet.addActionListener(new OnBtnCalibrateMagnet());
		btnResetCalibration.addActionListener(new OnBtnResetCalibration());
		btnResetMagnetCalibration.addActionListener(new OnBtnResetMagnetCalibration());
				
		this.add(new JLabel("AccelX/Off"));
		this.add(mlbAx);
		
		this.add(mChartViewer,"spany,wrap");
		
		this.add(new JLabel("AccelY/Off"));
		this.add(mlbAy,"wrap");
		
		this.add(new JLabel("AccelZ/Off"));
		this.add(mlbAz,"wrap");
		
		this.add(new JPanel(),"h 10!,wrap");
		
		this.add(new JLabel("GyroX/Off"));
		this.add(mlbGx,"wrap");
		
		this.add(new JLabel("GyroY/Off"));
		this.add(mlbGy,"wrap");
		
		this.add(new JLabel("GyroZ/Off"));
		this.add(mlbGz,"wrap");
		
		this.add(new JPanel(),"h 10!,wrap");
		
		this.add(new JLabel("MagnetX"));
		this.add(mlbMx,"wrap");
		this.add(new JLabel("MagnetY"));
		this.add(mlbMy,"wrap");
		this.add(new JLabel("MagnetZ"));
		this.add(mlbMz,"wrap");
		
		this.add(new JPanel(),"h 10!,wrap");
		
		this.add(new JLabel(ResBox.text("DATA_COUNT")));
		this.add(mlbDataCount,"wrap");
		this.add(mcbCollectData,"spanx 2, wrap");
		this.add(mrbMagnetRaw,"spanx 2, wrap");
		this.add(mrbMagnetPure,"spanx 2, wrap");
		
		this.add(new JPanel(),"h 10!,wrap");
		
		this.add(btnCalibrateAccel,"spanx 2,grow,wrap");
		this.add(btnCalibrateGyro,"spanx 2,grow,wrap");
		this.add(btnCalibrateMagnet,"spanx 2,grow,wrap");
		this.add(btnResetMagnetCalibration,"spanx 2,grow,wrap");
		this.add(btnResetCalibration,"spanx 2,grow,wrap");
		
		this.drawData();	
	}
	
	@Override
	public void setVisible(boolean b)
	{
		if(b)
		{
			DroneTelemetry.instance().addObserver(mObserver);
		}
		else
		{
			DroneTelemetry.instance().deleteObserver(mObserver);
			
			if(mModified)
			{
				int result = JOptionPane.showConfirmDialog(	this,
															ResBox.text("CONFIRM_SAVE_SETTINGS"),
															"",
															JOptionPane.YES_NO_OPTION,
															JOptionPane.QUESTION_MESSAGE);

				if(result == JOptionPane.YES_OPTION)
				{
					DroneState droneState = DroneTelemetry.instance().getDroneState();
					
					Settings.instance().setDroneSettings(droneState);
				
					Settings.instance().save();
				}
			}
		}
		
		super.setVisible(b);
	}
	
	private DroneState.TripleAxisSensor calcSensOffsets(double x[], double y[], double z[])
	{
		DroneState.TripleAxisSensor sens = new DroneState.TripleAxisSensor();
		
		List<Double> liX = ArrayHelper.asList(x, mDataCount);
		List<Double> liY = ArrayHelper.asList(y, mDataCount);
		List<Double> liZ = ArrayHelper.asList(z, mDataCount);
		
		if(liX.size() < 2 || liY.size() < 2 || liZ.size() < 2)
			return sens;
		
		double maxMx = Collections.max(liX);
		double minMx = Collections.min(liX);
		double maxMy = Collections.max(liY);
		double minMy = Collections.min(liY);
		double maxMz = Collections.max(liZ);
		double minMz = Collections.min(liZ);

		double dx = (maxMx + minMx)/2.0;
		double dy = (maxMy + minMy)/2.0;
		double dz = (maxMz + minMz)/2.0;
		
		// round to closest int
		if(dx > 0)
			dx += 0.5;
		else
			dx -= 0.5;
		
		if(dy > 0)
			dy += 0.5;
		else
			dy -= 0.5;
		
		if(dz > 0)
			dz += 0.5;
		else
			dz -= 0.5;

		sens.offsetX = (int)(dx);
		sens.offsetY = (int)(dy);
		sens.offsetZ = (int)(dz);
		
		return sens;
	}
	
	private class AxisBounds
	{
		double min;
		double max;
	}
	
	private AxisBounds calcChartBounds(double x[], double y[], double z[])
	{
		AxisBounds result = new AxisBounds();
		
		List<Double> liX = ArrayHelper.asList(x, mDataCount);
		List<Double> liY = ArrayHelper.asList(y, mDataCount);
		List<Double> liZ = ArrayHelper.asList(z, mDataCount);
		
		if(liX.size() < 2 || liY.size() < 2 || liZ.size() < 2)
			return result;
		
		double maxMx = Collections.max(liX);
		double minMx = Collections.min(liX);
		double maxMy = Collections.max(liY);
		double minMy = Collections.min(liY);
		double maxMz = Collections.max(liZ);
		double minMz = Collections.min(liZ);
		
		if( minMx < result.min )
			result.min = minMx;
		if( minMy < result.min )
			result.min = minMy;
		if( minMz < result.min )
			result.min = minMz;
		
		if( maxMx > result.max )
			result.max = maxMx;
		if( maxMy > result.max )
			result.max = maxMy;
		if( maxMz > result.max )
			result.max = maxMz;
		
		double dx = (result.max - result.min)*0.1;
		
		result.min -= dx;
		result.max += dx;
		
		return result;
	}
	
	private void drawData()
	{
		int w = Math.min(this.getWidth() - 180, this.getHeight() - 50);
		
		if(w < 500)
			w = 500;
		
        XYChart c = new XYChart(w, w);
        c.setPlotArea(60, 45, w - 90, w - 90, -1, -1, 0xc0c0c0, 0xc0c0c0, -1);
        c.addLegend(60, 5, false);
        // Set scale for axis
        c.xAxis().setTitle("X/Y");
        c.xAxis().setMargin(0,0);
        
        c.yAxis().setTitle("Y/Z");
        c.yAxis().setMargin(0,0);

        if(mrbMagnetRaw.isSelected())
        {
        	c.addScatterLayer(mMagnetRawX, mMagnetRawY, "MagnetXY", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetRawX, mMagnetRawZ, "MagnetXZ", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetRawY, mMagnetRawZ, "MagnetYZ", Chart.CircleShape, 5);
        	
        	AxisBounds bounds = calcChartBounds(mMagnetRawX,mMagnetRawY,mMagnetRawZ);
        	
        	c.xAxis().setLinearScale(bounds.min, bounds.max);
        	c.yAxis().setLinearScale(bounds.min, bounds.max);
        	
        }
        else if(mrbMagnetPure.isSelected())
        {
        	c.addScatterLayer(mMagnetPureX, mMagnetPureY, "MagnetXY", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetPureX, mMagnetPureZ, "MagnetXZ", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetPureY, mMagnetPureZ, "MagnetYZ", Chart.CircleShape, 5);
        	
        	AxisBounds bounds = calcChartBounds(mMagnetPureX,mMagnetPureY,mMagnetPureZ);
        	
        	c.xAxis().setLinearScale(bounds.min, bounds.max);
        	c.yAxis().setLinearScale(bounds.min, bounds.max);
        }
        
        mChartViewer.setChart(c);
	}

	@Override
	protected WndState loadWndState()
	{
		return Settings.instance().getSensorsWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		Settings.instance().setSensorsWnd(ws);
	}
}
