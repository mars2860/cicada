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
import main.AppSettings.WndState;
import net.miginfocom.swing.MigLayout;
import pdl.DroneCommander;
import pdl.DroneTelemetry;
import pdl.DroneState;

import pdl.commands.CmdSetAccel;
import pdl.commands.CmdSetGyro;
import pdl.commands.CmdSetMagneto;
import pdl.res.Profile;

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
					Integer.toString((int)droneState.magneto.offsetX) + "/" + fmt.format(droneState.magneto.scaleX);
			mlbMx.setText(text);
			mlbMx.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.magneto.rawY) + "/" +
					Integer.toString((int)droneState.magneto.offsetY) + "/" + fmt.format(droneState.magneto.scaleY);
			mlbMy.setText(text);
			mlbMy.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.magneto.rawZ) + "/" +
					Integer.toString((int)droneState.magneto.offsetZ) + "/" + fmt.format(droneState.magneto.scaleZ);
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
						
			//DroneState.TripleAxisSensor sens = calcSensOffsets(mMagnetRawX,mMagnetRawY,mMagnetRawZ);
			
			DroneState.Magneto sens = new DroneState.Magneto();
			
			List<Double> liX = ArrayHelper.asList(mMagnetRawX, mDataCount);
			List<Double> liY = ArrayHelper.asList(mMagnetRawY, mDataCount);
			List<Double> liZ = ArrayHelper.asList(mMagnetRawZ, mDataCount);
			
			if(liX.size() < 2 || liY.size() < 2 || liZ.size() < 2)
				return;
			
			double maxMx = Collections.max(liX);
			double minMx = Collections.min(liX);
			double maxMy = Collections.max(liY);
			double minMy = Collections.min(liY);
			double maxMz = Collections.max(liZ);
			double minMz = Collections.min(liZ);

			// Hard-iron compensation
			double dx = (maxMx + minMx)/2.0;
			double dy = (maxMy + minMy)/2.0;
			double dz = (maxMz + minMz)/2.0;
			
			sens.offsetX = (int)(dx + 0.5);
			sens.offsetY = (int)(dy + 0.5);
			sens.offsetZ = (int)(dz + 0.5);
			
			// Soft-iron compensation
	        double scaleX = (maxMx - minMx) / 2.;  // get average x axis max chord length in counts
	        double scaleY = (maxMy - minMy) / 2.;  // get average y axis max chord length in counts
	        double scaleZ = (maxMz - minMz) / 2.;  // get average z axis max chord length in counts

	        double avg_rad = scaleX + scaleY + scaleZ;
	        avg_rad /= 3.0;

	        sens.scaleX = (float)(avg_rad / scaleX);
	        sens.scaleY = (float)(avg_rad / scaleY);
	        sens.scaleZ = (float)(avg_rad / scaleZ);

			DroneCommander.instance().sendSetupCmd(new CmdSetMagneto(sens));
			
			mModified = true;
		}
	}
	
	private class OnBtnResetCalibration implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			DroneCommander.instance().addCmd(new CmdSetAccel(new DroneState.Accel()));
			DroneCommander.instance().addCmd(new CmdSetGyro(new DroneState.Gyro()));
			DroneCommander.instance().addCmd(new CmdSetMagneto(new DroneState.Magneto()));
			
			mModified = true;
		}
	}
	
	private class OnBtnResetMagnetCalibration implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			DroneCommander.instance().addCmd(new CmdSetMagneto(new DroneState.Magneto()));
			
			mModified = true;
		}
	}
	
	private class OnBtnCalibrateAccel implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			//DroneCommander.instance().addCmd(new CmdSelfCalibrateAccel());
			//JOptionPane.showMessageDialog(SensorsGui.this, ResBox.text("WAIT_CALIBRATION"),"",JOptionPane.INFORMATION_MESSAGE);
			
			//DroneState.Accel accel = Profile.instance().getDroneSettings().accel;
			DroneState.Accel accel = DroneTelemetry.instance().getDroneState().accel;
			DroneState.TripleAxisSensor sens = calcSensOffsets(mAccelRawX,mAccelRawY,mAccelRawZ);
			
			sens.offsetX = accel.offsetX - sens.offsetX;
			sens.offsetY = accel.offsetY - sens.offsetY;
			sens.offsetZ = accel.offsetZ - sens.offsetZ;
			sens.offsetZ += 2048;
			
			DroneState.Accel naccel = new DroneState.Accel();
			naccel.offsetX = sens.offsetX;
			naccel.offsetY = sens.offsetY;
			naccel.offsetZ = sens.offsetZ;
			naccel.dlpf = accel.dlpf;
			
			// MPU9250/6500/6050 apply only even values
			if(naccel.offsetX % 2 != 0)
				naccel.offsetX += 1;
			if(naccel.offsetY % 2 != 0)
				naccel.offsetY += 1;
			if(naccel.offsetZ % 2 != 0)
				naccel.offsetZ += 1;
			
			DroneCommander.instance().sendSetupCmd(new CmdSetAccel(naccel));
			
			mModified = true;
		}
	}
	
	private class OnBtnCalibrateGyro implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			//DroneCommander.instance().addCmd(new CmdSelfCalibrateGyro());
			//JOptionPane.showMessageDialog(SensorsGui.this, ResBox.text("WAIT_CALIBRATION"),"",JOptionPane.INFORMATION_MESSAGE);
			
			//DroneState.Gyro gyro = Profile.instance().getDroneSettings().gyro;
			DroneState.Gyro gyro = DroneTelemetry.instance().getDroneState().gyro;
			DroneState.TripleAxisSensor sens = calcSensOffsets(mGyroRawX,mGyroRawY,mGyroRawZ);
			
			sens.offsetX = gyro.offsetX - 2*sens.offsetX;
			sens.offsetY = gyro.offsetY - 2*sens.offsetY;
			sens.offsetZ = gyro.offsetZ - 2*sens.offsetZ;
			
			DroneState.Gyro ngyro = new DroneState.Gyro();
			ngyro.offsetX = sens.offsetX;
			ngyro.offsetY = sens.offsetY;
			ngyro.offsetZ = sens.offsetZ;
			ngyro.dlpf = gyro.dlpf;
			
			// MPU9250/6500/6050 apply only even values
			if(ngyro.offsetX % 2 != 0)
				ngyro.offsetX += 1;
			if(ngyro.offsetY % 2 != 0)
				ngyro.offsetY += 1;
			if(ngyro.offsetZ % 2 != 0)
				ngyro.offsetZ += 1;
			
			DroneCommander.instance().sendSetupCmd(new CmdSetGyro(ngyro));
			
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
					
					Profile.instance().setDroneSettings(droneState);
					Profile.instance().save();
				}
			}
		}
		
		super.setVisible(b);
	}
	
	private double average(List<Double> list)
	{
		if(list.size() == 0)
			return 0;
		
		double sum = 0;
		double sz = list.size();
		
		for(double val : list)
			sum += val;
		
		return sum/sz;
	}
	
	private DroneState.TripleAxisSensor calcSensOffsets(double x[], double y[], double z[])
	{
		DroneState.TripleAxisSensor sens = new DroneState.Accel();
		
		List<Double> liX = ArrayHelper.asList(x, mDataCount);
		List<Double> liY = ArrayHelper.asList(y, mDataCount);
		List<Double> liZ = ArrayHelper.asList(z, mDataCount);
		
		if(liX.size() < 2 || liY.size() < 2 || liZ.size() < 2)
			return sens;
		
		/*
		double maxMx = Collections.max(liX);
		double minMx = Collections.min(liX);
		double maxMy = Collections.max(liY);
		double minMy = Collections.min(liY);
		double maxMz = Collections.max(liZ);
		double minMz = Collections.min(liZ);

		double dx = (maxMx + minMx)/2.0;
		double dy = (maxMy + minMy)/2.0;
		double dz = (maxMz + minMz)/2.0;
		*/
		
		double dx = average(liX);
		double dy = average(liY);
		double dz = average(liZ);
		
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
		
		result.min = minMx;
		result.max = maxMx;
		
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
		
		// make beauty bounds 
		
		double val = Math.max(Math.abs(result.min),Math.abs(result.max));
		val = (double)((int)val - (((int)val)%10) + 10);
		
		if(result.min < 0)
			result.min = -val;
		else
			result.min = val;
		
		if(result.max < 0)
			result.max = -val;
		else
			result.max = val;
		
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
        	
        	c.xAxis().setLinearScale(bounds.min, bounds.max, Math.abs(bounds.max - bounds.min)/20.0);
        	c.yAxis().setLinearScale(bounds.min, bounds.max, Math.abs(bounds.max - bounds.min)/20.0);
        	
        }
        else if(mrbMagnetPure.isSelected())
        {
        	c.addScatterLayer(mMagnetPureX, mMagnetPureY, "MagnetXY", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetPureX, mMagnetPureZ, "MagnetXZ", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetPureY, mMagnetPureZ, "MagnetYZ", Chart.CircleShape, 5);
        	
        	AxisBounds bounds = calcChartBounds(mMagnetPureX,mMagnetPureY,mMagnetPureZ);
        	
        	c.xAxis().setLinearScale(bounds.min, bounds.max, Math.abs(bounds.max - bounds.min)/20.0);
        	c.yAxis().setLinearScale(bounds.min, bounds.max, Math.abs(bounds.max - bounds.min)/20.0);
        }
        
        mChartViewer.setChart(c);
	}

	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getSensorsWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setSensorsWnd(ws);
	}
}
