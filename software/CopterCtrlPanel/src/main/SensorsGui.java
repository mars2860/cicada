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
import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.DroneState;
import copter.commands.CmdCalibrateAccel;
import copter.commands.CmdCalibrateGyro;
import copter.commands.CmdCalibrateMagnet;
import copter.commands.CmdSelfCalibrateAccel;
import copter.commands.CmdSelfCalibrateGyro;
import helper.ArrayHelper;
import main.Settings.WndState;
import net.miginfocom.swing.MigLayout;

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
			
			DroneState droneState = CopterTelemetry.instance().getDroneState();
			
			text = ": " + Integer.toString((int)droneState.accel.filteredX) + "/" +
							Integer.toString((int)droneState.accel.offsetX);
			mlbAx.setText(text);
			mlbAx.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.accel.filteredY) + "/" +
					Integer.toString((int)droneState.accel.offsetY);
			mlbAy.setText(text);
			mlbAy.setToolTipText(text);
	
			text = ": " + Integer.toString((int)droneState.accel.filteredZ) + "/" +
					Integer.toString((int)droneState.accel.offsetZ);
			mlbAz.setText(text);
			mlbAz.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.gyroRad.filteredX) + "/" +
					Integer.toString((int)droneState.gyroRad.offsetX);
			mlbGx.setText(text);
			mlbGx.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.gyroRad.filteredY) + "/" +
					Integer.toString((int)droneState.gyroRad.offsetY);
			mlbGy.setText(text);
			mlbGy.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.gyroRad.filteredZ) + "/" +
					Integer.toString((int)droneState.gyroRad.offsetZ);
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
			
			text = ": " + Integer.toString((int)droneState.magneto.filteredX) + "/" +
					Integer.toString((int)droneState.magneto.offsetX);
			mlbMx.setText(text);
			mlbMx.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.magneto.filteredY) + "/" +
					Integer.toString((int)droneState.magneto.offsetY);
			mlbMy.setText(text);
			mlbMy.setToolTipText(text);
			
			text = ": " + Integer.toString((int)droneState.magneto.filteredZ) + "/" +
					Integer.toString((int)droneState.magneto.offsetZ);
			mlbMz.setText(text);
			mlbMz.setToolTipText(text);
			
			if(mcbCollectData.isSelected())
			{
				if(mDataCount < MAX_DATA_COUNT)
				{
					mAccelX[mDataCount] = droneState.accel.filteredX;
					mAccelY[mDataCount] = droneState.accel.filteredY;
					mAccelZ[mDataCount] = droneState.accel.filteredZ;
					
					mGyroX[mDataCount] = droneState.gyroRad.filteredX;
					mGyroY[mDataCount] = droneState.gyroRad.filteredY;
					mGyroZ[mDataCount] = droneState.gyroRad.filteredZ;
				
					mMagnetX[mDataCount] = droneState.magneto.filteredX;
					mMagnetY[mDataCount] = droneState.magneto.filteredY;
					mMagnetZ[mDataCount] = droneState.magneto.filteredZ;
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
			List<Double> liMagnetX = ArrayHelper.asList(mMagnetX, mDataCount);
			List<Double> liMagnetY = ArrayHelper.asList(mMagnetY, mDataCount);
			List<Double> liMagnetZ = ArrayHelper.asList(mMagnetZ, mDataCount);
			
			if(liMagnetX.size() < 2 || liMagnetY.size() < 2 || liMagnetZ.size() < 2)
				return;
			
			double maxMx = Collections.max(liMagnetX);
			double minMx = Collections.min(liMagnetX);
			double maxMy = Collections.max(liMagnetY);
			double minMy = Collections.min(liMagnetY);
			double maxMz = Collections.max(liMagnetZ);
			double minMz = Collections.min(liMagnetZ);
				
			double sx = Math.max((maxMy - minMy)/(maxMx - minMx), 1.0);
			double sy = Math.max((maxMx - minMx)/(maxMy - minMy), 1.0);
			double sz = Math.max((maxMy - minMy)/(maxMz - minMz), 1.0);
			
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
			
			mModified = true;
		}
	}
	
	private class OnBtnResetCalibration implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			CopterCommander.instance().addCmd(new CmdCalibrateAccel(0,0,0));
			CopterCommander.instance().addCmd(new CmdCalibrateGyro(0,0,0));
			CopterCommander.instance().addCmd(new CmdCalibrateMagnet(0,0,0,1.f,1.f,1.f));
			
			mModified = true;
		}
	}
	
	private class OnBtnResetMagnetCalibration implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			CopterCommander.instance().addCmd(new CmdCalibrateMagnet(0,0,0,1.f,1.f,1.f));
			
			mModified = true;
		}
	}
	
	private class OnBtnCalibrateAccel implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			CopterCommander.instance().addCmd(new CmdSelfCalibrateAccel());
			//JOptionPane.showMessageDialog(SensorsGui.this, Text.get("WAIT_CALIBRATION"),"",JOptionPane.INFORMATION_MESSAGE);
			
			mModified = true;
		}
	}
	
	private class OnBtnCalibrateGyro implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			CopterCommander.instance().addCmd(new CmdSelfCalibrateGyro());
			//JOptionPane.showMessageDialog(SensorsGui.this, Text.get("WAIT_CALIBRATION"),"",JOptionPane.INFORMATION_MESSAGE);
			
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
	private JRadioButton mrbAccel;
	private JRadioButton mrbMagnet;
	
	private static final int MAX_DATA_COUNT = 1024;
	
	private int mDataCount;
	private double mAccelX[];
	private double mAccelY[];
	private double mAccelZ[];
	private double mGyroX[];
	private double mGyroY[];
	private double mGyroZ[];
	private double mMagnetX[];
	private double mMagnetY[];
	private double mMagnetZ[];
	
	private boolean mModified = false;
	
	public SensorsGui()
	{
		super("Sensors",680,568);
		
		this.setTitle(ResBox.text("SENSORS"));
		this.setIconImage(ResBox.icon("SENSORS").getImage());
		
		mObserver = new OnTelemetryUpdate();
		mAccelX = new double[MAX_DATA_COUNT];
		mAccelY = new double[MAX_DATA_COUNT];
		mAccelZ = new double[MAX_DATA_COUNT];
		mGyroX = new double[MAX_DATA_COUNT];
		mGyroY = new double[MAX_DATA_COUNT];
		mGyroZ = new double[MAX_DATA_COUNT];
		mMagnetX = new double[MAX_DATA_COUNT];
		mMagnetY = new double[MAX_DATA_COUNT];
		mMagnetZ = new double[MAX_DATA_COUNT];
		
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
		mrbAccel = new JRadioButton(ResBox.text("SHOW_ACCEL"));
		mrbMagnet = new JRadioButton(ResBox.text("SHOW_MAGNET"));
		JButton btnCalibrateAccel = new JButton(ResBox.text("CALIBRATE_ACCEL"));
		JButton btnCalibrateGyro = new JButton(ResBox.text("CALIBRATE_GYRO"));
		JButton btnCalibrateMagnet = new JButton(ResBox.text("CALIBRATE_MAGNET"));
		JButton btnResetCalibration = new JButton(ResBox.text("RESET_CALIBRATION"));
		JButton btnResetMagnetCalibration = new JButton(ResBox.text("RESET_MAGNET_CALIBRATION"));
		
		ButtonGroup group = new ButtonGroup();
		group.add(mrbAccel);
		group.add(mrbMagnet);
		
		mrbAccel.setSelected(true);
		
		this.addComponentListener(new OnChartResize());
		mcbCollectData.addItemListener(new OnCollectData());
		mrbAccel.addItemListener(new OnChangeShowedData());
		mrbMagnet.addItemListener(new OnChangeShowedData());
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
		this.add(mrbAccel,"spanx 2, wrap");
		this.add(mrbMagnet,"spanx 2, wrap");
		
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
			CopterTelemetry.instance().addObserver(mObserver);
		}
		else
		{
			CopterTelemetry.instance().deleteObserver(mObserver);
			
			if(mModified)
			{
				int result = JOptionPane.showConfirmDialog(	this,
															ResBox.text("CONFIRM_SAVE_SETTINGS"),
															"",
															JOptionPane.YES_NO_OPTION,
															JOptionPane.QUESTION_MESSAGE);

				if(result == JOptionPane.YES_OPTION)
				{
					DroneState droneState = CopterTelemetry.instance().getDroneState();
					
					Settings.instance().setDroneSettings(droneState);
				
					Settings.instance().save();
				}
			}
		}
		
		super.setVisible(b);
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

        if(mrbAccel.isSelected())
        {
        	c.xAxis().setLinearScale(-18432, 18432, 2048);
        	c.yAxis().setLinearScale(-18432, 18432, 2048);
        	
        	c.addScatterLayer(mAccelX, mAccelY, "AccelXY", Chart.CircleShape, 5);
        	c.addScatterLayer(mAccelX, mAccelZ, "AccelXZ", Chart.CircleShape, 5);
        	c.addScatterLayer(mAccelY, mAccelZ, "AccelYZ", Chart.CircleShape, 5);
        }
        else if(mrbMagnet.isSelected())
        {
        	c.xAxis().setLinearScale(-9216, 9216, 1024);
        	c.yAxis().setLinearScale(-9216, 9216, 1024);
        	
        	c.addScatterLayer(mMagnetX, mMagnetY, "MagnetXY", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetX, mMagnetZ, "MagnetXZ", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetY, mMagnetZ, "MagnetYZ", Chart.CircleShape, 5);
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
