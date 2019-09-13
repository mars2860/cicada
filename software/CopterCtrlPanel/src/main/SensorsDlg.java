package main;

import java.util.Observable;
import java.util.Observer;

import javax.swing.ButtonGroup;
import javax.swing.JCheckBox;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import ChartDirector.Chart;
import ChartDirector.ChartViewer;
import ChartDirector.XYChart;
import copter.CopterTelemetry;
import net.miginfocom.swing.MigLayout;

public class SensorsDlg extends JDialog
{
	private static final long serialVersionUID = -4310119839211305793L;

	private class OnTelemetryUpdate implements Observer
	{
		@Override
		public void update(Observable o, Object arg)
		{
			String text;
			
			text = ": " + Integer.toString(CopterTelemetry.instance().getAccelX()) + "/" +
							Integer.toString(CopterTelemetry.instance().getAccelOffsetX());
			mlbAx.setText(text);
			
			text = ": " + Integer.toString(CopterTelemetry.instance().getAccelY()) + "/" +
					Integer.toString(CopterTelemetry.instance().getAccelOffsetY());
			mlbAy.setText(text);
	
			text = ": " + Integer.toString(CopterTelemetry.instance().getAccelZ()) + "/" +
					Integer.toString(CopterTelemetry.instance().getAccelOffsetZ());
			mlbAz.setText(text);
			
			text = ": " + Integer.toString(CopterTelemetry.instance().getGyroX()) + "/" +
					Integer.toString(CopterTelemetry.instance().getGyroOffsetX());
			mlbGx.setText(text);
			
			text = ": " + Integer.toString(CopterTelemetry.instance().getGyroY()) + "/" +
					Integer.toString(CopterTelemetry.instance().getGyroOffsetY());
			mlbGy.setText(text);
			
			text = ": " + Integer.toString(CopterTelemetry.instance().getGyroZ()) + "/" +
					Integer.toString(CopterTelemetry.instance().getGyroOffsetZ());
			mlbGz.setText(text);
			
			text = ": " + Integer.toString(CopterTelemetry.instance().getMagnetX());// + "/" +
					//Integer.toString(CopterTelemetry.instance().getGyroOffsetX());
			mlbMx.setText(text);
			
			text = ": " + Integer.toString(CopterTelemetry.instance().getMagnetY());// + "/" +
			//Integer.toString(CopterTelemetry.instance().getGyroOffsetX());
			mlbMy.setText(text);
			
			text = ": " + Integer.toString(CopterTelemetry.instance().getMagnetZ());// + "/" +
			//Integer.toString(CopterTelemetry.instance().getGyroOffsetX());
			mlbMz.setText(text);
			
			if(mcbCollectData.isSelected())
			{
				if(mDataCount < MAX_DATA_COUNT)
				{
					mAccelX[mDataCount] = CopterTelemetry.instance().getAccelX();
					mAccelY[mDataCount] = CopterTelemetry.instance().getAccelY();
					mAccelZ[mDataCount] = CopterTelemetry.instance().getAccelZ();
				
					mMagnetX[mDataCount] = CopterTelemetry.instance().getMagnetX();
					mMagnetY[mDataCount] = CopterTelemetry.instance().getMagnetY();
					mMagnetZ[mDataCount] = CopterTelemetry.instance().getMagnetZ();
				}
				
				mDataCount++;
				if(mDataCount >= MAX_DATA_COUNT)
					mcbCollectData.setSelected(false);
			}
			
			text = ": " + Integer.toString(mDataCount);
			mlbDataCount.setText(text);
		}
	}
	
	private class OnCollectData implements ChangeListener
	{
		@Override
		public void stateChanged(ChangeEvent e)
		{
			if(mcbCollectData.isSelected())
			{
				mDataCount = 0;
			}
			else
			{
				SensorsDlg.this.drawData();
			}
			
		}
	}
	
	private class OnChangeShowedData implements ChangeListener
	{
		@Override
		public void stateChanged(ChangeEvent e)
		{
			SensorsDlg.this.drawData();
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
	
	private static final int MAX_DATA_COUNT = 256;
	
	private int mDataCount;
	private double mAccelX[];
	private double mAccelY[];
	private double mAccelZ[];
	private double mMagnetX[];
	private double mMagnetY[];
	private double mMagnetZ[];
	
	public SensorsDlg(JFrame owner)
	{
		super(owner,true);
		
		mObserver = new OnTelemetryUpdate();
		mAccelX = new double[MAX_DATA_COUNT];
		mAccelY = new double[MAX_DATA_COUNT];
		mAccelZ = new double[MAX_DATA_COUNT];
		mMagnetX = new double[MAX_DATA_COUNT];
		mMagnetY = new double[MAX_DATA_COUNT];
		mMagnetZ = new double[MAX_DATA_COUNT];
		
		this.setTitle(Text.get("SENSORS"));
		this.setSize(680, 550);
		this.setLocationRelativeTo(null);
		this.setResizable(false);
		this.setLayout(new MigLayout("","[][80!]"));
		
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
		mcbCollectData = new JCheckBox(Text.get("COLLECT_DATA"));
		mlbDataCount = new JLabel();
		mrbAccel = new JRadioButton(Text.get("SHOW_ACCEL"));
		mrbMagnet = new JRadioButton(Text.get("SHOW_MAGNET"));
		
		ButtonGroup group = new ButtonGroup();
		group.add(mrbAccel);
		group.add(mrbMagnet);
		
		mrbAccel.setSelected(true);
		mcbCollectData.addChangeListener(new OnCollectData());
		mrbAccel.addChangeListener(new OnChangeShowedData());
		mrbMagnet.addChangeListener(new OnChangeShowedData());
		
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
		
		this.add(new JLabel(Text.get("DATA_COUNT")));
		this.add(mlbDataCount,"wrap");
		this.add(mcbCollectData,"spanx 2, wrap");
		this.add(mrbAccel,"spanx 2, wrap");
		this.add(mrbMagnet,"spanx 2, wrap");
		
		this.drawData();
	}
	
	@Override
	public void setVisible(boolean b)
	{
		if(b)
			CopterTelemetry.instance().addObserver(mObserver);
		else
			CopterTelemetry.instance().deleteObserver(mObserver);
		
		super.setVisible(b);
	}
	
	private void drawData()
	{
        // Create a XYChart object of size 250 x 250 pixels
        XYChart c = new XYChart(500, 500);
        // Set the plotarea at (30, 20) and of size 200 x 200 pixels
        c.setPlotArea(60, 45, 410, 410, -1, -1, 0xc0c0c0, 0xc0c0c0, -1);
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
        	c.xAxis().setLinearScale(-13312, 13312, 1024);
        	c.yAxis().setLinearScale(-13312, 13312, 1024);
        	
        	c.addScatterLayer(mMagnetX, mMagnetY, "MagnetXY", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetX, mMagnetZ, "MagnetXZ", Chart.CircleShape, 5);
        	c.addScatterLayer(mMagnetY, mMagnetZ, "MagnetYZ", Chart.CircleShape, 5);
        }
        
        mChartViewer.setChart(c);
	}
}
