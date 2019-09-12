package main;

import java.util.Observable;
import java.util.Observer;

import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

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
	
	public SensorsDlg(JFrame owner)
	{
		super(owner,true);
		
		mObserver = new OnTelemetryUpdate();
		
		this.setTitle(Text.get("SENSORS"));
		this.setSize(460, 300);
		this.setLocationRelativeTo(null);
		this.setResizable(true);
		this.setLayout(new MigLayout());
		
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
		
		this.add(new JLabel("AccelX/Off"));
		this.add(mlbAx,"wrap");
		
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
}
