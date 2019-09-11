package main;

import java.util.Observable;
import java.util.Observer;

import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;

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
			mlbAx.setText(Integer.toString(CopterTelemetry.instance().getAccelX()));
			mlbAy.setText(Integer.toString(CopterTelemetry.instance().getAccelY()));
			mlbAz.setText(Integer.toString(CopterTelemetry.instance().getAccelZ()));
			mlbGx.setText(Integer.toString(CopterTelemetry.instance().getGyroX()));
			mlbGy.setText(Integer.toString(CopterTelemetry.instance().getGyroY()));
			mlbGz.setText(Integer.toString(CopterTelemetry.instance().getGyroZ()));
		}
	}
	
	private OnTelemetryUpdate mObserver;
	
	private JLabel mlbAx;
	private JLabel mlbAy;
	private JLabel mlbAz;
	private JLabel mlbGx;
	private JLabel mlbGy;
	private JLabel mlbGz;
	
	public SensorsDlg(JFrame owner)
	{
		super(owner,true);
		
		mObserver = new OnTelemetryUpdate();
		
		this.setTitle(Text.get("SENSORS"));
		this.setSize(460, 300);
		this.setLocationRelativeTo(null);
		this.setResizable(false);
		this.setLayout(new MigLayout());
		
		mlbAx = new JLabel();
		mlbAx.setHorizontalAlignment(JLabel.CENTER);
		mlbAy = new JLabel();
		mlbAy.setHorizontalAlignment(JLabel.CENTER);
		mlbAz = new JLabel();
		mlbAz.setHorizontalAlignment(JLabel.CENTER);
		mlbGx = new JLabel();
		mlbGx.setHorizontalAlignment(JLabel.CENTER);
		mlbGy = new JLabel();
		mlbGy.setHorizontalAlignment(JLabel.CENTER);
		mlbGz = new JLabel();
		mlbGz.setHorizontalAlignment(JLabel.CENTER);
		
		this.add(new JLabel("ax = "));
		this.add(mlbAx,"w 40!");
		this.add(new JLabel("ay = "));
		this.add(mlbAy,"w 40!");
		this.add(new JLabel("az = "));
		this.add(mlbAz,"w 40!");
		this.add(new JLabel("gx = "));
		this.add(mlbGx,"w 40!");
		this.add(new JLabel("gy = "));
		this.add(mlbGy,"w 40!");
		this.add(new JLabel("gz = "));
		this.add(mlbGz,"w 40!");
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
