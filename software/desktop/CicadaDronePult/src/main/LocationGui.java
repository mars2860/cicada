package main;

import java.text.DecimalFormat;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JLabel;
import javax.swing.JPanel;

import main.AppSettings.WndState;
import net.miginfocom.swing.MigLayout;
import pdl.DroneTelemetry;
import pdl.DroneState;

public class LocationGui extends JSavedFrame
{
	private static final long serialVersionUID = 510087124094046530L;
	
	private JLabel mlbHome;
	private JLabel mlbSatellCount;
	private JLabel mlbLat;
	private JLabel mlbLon;
	private JLabel mlbPosNorth;
	private JLabel mlbPosEast;

	private class OnTelemetryUpdate implements Observer
	{
		private long timestamp;
		
		@Override
		public void update(Observable o, Object arg)
		{
			if(	System.currentTimeMillis() - timestamp < 100 ||
				LocationGui.this.isVisible() == false )
				return;
			
			timestamp = System.currentTimeMillis();
			
			DroneState ds = DroneTelemetry.instance().getDroneState();
			
			DecimalFormat fmt1 = new DecimalFormat();
			fmt1.setMaximumFractionDigits(0);
			fmt1.setMinimumFractionDigits(0);
			fmt1.setGroupingUsed(false);
			
			DecimalFormat fmt2 = new DecimalFormat();
			fmt2.setMaximumFractionDigits(6);
			fmt2.setMinimumFractionDigits(6);
			fmt2.setGroupingUsed(false);
			
			DecimalFormat fmt3 = new DecimalFormat();
			fmt3.setMaximumFractionDigits(2);
			fmt3.setMinimumFractionDigits(0);
			fmt3.setGroupingUsed(false);
			
			/// Home
			String home = fmt1.format(ds.distToHome) + "m/" + fmt1.format(ds.headToHome) + "deg";
			mlbHome.setText(home);
			/// GPS
			mlbSatellCount.setText(fmt1.format(ds.gps.numSV));
			mlbLat.setText(fmt2.format(ds.gps.lat));
			mlbLon.setText(fmt2.format(ds.gps.lon));
			/// Position
			mlbPosNorth.setText(fmt3.format(ds.posNorth));
			mlbPosEast.setText(fmt3.format(ds.posEast));
		}
	}
	
	public LocationGui()
	{
		super("Location",180,330);
		this.setTitle(ResBox.text("LOCATION"));
		this.setIconImage(ResBox.icon("LOCATION").getImage());
		this.createUI();
		DroneTelemetry.instance().addObserver(new OnTelemetryUpdate());
	}
	
	private void createUI()
	{
		JPanel pnlStatus = new JPanel(new MigLayout("","[][grow]",""));
		
		mlbHome = new JLabel();
		mlbSatellCount = new JLabel();
		mlbLat = new JLabel();
		mlbLon = new JLabel();
		mlbPosNorth = new JLabel();
		mlbPosEast = new JLabel();
		
		JLabel lbHome = new JLabel(ResBox.icon("HOME"));
		JLabel lbSatellCount = new JLabel(ResBox.icon("SATELLITE"));
		JLabel lbLat = new JLabel(ResBox.icon("LATITUDE"));
		JLabel lbLon = new JLabel(ResBox.icon("LONGITUDE"));
		JLabel lbPosNorth = new JLabel(ResBox.icon("NORTH"));
		JLabel lbPosEast = new JLabel(ResBox.icon("EAST"));
		
		lbHome.setToolTipText(ResBox.text("HOME_POS"));
		lbSatellCount.setToolTipText(ResBox.text("SATELL_COUNT"));
		lbLat.setToolTipText(ResBox.text("LATITUDE"));
		lbLon.setToolTipText(ResBox.text("LONGITUDE"));
		lbPosNorth.setToolTipText(ResBox.text("POS_NORTH"));
		lbPosEast.setToolTipText(ResBox.text("POS_EAST"));

		pnlStatus.add(lbHome);
		pnlStatus.add(mlbHome,"grow,wrap");
		pnlStatus.add(lbSatellCount);
		pnlStatus.add(mlbSatellCount,"grow,wrap");
		pnlStatus.add(lbLat);
		pnlStatus.add(mlbLat,"grow,wrap");
		pnlStatus.add(lbLon);
		pnlStatus.add(mlbLon,"grow,wrap");
		pnlStatus.add(lbPosNorth);
		pnlStatus.add(mlbPosNorth,"grow,wrap");
		pnlStatus.add(lbPosEast);
		pnlStatus.add(mlbPosEast,"grow,wrap");
		
		this.add(pnlStatus);
	}

	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getLocationWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setLocationWnd(ws);	
	}
}
