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

public class StatusGui extends JSavedFrame
{
	private static final long serialVersionUID = -1172591432888209143L;
	
	private JLabel mlbBattery;
	private JLabel mlbWifiLevel;
	private JLabel mlbYaw;
	private JLabel mlbYawValue;
	private JLabel mlbPitch;
	private JLabel mlbPitchValue;
	private JLabel mlbRoll;
	private JLabel mlbRollValue;
	private JLabel mlbVertVelo;
	private JLabel mlbHorVelo;
	private JLabel mlbLoopTime;
	private JLabel mlbTemperature;
	private JLabel mlbPressure;
	private JLabel mlbAltitude;
	private JLabel mlbLidarRange;
	private JLabel mlbSystemTime;
	private JLabel mlbFlyTime;

	private class OnTelemetryUpdate implements Observer
	{
		private long timestamp;
		
		@Override
		public void update(Observable o, Object arg)
		{
			if(	System.currentTimeMillis() - timestamp < 100 ||
				StatusGui.this.isVisible() == false )
				return;
			
			timestamp = System.currentTimeMillis();
			
			DroneState droneState = DroneTelemetry.instance().getDroneState();
			
			DecimalFormat fmt1 = new DecimalFormat();
			fmt1.setMaximumFractionDigits(2);
			fmt1.setMinimumFractionDigits(0);
			fmt1.setGroupingUsed(false);
			DecimalFormat fmt2 = new DecimalFormat();
			fmt2.setMaximumFractionDigits(0);
			fmt2.setGroupingUsed(false);
			DecimalFormat fmt3 = new DecimalFormat();
			fmt3.setMaximumFractionDigits(3);
			fmt3.setGroupingUsed(false);
			
			String batState = fmt1.format(droneState.battery.voltage) + "V/" + 
								fmt1.format(droneState.battery.percent) + "%";
			
			mlbBattery.setText(batState);
			mlbWifiLevel.setText(Integer.toString((int)droneState.rssi));
			
			double yaw = droneState.yawDeg;
			double yawTarget = droneState.headingPid.targetDeg;
			double pitch = droneState.pitchDeg;
			double pitchTarget = droneState.pitchPid.targetDeg;
			double roll = droneState.rollDeg;
			double rollTarget = droneState.rollPid.targetDeg;
			
			if(droneState.headingPid.enabled == false)
				yawTarget = droneState.yawRateTargetDeg;
			if(droneState.pitchPid.enabled == false)
				pitchTarget = droneState.pitchRateTargetDeg;
			if(droneState.rollPid.enabled == false)
				rollTarget = droneState.rollRateTargetDeg;
			
			mlbYawValue.setText(fmt1.format(yaw) + "/" + fmt1.format(yawTarget));
			mlbPitchValue.setText(fmt1.format(pitch) + "/" + fmt1.format(pitchTarget));
			mlbRollValue.setText(fmt1.format(roll) + "/" + fmt1.format(rollTarget));
			mlbHorVelo.setText(fmt1.format(droneState.velGnd));
			mlbVertVelo.setText(fmt1.format(droneState.velUp));
			mlbLoopTime.setText(fmt2.format(droneState.loopPeriod));
			mlbTemperature.setText(fmt1.format(droneState.temperature));
			mlbPressure.setText(fmt1.format(droneState.baro.pressure) + "/" +
								fmt1.format(droneState.baro.seaLevelPressure));
			mlbAltitude.setText(fmt3.format(droneState.altitude) + "/" +
								fmt3.format(droneState.altPid.target));
			mlbLidarRange.setText(fmt3.format(droneState.lidar.range));
			
			mlbYaw.setIcon(rotateImageIcon(ResBox.icon("YAW"), yaw));
			mlbPitch.setIcon(rotateImageIcon(ResBox.icon("PITCH"), -pitch));
			mlbRoll.setIcon(rotateImageIcon(ResBox.icon("ROLL"), roll));
			
			//---------------------------------------
			// System time
			DecimalFormat fmt5 = new DecimalFormat();
			fmt5.setMinimumIntegerDigits(2);
			DecimalFormat fmt6 = new DecimalFormat();
			fmt6.setMinimumIntegerDigits(2);
			
			int secs = (int)(droneState.timestamp / 1000000.0);
			int mins = secs / 60;
			secs = secs - mins*60;
			mlbSystemTime.setText(fmt5.format(mins) + ":" + fmt6.format(secs));
			//----------------------------------------
			// Fly time
			secs = (int)(DroneTelemetry.instance().getFlyTime() / 1000000.0);
			mins = secs / 60;
			secs = secs - mins*60;
			mlbFlyTime.setText(fmt5.format(mins) + ":" + fmt6.format(secs));
		}
	}
	
	public StatusGui()
	{
		super("Status",180,330);
		this.setTitle(ResBox.text("STATUS"));
		this.setIconImage(ResBox.icon("GAUGE").getImage());
		this.createUI();
		DroneTelemetry.instance().addObserver(new OnTelemetryUpdate());
	}
	
	private void createUI()
	{
		JPanel pnlStatus = new JPanel(new MigLayout("","[][grow]",""));
		
		mlbSystemTime = new JLabel();
		mlbFlyTime = new JLabel();

		mlbBattery = new JLabel();
		//mlbBattery.setHorizontalAlignment(JLabel.CENTER);
		
		mlbWifiLevel = new JLabel();
		//mlbWifiLevel.setHorizontalAlignment(JLabel.CENTER);
		
		mlbYawValue = new JLabel();
		//mlbYawValue.setHorizontalAlignment(JLabel.CENTER);
		
		mlbPitchValue = new JLabel();
		//mlbPitchValue.setHorizontalAlignment(JLabel.CENTER);
		
		mlbRollValue = new JLabel();
		//mlbRollValue.setHorizontalAlignment(JLabel.CENTER);
		
		mlbHorVelo = new JLabel();
		mlbVertVelo = new JLabel();
		//mlbHeading.setHorizontalAlignment(JLabel.CENTER);
		
		mlbYaw = new JLabel(ResBox.icon("YAW"));
		mlbPitch = new JLabel(ResBox.icon("PITCH"));
		mlbRoll = new JLabel(ResBox.icon("ROLL"));
		
		mlbLoopTime = new JLabel();
		//mlbLoopTime.setHorizontalAlignment(JLabel.CENTER);
		
		mlbTemperature = new JLabel();
		//mlbTemperature.setHorizontalAlignment(JLabel.CENTER);
		
		mlbPressure = new JLabel();
		//mlbPressure.setHorizontalAlignment(JLabel.CENTER);
		
		mlbAltitude = new JLabel();
		//mlbAltitude.setHorizontalAlignment(JLabel.CENTER);
		
		mlbLidarRange = new JLabel();
		
		JLabel lbSystemTime = new JLabel(ResBox.icon("CLOCK"));
		JLabel lbFlyTime = new JLabel(ResBox.icon("STOPWATCH"));
		JLabel lbBattery = new JLabel(ResBox.icon("BATTERY"));
		JLabel lbWifiLevel = new JLabel(ResBox.icon("WIFI"));
		JLabel lbAltitude = new JLabel(ResBox.icon("ALTITUDE"));
		JLabel lbTemperature = new JLabel(ResBox.icon("TEMPERATURE"));
		JLabel lbPressure = new JLabel(ResBox.icon("PRESSURE"));
		JLabel lbLoopTime = new JLabel(ResBox.icon("CPUTIME"));
		JLabel lbLidarRange = new JLabel(ResBox.icon("LIDAR"));
		JLabel lbHorVelo = new JLabel(ResBox.icon("HOR_VELO"));
		JLabel lbVertVelo = new JLabel(ResBox.icon("VERT_VELO"));
		
		lbBattery.setToolTipText(ResBox.text("BATTERY_STATE"));
		lbWifiLevel.setToolTipText(ResBox.text("WIFI_STATE"));
		mlbYaw.setToolTipText(ResBox.text("YAW_STATE"));
		mlbPitch.setToolTipText(ResBox.text("PITCH_STATE"));
		mlbRoll.setToolTipText(ResBox.text("ROLL_STATE"));
		lbHorVelo.setToolTipText(ResBox.text("HOR_VELO"));
		lbVertVelo.setToolTipText(ResBox.text("VERT_VELO"));
		lbAltitude.setToolTipText(ResBox.text("ALTITUDE_STATE"));
		lbTemperature.setToolTipText(ResBox.text("TEMPERATURE_STATE"));
		lbPressure.setToolTipText(ResBox.text("PRESSURE_STATE"));
		lbLoopTime.setToolTipText(ResBox.text("LOOP_TIME_STATE"));
		lbLidarRange.setToolTipText(ResBox.text("LIDAR_RANGE"));
		lbSystemTime.setToolTipText(ResBox.text("SYSTEM_TIME"));
		lbFlyTime.setToolTipText(ResBox.text("FLY_TIME"));
		
		pnlStatus.add(lbBattery);
		pnlStatus.add(mlbBattery,"grow,wrap");
		pnlStatus.add(lbFlyTime);
		pnlStatus.add(mlbFlyTime,"grow,wrap");
		pnlStatus.add(lbWifiLevel);
		pnlStatus.add(mlbWifiLevel,"grow,wrap");
		pnlStatus.add(mlbYaw);
		pnlStatus.add(mlbYawValue,"grow,wrap");
		pnlStatus.add(mlbPitch);
		pnlStatus.add(mlbPitchValue,"grow,wrap");
		pnlStatus.add(mlbRoll);
		pnlStatus.add(mlbRollValue,"grow,wrap");
		pnlStatus.add(lbVertVelo);
		pnlStatus.add(mlbVertVelo,"grow,wrap");
		pnlStatus.add(lbHorVelo);
		pnlStatus.add(mlbHorVelo,"grow,wrap");
		pnlStatus.add(lbAltitude);
		pnlStatus.add(mlbAltitude,"grow,wrap");
		pnlStatus.add(lbLidarRange);
		pnlStatus.add(mlbLidarRange,"grow,wrap");
		pnlStatus.add(lbTemperature);
		pnlStatus.add(mlbTemperature,"grow,wrap");
		pnlStatus.add(lbPressure);
		pnlStatus.add(mlbPressure,"grow,wrap");
		pnlStatus.add(lbLoopTime);
		pnlStatus.add(mlbLoopTime,"grow,wrap");
		pnlStatus.add(lbSystemTime);
		pnlStatus.add(mlbSystemTime,"grow,wrap");

		this.add(pnlStatus);
	}

	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getStatusWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setStatusWnd(ws);	
	}
}
