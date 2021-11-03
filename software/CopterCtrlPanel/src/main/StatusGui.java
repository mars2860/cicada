package main;

import java.text.DecimalFormat;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JLabel;
import javax.swing.JPanel;

import copter.CopterTelemetry;
import copter.DroneState;
import main.Settings.WndState;
import net.miginfocom.swing.MigLayout;

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
	private JLabel mlbHeading;
	private JLabel mlbLoopTime;
	private JLabel mlbTemperature;
	private JLabel mlbPressure;
	private JLabel mlbAltitude;
	private JLabel mlbLidarRange;
	private JLabel mlbOpticalFlowX;
	private JLabel mlbOpticalFlowY;

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
			
			DroneState droneState = CopterTelemetry.instance().getDroneState();
			
			DecimalFormat fmt1 = new DecimalFormat();
			fmt1.setMaximumFractionDigits(2);
			fmt1.setMinimumFractionDigits(0);
			fmt1.setGroupingUsed(false);
			DecimalFormat fmt2 = new DecimalFormat();
			fmt2.setMaximumFractionDigits(0);
			fmt2.setGroupingUsed(false);
			DecimalFormat fmt3 = new DecimalFormat();
			fmt2.setMaximumFractionDigits(3);
			fmt2.setGroupingUsed(false);
			
			String batState = fmt1.format(droneState.battery.voltage) + "V/" + 
								fmt1.format(droneState.battery.percent) + "%";
			
			mlbBattery.setText(batState);
			mlbWifiLevel.setText(Integer.toString((int)droneState.wifiLevel));
			
			double yaw = droneState.yawDeg;
			double yawTarget = droneState.yawRatePid.targetDeg;
			double pitch = droneState.pitchDeg;
			double pitchTarget = droneState.pitchPid.targetDeg;
			double roll = droneState.rollDeg;
			double rollTarget = droneState.rollPid.targetDeg;
			double heading = droneState.headingDeg;
			
			mlbYawValue.setText(fmt1.format(yaw) + "/" + fmt1.format(yawTarget));
			mlbPitchValue.setText(fmt1.format(pitch) + "/" + fmt1.format(pitchTarget));
			mlbRollValue.setText(fmt1.format(roll) + "/" + fmt1.format(rollTarget));
			mlbHeading.setText(fmt2.format(heading));
			mlbLoopTime.setText(fmt2.format(droneState.avgLoopTime));
			mlbTemperature.setText(fmt1.format(droneState.temperature));
			mlbPressure.setText(fmt1.format(droneState.baro.pressure) + "/" +
								fmt1.format(droneState.baro.seaLevelPressure));
			mlbAltitude.setText(fmt3.format(droneState.altitude) + "/" +
								fmt3.format(droneState.altPid.target));
			mlbLidarRange.setText(fmt3.format(droneState.lidarRange));
			mlbOpticalFlowX.setText(fmt2.format(droneState.opticalFlow.rawX));
			mlbOpticalFlowY.setText(fmt2.format(droneState.opticalFlow.rawY));
			
			mlbYaw.setIcon(rotateImageIcon(ResBox.icon("YAW"), -yaw));
			mlbPitch.setIcon(rotateImageIcon(ResBox.icon("PITCH"), pitch));
			mlbRoll.setIcon(rotateImageIcon(ResBox.icon("ROLL"), roll));
		}
	}
	
	public StatusGui()
	{
		super("Status",180,330);
		this.setTitle(ResBox.text("STATUS"));
		this.setIconImage(ResBox.icon("GAUGE").getImage());
		this.createUI();
		CopterTelemetry.instance().addObserver(new OnTelemetryUpdate());
	}
	
	private void createUI()
	{
		JPanel pnlStatus = new JPanel(new MigLayout("","[][grow]",""));

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
		
		mlbHeading = new JLabel();
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
		mlbOpticalFlowX = new JLabel();
		mlbOpticalFlowY = new JLabel();
		
		JLabel lbBattery = new JLabel(ResBox.icon("BATTERY"));
		JLabel lbWifiLevel = new JLabel(ResBox.icon("WIFI"));
		JLabel lbHeading = new JLabel(ResBox.icon("HEADING"));
		JLabel lbAltitude = new JLabel(ResBox.icon("ALTITUDE"));
		JLabel lbTemperature = new JLabel(ResBox.icon("TEMPERATURE"));
		JLabel lbPressure = new JLabel(ResBox.icon("PRESSURE"));
		JLabel lbLoopTime = new JLabel(ResBox.icon("CPUTIME"));
		JLabel lbLidarRange = new JLabel(ResBox.icon("LIDAR"));
		JLabel lbOpticalFlowX = new JLabel(ResBox.icon("OPTICAL_FLOW_X"));
		JLabel lbOpticalFlowY = new JLabel(ResBox.icon("OPTICAL_FLOW_Y"));
		
		lbBattery.setToolTipText(ResBox.text("BATTERY_STATE"));
		lbWifiLevel.setToolTipText(ResBox.text("WIFI_STATE"));
		mlbYaw.setToolTipText(ResBox.text("YAW_STATE"));
		mlbPitch.setToolTipText(ResBox.text("PITCH_STATE"));
		mlbRoll.setToolTipText(ResBox.text("ROLL_STATE"));
		lbHeading.setToolTipText(ResBox.text("HEADING_STATE"));
		lbAltitude.setToolTipText(ResBox.text("ALTITUDE_STATE"));
		lbTemperature.setToolTipText(ResBox.text("TEMPERATURE_STATE"));
		lbPressure.setToolTipText(ResBox.text("PRESSURE_STATE"));
		lbLoopTime.setToolTipText(ResBox.text("LOOP_TIME_STATE"));
		lbLidarRange.setToolTipText(ResBox.text("LIDAR_RANGE"));
		lbOpticalFlowX.setToolTipText(ResBox.text("OPTICAL_FLOW_X"));
		lbOpticalFlowY.setToolTipText(ResBox.text("OPTICAL_FLOW_Y"));
		
		pnlStatus.add(lbBattery);
		pnlStatus.add(mlbBattery,"grow,wrap");
		pnlStatus.add(lbWifiLevel);
		pnlStatus.add(mlbWifiLevel,"grow,wrap");
		pnlStatus.add(mlbYaw);
		pnlStatus.add(mlbYawValue,"grow,wrap");
		pnlStatus.add(mlbPitch);
		pnlStatus.add(mlbPitchValue,"grow,wrap");
		pnlStatus.add(mlbRoll);
		pnlStatus.add(mlbRollValue,"grow,wrap");
		pnlStatus.add(lbHeading);
		pnlStatus.add(mlbHeading,"grow,wrap");
		pnlStatus.add(lbAltitude);
		pnlStatus.add(mlbAltitude,"grow,wrap");
		pnlStatus.add(lbLidarRange);
		pnlStatus.add(mlbLidarRange,"grow,wrap");
		pnlStatus.add(lbTemperature);
		pnlStatus.add(mlbTemperature,"grow,wrap");
		pnlStatus.add(lbPressure);
		pnlStatus.add(mlbPressure,"grow,wrap");
		pnlStatus.add(lbOpticalFlowX);
		pnlStatus.add(mlbOpticalFlowX,"grow,wrap");
		pnlStatus.add(lbOpticalFlowY);
		pnlStatus.add(mlbOpticalFlowY,"grow,wrap");
		pnlStatus.add(lbLoopTime);
		pnlStatus.add(mlbLoopTime,"grow,wrap");

		this.add(pnlStatus);
	}

	@Override
	protected WndState loadWndState()
	{
		return Settings.instance().getStatusWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		Settings.instance().setStatusWnd(ws);	
	}
}
