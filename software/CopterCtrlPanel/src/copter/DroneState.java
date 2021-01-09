package copter;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.net.DatagramPacket;

/** @note all types are double for fast processing charts */
public class DroneState implements Cloneable
{	
	@Retention(RetentionPolicy.RUNTIME)
	@Target(ElementType.FIELD)
	public @interface NoChart {}
	
	@Retention(RetentionPolicy.RUNTIME)
	@Target(ElementType.FIELD)
	public @interface Setting {}
	
	@Retention(RetentionPolicy.RUNTIME)
	@Target(ElementType.FIELD)
	public @interface SettingGroup
	{
		public String name();
	}
	
	public static final String DEFAULT_IP = "192.168.1.33";
	public static final int DEFAULT_CMD_PORT = 4210;
	public static final int DEFAULT_TELEMETRY_PORT = 4211;
	public static final int DEFAULT_VIDEO_PORT = 4212;
	
	public class Vector3
	{
		public double x;
		public double y;
		public double z;
	}
	
	public class Net
	{
		@Setting
		@NoChart
		public String ip = DEFAULT_IP;
		@Setting
		@NoChart
		public int cmdPort = DEFAULT_CMD_PORT;
		@Setting
		@NoChart
		public int telemetryPort = DEFAULT_TELEMETRY_PORT;
		@Setting
		@NoChart
		public int videoPort = DEFAULT_VIDEO_PORT;
	}
	
	public class Battery implements Cloneable
	{
		public double voltage;
		public double percent;
		
		private void parse(BinaryParser parser, DatagramPacket packet)
		{
			voltage = parser.getFloat(packet);
			percent = parser.getFloat(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	public class TripleAxisSensor implements Cloneable
	{
		// raw values from sensor
		public double rawX;
		public double rawY;
		public double rawZ;
		// values after digital filter
		public double filteredX;
		public double filteredY;
		public double filteredZ;
		// values after digital filter and scaled to real units
		public double pureX;
		public double pureY;
		public double pureZ;
		@NoChart
		@Setting
		public double offsetX;
		@NoChart
		@Setting
		public double offsetY;
		@NoChart
		@Setting
		public double offsetZ;
		// no sense variable, this is 2 bytes gap in C structure
		protected int gap;
		
		private void parse(BinaryParser parser, DatagramPacket packet)
		{
			pureX = parser.getFloat(packet);
			pureY = parser.getFloat(packet);
			pureZ = parser.getFloat(packet);
			
			rawX = parser.getInt16t(packet);
			rawY = parser.getInt16t(packet);
			rawZ = parser.getInt16t(packet);
			
			filteredX = parser.getInt16t(packet);
			filteredY = parser.getInt16t(packet);
			filteredZ = parser.getInt16t(packet);

			offsetX = parser.getInt16t(packet);
			offsetY = parser.getInt16t(packet);
			offsetZ = parser.getInt16t(packet);
			
			// read gap
			gap = parser.getInt16t(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	public class Pid implements Cloneable
	{
		@NoChart
		@Setting
		public boolean enabled;
		@NoChart
		@Setting
		public double kp;
		@NoChart
		@Setting
		public double ki;
		@NoChart
		@Setting
		public double kd;
		@NoChart
		@Setting
		public double target;
		@NoChart
		public double errSum;
		@NoChart
		public double prevErr;
		
		public double out;
		
		private void parse(BinaryParser parser, DatagramPacket packet)
		{
			kp = parser.getFloat(packet);
			ki = parser.getFloat(packet);
			kd = parser.getFloat(packet);
			target = parser.getFloat(packet);
			out = parser.getFloat(packet);
			errSum = parser.getFloat(packet);
			prevErr = parser.getFloat(packet);
			enabled = (parser.getUint32t(packet) > 0)?true:false;
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@NoChart
	public double timestamp;
	public Battery battery = new Battery();
	public double wifiLevel;
	@SettingGroup(name = "ACCEL")
	public TripleAxisSensor accel = new TripleAxisSensor();
	@SettingGroup(name = "GYRO")
	public TripleAxisSensor gyroRad = new TripleAxisSensor();
	public TripleAxisSensor gyroDeg = new TripleAxisSensor();
	@SettingGroup(name = "MAGNETO")
	public TripleAxisSensor magneto = new TripleAxisSensor();
	public double yawRad;
	public double yawDeg;
	public double pitchRad;
	public double pitchDeg;
	public double rollRad;
	public double rollDeg;
	public double headingRad;
	public double headingDeg;
	@SettingGroup(name = "YAW_RATE_PID")
	public Pid yawRadRatePid = new Pid();
	public Pid yawDegRatePid = new Pid();
	@SettingGroup(name = "PITCH_PID")
	public Pid pitchRadPid = new Pid();
	public Pid pitchDegPid = new Pid();
	@SettingGroup(name = "ROLL_PID")
	public Pid rollRadPid = new Pid();
	public Pid rollDegPid = new Pid();
	@SettingGroup(name = "ALT_PID")
	public Pid altPid = new Pid();
	@NoChart
	public boolean motorsEnabled;
	public double baseGas;
	public double motorGas0;
	public double motorGas1;
	public double motorGas2;
	public double motorGas3;
	@NoChart
	public boolean stabilizationEnabled;
	public double mainLoopTime;
	public double temperature;
	public double pressure;
	public double altitude;
	@NoChart
	public double seaLevel;
	// no sense variable, this is 2 bytes gap in C structure
	protected int gap;
	
	@Override
	public Object clone() throws CloneNotSupportedException
	{
		DroneState state = (DroneState)super.clone();
		state.battery = (Battery)battery.clone();
		state.accel = (TripleAxisSensor)accel.clone();
		state.gyroRad = (TripleAxisSensor)gyroRad.clone();
		state.magneto = (TripleAxisSensor)magneto.clone();
		state.yawRadRatePid = (Pid)yawRadRatePid.clone();
		state.pitchRadPid = (Pid)pitchRadPid.clone();
		state.rollRadPid = (Pid)rollRadPid.clone();
		state.altPid = (Pid)altPid.clone();
		return state;
	}
	
	public void parse(BinaryParser parser, DatagramPacket packet)
	{
		timestamp = parser.getUint32t(packet);
		battery.parse(parser, packet);
		wifiLevel = parser.getInt32t(packet);
		accel.parse(parser, packet);
		gyroRad.parse(parser, packet);
		magneto.parse(parser, packet);
		yawRad = parser.getFloat(packet);
		yawDeg = Math.toDegrees(yawRad);
		pitchRad = parser.getFloat(packet);
		pitchDeg = Math.toDegrees(pitchRad);
		rollRad = parser.getFloat(packet);
		rollDeg = Math.toDegrees(rollRad);
		headingRad = parser.getFloat(packet);
		headingDeg = Math.toDegrees(headingRad);
		mainLoopTime = parser.getInt32t(packet);
		temperature = parser.getFloat(packet);
		pressure = parser.getFloat(packet);
		altitude = parser.getFloat(packet);
		seaLevel = parser.getFloat(packet);
		yawRadRatePid.parse(parser, packet);
		pitchRadPid.parse(parser, packet);
		rollRadPid.parse(parser, packet);
		altPid.parse(parser, packet);
		baseGas = parser.getInt32t(packet);
		motorGas0 = parser.getInt32t(packet);
		motorGas1 = parser.getInt32t(packet);
		motorGas2 = parser.getInt32t(packet);
		motorGas3 = parser.getInt32t(packet);
		motorsEnabled = parser.getBool(packet);
		stabilizationEnabled = parser.getBool(packet);
		gap = parser.getInt16t(packet);
		
		try
		{
			gyroDeg = (TripleAxisSensor)gyroRad.clone();
			gyroDeg.pureX = Math.toDegrees(gyroRad.pureX);
			gyroDeg.pureY = Math.toDegrees(gyroRad.pureY);
			gyroDeg.pureZ = Math.toDegrees(gyroRad.pureZ);
			
			yawDegRatePid = (Pid)yawRadRatePid.clone();
			yawDegRatePid.target = Math.toDegrees(yawRadRatePid.target);
			
			pitchDegPid = (Pid)pitchRadPid.clone();
			pitchDegPid.target = Math.toDegrees(pitchRadPid.target);
			
			rollDegPid = (Pid)rollRadPid.clone();
			rollDegPid.target = Math.toDegrees(rollRadPid.target);
		}
		catch(CloneNotSupportedException e)
		{
			e.printStackTrace();
		}
	}
}
