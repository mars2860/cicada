package copter;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.net.DatagramPacket;

import com.google.gson.annotations.Expose;
import com.google.gson.annotations.SerializedName;

/** @note all types are double for fast processing in charts */
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
	public static final int DEFAULT_TELEMETRY_PERIOD = 50000;
	
	public static class Vector3
	{
		public double x;
		public double y;
		public double z;
	}
	
	public static class Net implements Cloneable
	{
		@Setting
		@NoChart
		@Expose
		public String ip = DEFAULT_IP;
		@Setting
		@NoChart
		@Expose
		public int cmdPort = DEFAULT_CMD_PORT;
		@Setting
		@NoChart
		@Expose
		public int telemetryPort = DEFAULT_TELEMETRY_PORT;
		@Setting
		@NoChart
		@Expose
		public int videoPort = DEFAULT_VIDEO_PORT;
		@Setting
		@NoChart
		@Expose
		public int telemetryPeriod = DEFAULT_TELEMETRY_PERIOD;
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	public static class Battery implements Cloneable
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
	
	public static class TripleAxisSensor implements Cloneable
	{
		// raw values from sensor
		public double rawX;
		public double rawY;
		public double rawZ;
		// values after digital filter and scaled to real units
		public double pureX;
		public double pureY;
		public double pureZ;
		@NoChart
		@Setting
		@Expose
		public int offsetX;
		@NoChart
		@Setting
		@Expose
		public int offsetY;
		@NoChart
		@Setting
		@Expose
		public int offsetZ;
		
		protected void parse(BinaryParser parser, DatagramPacket packet)
		{
			pureX = parser.getFloat(packet);
			pureY = parser.getFloat(packet);
			pureZ = parser.getFloat(packet);
			
			rawX = parser.getInt16t(packet);
			rawY = parser.getInt16t(packet);
			rawZ = parser.getInt16t(packet);

			offsetX = parser.getInt16t(packet);
			offsetY = parser.getInt16t(packet);
			offsetZ = parser.getInt16t(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
		
		public boolean settingsEquals(TripleAxisSensor s)
		{
			if(	offsetX == s.offsetX &&
				offsetY == s.offsetY &&
				offsetZ == s.offsetZ)
				return true;
				
			return false;
		}
	}
	
	public static class Pid implements Cloneable
	{
		@NoChart
		@Setting
		@Expose
		public boolean enabled;
		@NoChart
		@Setting
		@Expose
		public float kp;
		@NoChart
		@Setting
		@Expose
		public float ki;
		@NoChart
		@Setting
		@Expose
		public float kd;
		@NoChart
		@Setting
		@Expose
		public float maxOut;
		
		/*@NoChart
		@Setting
		@Expose
		not used any more*/
		@NoChart
		public float maxErrSum;

		public double target;
		public double errSum;
		public double input;
		
		public double out;
		
		protected void parse(BinaryParser parser, DatagramPacket packet)
		{
			kp = parser.getFloat(packet);
			ki = parser.getFloat(packet);
			kd = parser.getFloat(packet);
			target = parser.getFloat(packet);
			out = parser.getFloat(packet);
			errSum = parser.getFloat(packet);
			input = parser.getFloat(packet);
			maxOut = parser.getFloat(packet);
			//maxErrSum = parser.getFloat(packet);
			enabled = (parser.getUint32t(packet) > 0)?true:false;
		}
		
		public boolean settingsEquals(Pid pid)
		{
			if(	this.enabled == pid.enabled &&
				this.kp == pid.kp &&
				this.ki == pid.ki &&
				this.kd == pid.kd &&
				this.maxOut == pid.maxOut)
				//this.maxErrSum == pid.maxErrSum 
				return true;
			
			return false;
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	/**@note it class needs only to have target fields in rad and deg to plot its on chart */
	public static class AngularPid extends Pid implements Cloneable
	{
		public double targetDeg;
		
		@Override
		protected void parse(BinaryParser parser, DatagramPacket packet)
		{
			super.parse(parser, packet);
			
			targetDeg = Math.toDegrees(this.target);
		}

		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	/**@note it class needs only to have pure fields in rad and deg to plot its on chart */
	public static class AngularSensor extends TripleAxisSensor implements Cloneable
	{
		public double pureXrad;
		public double pureYrad;
		public double pureZrad;
		public double pureXdeg;
		public double pureYdeg;
		public double pureZdeg;
		
		@Override
		protected void parse(BinaryParser parser, DatagramPacket packet)
		{
			super.parse(parser, packet);
			
			pureXrad = this.pureX;
			pureYrad = this.pureY;
			pureZrad = this.pureZ;
			pureXdeg = Math.toDegrees(this.pureX);
			pureYdeg = Math.toDegrees(this.pureY);
			pureZdeg = Math.toDegrees(this.pureZ);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@Expose
	@SettingGroup(name = "NET")
	@SerializedName("net")
	public Net net = new Net();
	@NoChart
	public double timestamp;
	public Battery battery = new Battery();
	public double wifiLevel;
	@SettingGroup(name = "ACCEL")
	@Expose
	@SerializedName("accel")
	public TripleAxisSensor accel = new TripleAxisSensor();
	@SettingGroup(name = "GYRO")
	@Expose
	@SerializedName("gyro")
	public AngularSensor gyro = new AngularSensor();
	@SettingGroup(name = "MAGNETO")
	@Expose
	@SerializedName("magneto")
	public TripleAxisSensor magneto = new TripleAxisSensor();
	public double yawRad;
	public double yawDeg;
	public double pitchRad;
	public double pitchDeg;
	public double rollRad;
	public double rollDeg;
	public double headingRad;
	public double headingDeg;
	public double altitude;
	public double velocityX;
	public double velocityY;
	public double velocityZ;
	@SettingGroup(name = "YAW_RATE_PID")
	@Expose
	@SerializedName("yawRatePid")
	public AngularPid yawRatePid = new AngularPid();
	@SettingGroup(name = "PITCH_RATE_PID")
	@Expose
	@SerializedName("pitchRatePid")
	public AngularPid pitchRatePid = new AngularPid();
	@SettingGroup(name = "ROLL_RATE_PID")
	@Expose
	@SerializedName("rollRatePid")
	public AngularPid rollRatePid = new AngularPid();
	@SettingGroup(name = "PITCH_PID")
	@Expose
	@SerializedName("pitchPid")
	public AngularPid pitchPid = new AngularPid();
	@SettingGroup(name = "ROLL_PID")
	@Expose
	@SerializedName("rollPid")
	public AngularPid rollPid = new AngularPid();
	@SettingGroup(name = "ALT_PID")
	@Expose
	@SerializedName("altPid")
	public Pid altPid = new Pid();
	@SettingGroup(name = "VELOCITY_X_PID")
	@Expose
	@SerializedName("velocityXPid")
	public Pid velocityXPid = new Pid();
	@SettingGroup(name = "VELOCITY_Y_PID")
	@Expose
	@SerializedName("velocityYPid")
	public Pid velocityYPid = new Pid();
	@SettingGroup(name = "VELOCITY_Z_PID")
	@Expose
	@SerializedName("velocityZPid")
	public Pid velocityZPid = new Pid();
	@NoChart
	public boolean motorsEnabled;
	public double baseGas;
	public double motorGas0;
	public double motorGas1;
	public double motorGas2;
	public double motorGas3;
	@NoChart
	public boolean stabilizationEnabled;
	public double avgLoopTime;
	public double maxLoopTime;
	public double temperature;
	public double lidarRange;
	
	public OpticalFlow opticalFlow = new OpticalFlow();
	
	@SettingGroup(name = "BARO")
	@Expose
	@SerializedName("baro")
	public Baro baro = new Baro();
	
	public double holdPosEnabled;
	
	@NoChart
	public int version;
	
	@Override
	public Object clone() throws CloneNotSupportedException
	{
		DroneState state = (DroneState)super.clone();
		state.net = (Net)net.clone();
		state.battery = (Battery)battery.clone();
		state.accel = (TripleAxisSensor)accel.clone();
		state.gyro = (AngularSensor)gyro.clone();
		state.magneto = (TripleAxisSensor)magneto.clone();
		state.yawRatePid = (AngularPid)yawRatePid.clone();
		state.pitchPid = (AngularPid)pitchPid.clone();
		state.rollPid = (AngularPid)rollPid.clone();
		state.altPid = (Pid)altPid.clone();
		return state;
	}
	
	public void parse(BinaryParser parser, DatagramPacket packet)
	{
		version = parser.getUint8t(packet);
		motorsEnabled = parser.getBool(packet);
		stabilizationEnabled = parser.getBool(packet);
		holdPosEnabled = parser.getUint8t(packet);
		baseGas = parser.getInt32t(packet);
		motorGas0 = parser.getInt32t(packet);
		motorGas1 = parser.getInt32t(packet);
		motorGas2 = parser.getInt32t(packet);
		motorGas3 = parser.getInt32t(packet);
		timestamp = parser.getUint32t(packet);
		battery.parse(parser, packet);
		wifiLevel = parser.getInt32t(packet);
		accel.parse(parser, packet);
		gyro.parse(parser, packet);
		magneto.parse(parser, packet);
		yawRad = parser.getFloat(packet);
		yawDeg = Math.toDegrees(yawRad);
		pitchRad = parser.getFloat(packet);
		pitchDeg = Math.toDegrees(pitchRad);
		rollRad = parser.getFloat(packet);
		rollDeg = Math.toDegrees(rollRad);
		headingRad = parser.getFloat(packet);
		headingDeg = Math.toDegrees(headingRad);
		altitude = parser.getFloat(packet);
		velocityX = parser.getFloat(packet);
		velocityY = parser.getFloat(packet);
		velocityZ = parser.getFloat(packet);
		avgLoopTime = parser.getInt32t(packet);
		maxLoopTime = parser.getInt32t(packet);
		temperature = parser.getFloat(packet);
		lidarRange = parser.getFloat(packet);
		baro.parse(parser, packet);;
		opticalFlow.parse(parser, packet);;
		yawRatePid.parse(parser, packet);
		pitchRatePid.parse(parser, packet);
		rollRatePid.parse(parser, packet);
		pitchPid.parse(parser, packet);
		rollPid.parse(parser, packet);
		altPid.parse(parser, packet);
		velocityXPid.parse(parser, packet);
		velocityYPid.parse(parser, packet);
		velocityZPid.parse(parser, packet);
		net.telemetryPeriod = (int)parser.getUint32t(packet);
	}
	
	public static class OpticalFlow implements Cloneable
	{
		public double rawX;
		public double rawY;
		
		private void parse(BinaryParser parser, DatagramPacket packet)
		{
			rawX = parser.getInt16t(packet);
			rawY = parser.getInt16t(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	public static class Baro implements Cloneable
	{
		public double pressure;
		public double altitude;
		@NoChart
		@Expose
		@Setting
		public double seaLevelPressure;
		
		private void parse(BinaryParser parser, DatagramPacket packet)
		{
			pressure = parser.getFloat(packet);
			altitude = parser.getFloat(packet);
			seaLevelPressure = parser.getFloat(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
}
