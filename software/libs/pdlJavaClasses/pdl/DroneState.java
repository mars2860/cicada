package pdl;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import com.google.gson.annotations.Expose;
import com.google.gson.annotations.SerializedName;

/** This class represents current drone state and drone settings simultaneously,
 *  also it represents some ground station app settings as Widgets,
 *  Sounds, RemoteControl and other settings. We have some ground app settings
 *  inside this class because it is comfortable to build settings GUI by
 *  java reflection. These settings is stored in the static fields of this class.
 * @note all types are double for fast processing in charts */
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
	
	/// Supported PDL library version
	public static final int SUPPORTED_PDL_VERSION = 3;
	
	public static final String DEFAULT_SSID = "liftelectronica";
	public static final String DEFAULT_PSK = "cosmos327";
	public static final String DEFAULT_IP = "192.168.1.33";
	public static final String DEFAULT_GATEWAY = "192.168.1.1";
	public static final String DEFAULT_SUBNET = "255.255.255.0";
	public static final int DEFAULT_UDP_PORT = 4210;
	//public static final int DEFAULT_CMD_PORT = 4210;
	//public static final int DEFAULT_TELEMETRY_PORT = 4211;
	//public static final int DEFAULT_LOG_PORT = 4212;
	public static final int DEFAULT_TELEMETRY_PERIOD = 50000;
	public static final int DEFAULT_RSSI_ALARM_LEVEL = -82;
	public static final boolean DEFAULT_WIFI_STA_MODE = false;
	public static final int DEFAULT_WIFI_CHANNEL = 7;
	public static final float DEFAULT_WIFI_TX_POWER_DBM = 20;
	public static final int DEFAULT_WIFI_PHY = 2;
	public static final int DEFAULT_WIFI_RATE = 0x0B;
	public static final boolean DEFAULT_WIFI_BROADCAST_ENABLED = false;
	 
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
		public String ssid = DEFAULT_SSID;
		@Setting
		@NoChart
		@Expose
		public String psk = DEFAULT_PSK;
		@Setting
		@NoChart
		@Expose
		public String gateway = DEFAULT_GATEWAY;
		@Setting
		@NoChart
		@Expose
		public String subnet = DEFAULT_SUBNET;
		@Setting
		@NoChart
		@Expose
		public String ip = DEFAULT_IP;
		@Setting
		@NoChart
		@Expose
		public boolean dynamicIp = false;
		@Setting
		@NoChart
		@Expose
		public int udpPort = DEFAULT_UDP_PORT;
		@Setting
		@NoChart
		@Expose
		public int droneId = 0;
		/* DEPRECATED
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
		public int logPort = DEFAULT_LOG_PORT;
		*/
		@Setting
		@NoChart
		@Expose
		public int rssiAlarmLevel = DEFAULT_RSSI_ALARM_LEVEL;
		@Setting
		@NoChart
		@Expose
		public boolean wifiStaMode = DEFAULT_WIFI_STA_MODE;
		@Setting
		@NoChart
		@Expose
		public int wifiChannel = DEFAULT_WIFI_CHANNEL;
		@Setting
		@NoChart
		@Expose
		public float wifiTxPowerDbm = DEFAULT_WIFI_TX_POWER_DBM;
		@Setting
		@NoChart
		@Expose
		public int wifiPhy = DEFAULT_WIFI_PHY;
		@Setting
		@NoChart
		@Expose
		public int wifiRate = DEFAULT_WIFI_RATE;
		@Setting
		@NoChart
		@Expose
		public boolean wifiBroadcastEnabled = DEFAULT_WIFI_BROADCAST_ENABLED;
		@Setting
		@NoChart
		@Expose
		public String wifiBroadcastModemComPort = "";
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@Expose
	@SettingGroup(name = "NET")
	@SerializedName("net")
	public static Net net = new Net();
	
	public static class Telemetry implements Cloneable
	{
		@Setting
		@NoChart
		@Expose
		public int period = DEFAULT_TELEMETRY_PERIOD;
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@Expose
	@SettingGroup(name = "TELEMETRY")
	@SerializedName("telemetry")
	public Telemetry telemetry = new Telemetry();
	
	public static class Misc implements Cloneable
	{		
		@Setting
		@NoChart
		@Expose
		public int blackBoxSize = 64000;
		
		@Setting
		@NoChart
		@Expose
		public int maxCtrlCmdsInQueue = 3;
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@Expose
	@SettingGroup(name = "MISC")
	@SerializedName("misc")
	public static Misc misc = new Misc();
	
	public static class Widgets implements Cloneable
	{
		@Setting
		@NoChart
		@Expose
		public int maxRowCount = 4;

		@Setting
		@NoChart
		@Expose
		public boolean battery = true;

		@Setting
		@NoChart
		@Expose
		public boolean flyTime = true;

		@Setting
		@NoChart
		@Expose
		public boolean rssi = true;

		@Setting
		@NoChart
		@Expose
		public boolean yaw = false;
		
		@Setting
		@NoChart
		@Expose
		public boolean pitch = false;
		
		@Setting
		@NoChart
		@Expose
		public boolean roll = false;
		
		@Setting
		@NoChart
		@Expose
		public boolean alt = true;
		
		@Setting
		@NoChart
		@Expose
		public boolean home = false;
		
		@Setting
		@NoChart
		@Expose
		public boolean vertSpeed = false;
		
		@Setting
		@NoChart
		@Expose
		public boolean horSpeed = false;

		@Setting
		@NoChart
		@Expose
		public boolean temperature = false;

		@Setting
		@NoChart
		@Expose
		public boolean pressure = false;

		@Setting
		@NoChart
		@Expose
		public boolean gx = false;

		@Setting
		@NoChart
		@Expose
		public boolean gy = false;

		@Setting
		@NoChart
		@Expose
		public boolean gz = false;

		@Setting
		@NoChart
		@Expose
		public boolean ax = false;

		@Setting
		@NoChart
		@Expose
		public boolean ay = false;

		@Setting
		@NoChart
		@Expose
		public boolean az = false;
		
		@Setting
		@NoChart
		@Expose
		public boolean lat = false;
		
		@Setting
		@NoChart
		@Expose
		public boolean lon = false;
		
		@Setting
		@NoChart
		@Expose
		public boolean sattels = false;

		@Setting
		@NoChart
		@Expose
		public boolean latency = false;

		@Setting
		@NoChart
		@Expose
		public boolean lostRxPackets = false;
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@Expose
	@SettingGroup(name = "WIDGETS")
	@SerializedName("widgets")
	public static Widgets widgets = new Widgets();
	
	public static class Sounds implements Cloneable
	{
		@Setting
		@NoChart
		@Expose
		public boolean enabled = true;

		@Setting
		@NoChart
		@Expose
		public boolean altitude = true;

		@Setting
		@NoChart
		@Expose
		public boolean battery = true;

		@Setting
		@NoChart
		@Expose
		public boolean radio = true;
		
		@Setting
		@NoChart
		@Expose
		public boolean system = true;
			
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@Expose
	@SettingGroup(name = "SOUNDS")
	@SerializedName("sounds")
	public static Sounds sounds = new Sounds();
	
	public static class Motors implements Cloneable
	{
		public static final int ESC_DSHOT300 = 0;
		public static final int ESC_PWM_40KHZ_125 = 1;
		public static final int ESC_PWM_20KHZ_250 = 2;
		public static final int ESC_PWM_10KHZ_500 = 3;
		public static final int ESC_PWM_2K5HZ_2000 = 4;
		
		public static final int FRAME_QUAD_X = 0;
		public static final int FRAME_QUAD_CROSS = 1;
		
		@Setting
		@NoChart
		@Expose
		public static int count = 4;// it is static because we don't need to store this value in blackbox
		@Setting
		@NoChart
		@Expose
		public static int maxGas = 1999;// it is static because we don't need to store this value in blackbox
		@Setting
		@NoChart
		@Expose
		public static int nullGas = 0;// it is static because we don't need to store this value in blackbox
		@Setting
		@NoChart
		@Expose
		public static int minGas = 0;// it is static because we don't need to store this value in blackbox
		@Setting
		@NoChart
		@Expose
		public static int antiTurtleGas = 0;// it is static because we don't need to store this value in blackbox
		@Expose
		@Setting
		@NoChart
		public int esc;
		@Expose
		@Setting
		@NoChart
		public int frame;
		
		public Motors()
		{
			esc = ESC_DSHOT300;
			frame = FRAME_QUAD_X;
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@Expose
	@SettingGroup(name = "MOTORS")
	@SerializedName("motors")
	public Motors motors = new Motors();

	public static class RemoteCtrl implements Cloneable
	{
		public static enum RotateBy
		{
			ROTATE_BY_YAW_RATE,
			ROTATE_BY_HEADING,
			ROTATE_BY_Z_RATE
		}

		public static enum MoveBy
		{
			MOVE_BY_PITCH_ROLL_RATE,
			MOVE_BY_PITCH_ROLL,
			MOVE_BY_VELOCITY,
			MOVE_BY_X_Y_RATE
		}

		public static enum LiftBy
		{
			LIFT_BY_GAS_DELTA,
			LIFT_BY_VELOCITY,
			LIFT_BY_ALT,
			LIFT_BY_GAS_FORSAGE,
			LIFT_BY_GAS
		}

		@Expose
		@Setting
		@NoChart
		public RotateBy rotateBy;
		@Expose
		@Setting
		@NoChart
		public float rotateDelta;
		@Expose
		@Setting
		@NoChart
		public MoveBy moveBy;
		@Expose
		@Setting
		@NoChart
		public float moveDelta;
		@Expose
		@Setting
		@NoChart
		public float trickDelta;
		@Expose
		@Setting
		@NoChart
		public LiftBy liftBy;
		@Expose
		@Setting
		@NoChart
		public float liftDelta;
		@Expose
		@Setting
		@NoChart
		public int maxGas;
		@Expose
		@Setting
		@NoChart
		public int middleGas;
		@Setting
		@NoChart
		@Expose
		public int maxMiddleGas;
		@Expose
		@Setting
		@NoChart
		public int minGas;
		@Expose
		@Setting
		@NoChart
		public float liftAccelTime;
		@Expose
		@Setting
		@NoChart
		public float moveAccelTime;
		@Expose
		@Setting
		@NoChart
		public float rotateAccelTime;
		@Setting
		@NoChart
		@Expose
		public boolean virtualGamepad;
		@Expose
		@Setting
		@NoChart
		public boolean leftJoystickAutoCenter;
		@Expose
		@Setting
		@NoChart
		public boolean rightJoystickAutoCenter;
		@Setting
		@NoChart
		@Expose
		public boolean pitchAutoLevel;
		@Setting
		@NoChart
		@Expose
		public boolean rollAutoLevel;

		public RemoteCtrl()
		{
			rotateBy = RotateBy.ROTATE_BY_YAW_RATE;
			rotateDelta = 80.f;
			trickDelta = 120.f;
			moveBy = MoveBy.MOVE_BY_PITCH_ROLL;
			moveDelta = 5.f;
			liftBy = LiftBy.LIFT_BY_VELOCITY;
			liftDelta = 0.4f;
			maxGas = 1700;
			middleGas = 1380;
			minGas = 0;
			liftAccelTime = 1.0f;
			moveAccelTime = 0.2f;
			rotateAccelTime = 0.3f;
			virtualGamepad = true;
			leftJoystickAutoCenter = true;
			rightJoystickAutoCenter = true;
			pitchAutoLevel = true;
			rollAutoLevel = true;
		}

		@Override
		protected Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}

	@Expose
	@SettingGroup(name = "REMOTE_CONTROL")
	@SerializedName("rc")
	public static RemoteCtrl rc = new RemoteCtrl();

	public static class Battery implements Cloneable
	{
		public double voltage;
		public double percent;
		public double current;
		public double capacity;
		@Setting
		@NoChart
		@Expose
		public float voltScaler;
		@Setting
		@NoChart
		@Expose
		public float curnScaler;
		@Setting
		@NoChart
		@Expose
		public float minVoltage;
		@Setting
		@NoChart
		@Expose
		public float maxVoltage;
		
		private void parse(BinaryParser parser, byte[] packet)
		{
			voltage = parser.getFloat(packet);
			percent = parser.getFloat(packet);
			current = parser.getFloat(packet);
			capacity = parser.getFloat(packet);
			voltScaler = parser.getFloat(packet);
			curnScaler = parser.getFloat(packet);
			minVoltage = parser.getFloat(packet);
			maxVoltage = parser.getFloat(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@Expose
	@SettingGroup(name = "BATTERY")
	@SerializedName("battery")
	public Battery battery = new Battery();
	
	public static abstract class TripleAxisSensor implements Cloneable
	{
		public double rawX;
		public double rawY;
		public double rawZ;

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
		
		protected abstract void parse(BinaryParser parser, byte[] packet);
		
		public boolean settingsEquals(TripleAxisSensor s)
		{
			if(	offsetX == s.offsetX &&
				offsetY == s.offsetY &&
				offsetZ == s.offsetZ)
				return true;
				
			return false;
		}
	}
	
	public static class Gyro extends TripleAxisSensor 
	{
		public double pureXdeg;
		public double pureYdeg;
		public double pureZdeg;
		
		public double yawRateRad;
		public double yawRateDeg;
		
		public double pitchRateRad;
		public double pitchRateDeg;
		
		public double rollRateRad;
		public double rollRateDeg;
		
		@NoChart
		@Setting
		@Expose
		public int dlpf;
		
		@Override
		protected void parse(BinaryParser parser, byte[] packet)
		{
			dlpf = (int)parser.getUint32t(packet);
			
			pureX = parser.getFloat(packet);
			pureY = parser.getFloat(packet);
			pureZ = parser.getFloat(packet);
			
			rollRateRad = parser.getFloat(packet);
			pitchRateRad = parser.getFloat(packet);
			yawRateRad = parser.getFloat(packet);
			
			rawX = parser.getInt16t(packet);
			rawY = parser.getInt16t(packet);
			rawZ = parser.getInt16t(packet);

			offsetX = parser.getInt16t(packet);
			offsetY = parser.getInt16t(packet);
			offsetZ = parser.getInt16t(packet);
			
			pureXdeg = Math.toDegrees(this.pureX);
			pureYdeg = Math.toDegrees(this.pureY);
			pureZdeg = Math.toDegrees(this.pureZ);
			
			rollRateDeg = Math.toDegrees(this.rollRateRad);
			pitchRateDeg = Math.toDegrees(this.pitchRateRad);
			yawRateDeg = Math.toDegrees(this.yawRateRad);
		}
		
		public boolean settingsEquals(TripleAxisSensor s)
		{
			if(s instanceof Gyro)
			{
				Gyro gyro = (Gyro)s;
				
				if(	offsetX == gyro.offsetX &&
					offsetY == gyro.offsetY &&
					offsetZ == gyro.offsetZ &&
					dlpf == gyro.dlpf )
				{
					return true;
				}
			}
				
			return false;
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@SettingGroup(name = "GYRO")
	@Expose
	@SerializedName("gyro")
	public Gyro gyro = new Gyro();
	
	public static class Accel extends TripleAxisSensor
	{
		public double north;
		public double east;
		public double up;
		public double pitchRad;
		public double rollRad;
		public double pitchDeg;
		public double rollDeg;
		
		@NoChart
		@Setting
		@Expose
		public int dlpf;
		
		protected void parse(BinaryParser parser, byte[] packet)
		{
			dlpf = (int)parser.getUint32t(packet);
			
			pureX = parser.getFloat(packet);
			pureY = parser.getFloat(packet);
			pureZ = parser.getFloat(packet);
			
			north = parser.getFloat(packet);
			east = parser.getFloat(packet);
			up = parser.getFloat(packet);
			
			pitchRad = parser.getFloat(packet);
			rollRad = parser.getFloat(packet);
			
			pitchDeg = Math.toDegrees(pitchRad);
			rollDeg = Math.toDegrees(rollRad);

			rawX = parser.getInt16t(packet);
			rawY = parser.getInt16t(packet);
			rawZ = parser.getInt16t(packet);

			offsetX = parser.getInt16t(packet);
			offsetY = parser.getInt16t(packet);
			offsetZ = parser.getInt16t(packet);
		}
		
		public boolean settingsEquals(TripleAxisSensor s)
		{
			if(s instanceof Accel)
			{
				Accel acc = (Accel)s;
				
				if(	offsetX == acc.offsetX &&
					offsetY == acc.offsetY &&
					offsetZ == acc.offsetZ &&
					dlpf == acc.dlpf )
				{
					return true;
				}
			}
				
			return false;
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	@SettingGroup(name = "ACCEL")
	@Expose
	@SerializedName("accel")
	public Accel accel = new Accel();
	
	public static class Magneto extends TripleAxisSensor
	{
		@NoChart
		@Setting
		@Expose
		public float scaleX = 1.f;
		@NoChart
		@Setting
		@Expose
		public float scaleY = 1.f;
		@NoChart
		@Setting
		@Expose
		public float scaleZ = 1.f;
		
		@NoChart
		@Setting
		@Expose
		public float declination;
		
		@NoChart
		@Setting
		@Expose
		public float inflightCorrection;
		
		public double headingRad;
		public double headingDeg;
		
		@Override
		protected void parse(BinaryParser parser, byte[] packet)
		{
			headingRad = parser.getFloat(packet);
			headingDeg = Math.toDegrees(headingRad);
			
			declination = parser.getFloat(packet);
			inflightCorrection = parser.getFloat(packet);
			
			pureX = parser.getFloat(packet);
			pureY = parser.getFloat(packet);
			pureZ = parser.getFloat(packet);
			
			scaleX = parser.getFloat(packet);
			scaleY = parser.getFloat(packet);
			scaleZ = parser.getFloat(packet);
			
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
		
		@Override
		public boolean settingsEquals(TripleAxisSensor s)
		{
			if(s instanceof Magneto)
			{
				Magneto mag = (Magneto)s;
				
				if(	offsetX == mag.offsetX &&
					offsetY == mag.offsetY &&
					offsetZ == mag.offsetZ &&
					scaleX == mag.scaleX &&
					scaleY == mag.scaleY &&
					scaleZ == mag.scaleZ &&
					declination == mag.declination &&
					inflightCorrection == mag.inflightCorrection)
				return true;
			}
				
			return false;
		}	
	}
	
	@SettingGroup(name = "MAGNETO")
	@Expose
	@SerializedName("magneto")
	public Magneto magneto = new Magneto();
	
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

		public double target;
		public double errSum;
		public double input;
		
		public double out;
		
		protected void parse(BinaryParser parser, byte[] packet)
		{
			kp = parser.getFloat(packet);
			ki = parser.getFloat(packet);
			kd = parser.getFloat(packet);
			target = parser.getFloat(packet);
			out = parser.getFloat(packet);
			errSum = parser.getFloat(packet);
			input = parser.getFloat(packet);
			maxOut = parser.getFloat(packet);
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
		public Object clone()
		{
			Pid pid = new Pid();
			
			pid.enabled = this.enabled;
			pid.errSum = this.errSum;
			pid.input = this.input;
			pid.kd = this.kd;
			pid.ki = this.ki;
			pid.kp = this.kp;
			pid.maxOut = this.maxOut;
			pid.out = this.out;
			pid.target = this.target;
			
			return pid;
		}
	}
	
	public static class VeloZPid extends Pid
	{
		@NoChart
		@Setting
		@Expose
		public static double takeoffErrSum;
		
		@NoChart
		@Setting
		@Expose
		public float accUpOffset;
	}
	
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
	public VeloZPid velocityZPid = new VeloZPid();
	
	/**@note it class needs only to have target fields in rad and deg to plot its on chart */
	public static class AngularPid extends Pid implements Cloneable
	{
		public double targetDeg;
		public double inputDeg;
		
		@Override
		protected void parse(BinaryParser parser, byte[] packet)
		{
			super.parse(parser, packet);
			
			targetDeg = Math.toDegrees(this.target);
			inputDeg = Math.toDegrees(this.input);
		}

		@Override
		public Object clone()
		{
			AngularPid pid = new AngularPid();
			
			pid.enabled = this.enabled;
			pid.errSum = this.errSum;
			pid.input = this.input;
			pid.kd = this.kd;
			pid.ki = this.ki;
			pid.kp = this.kp;
			pid.maxOut = this.maxOut;
			pid.out = this.out;
			pid.target = this.target;
			
			pid.targetDeg = this.targetDeg;
			pid.inputDeg = this.inputDeg;
			
			return pid;
		}
	}
	
	@SettingGroup(name = "Z_RATE_PID")
	@Expose
	@SerializedName("zRatePid")
	public AngularPid zRatePid = new AngularPid();
	@SettingGroup(name = "Y_RATE_PID")
	@Expose
	@SerializedName("yRatePid")
	public AngularPid yRatePid = new AngularPid();
	@SettingGroup(name = "X_RATE_PID")
	@Expose
	@SerializedName("xRatePid")
	public AngularPid xRatePid = new AngularPid();
	@SettingGroup(name = "PITCH_PID")
	@Expose
	@SerializedName("pitchPid")
	public AngularPid pitchPid = new AngularPid();
	@SettingGroup(name = "ROLL_PID")
	@Expose
	@SerializedName("rollPid")
	public AngularPid rollPid = new AngularPid();
	@SettingGroup(name = "HEADING_PID")
	@Expose
	@SerializedName("headingPid")
	public AngularPid headingPid = new AngularPid();
	
	public static class NavState implements Cloneable
	{
		public double acc;
		public double vel;
		public double pos;
		public double accBias;
		
		protected void parse(BinaryParser parser, byte[] packet)
		{
			pos = parser.getFloat(packet);
			vel = parser.getFloat(packet);
			acc = parser.getFloat(packet);
			accBias = parser.getFloat(packet);
		}

		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}
	
	public NavState navNorth = new NavState();
	public NavState navEast = new NavState();
	public NavState navUp = new NavState();
	public NavState navYaw = new NavState();
	public NavState navPitch = new NavState();
	public NavState navRoll = new NavState();
	
	/// Kalman Filter settings takes 24 bytes
	public static class KalmanSettings implements Cloneable
	{
		/// Navigation model uncertainty
		@NoChart
		@Setting
		@Expose
		public float navModelNoise;

		/// The uncertainty of accel
		@NoChart
		@Setting
		@Expose
		public float accVariance;
		
		/// The uncertainty of altitude measured by baro
		@NoChart
		@Setting
		@Expose
		public float baroAltVariance;
		
		/// The uncertainty of vertical velocity measured by lidar when horizontal velocity below 0.3 m/s
		@NoChart
		@Setting
		@Expose
		public float lidarVelVariance1;

		/// The uncertainty of horizontal velocity measured by optical flow when lidar valid
		@NoChart
		@Setting
		@Expose
		public float ofVelVariance1;

		/// The uncertainty of horizontal velocity measured by optical flow when lidar invalid
		@NoChart
		@Setting
		@Expose
		public float ofVelVariance2;
		
		/// The uncertainty of vertical velocity measured by lidar when horizontal velocity above 0.3 m/s
		@NoChart
		@Setting
		@Expose
		public float lidarVelVariance2;
		
		/// Pose model uncertainty
		@NoChart
		@Setting
		@Expose
		public float poseModelNoise;
		
		/// The uncertainty of yaw angular rate measured by gyroscope
		@NoChart
		@Setting
		@Expose
		public float yawRateVariance;
		
		/// The uncertainty of pitch/roll angular rate measured by gyroscope
		@NoChart
		@Setting
		@Expose
		public float pitchRollRateVariance;
		
		/// The uncertainty of heading measured by magnetometer
		@NoChart
		@Setting
		@Expose
		public float magHeadingVariance;
		
		/// The uncertainty of pitch/roll measured by accelerometer
		@NoChart
		@Setting
		@Expose
		public float accPitchRollVariance;
		
		/// The uncertainty of horizontal speed from GPS
		@NoChart
		@Setting
		@Expose
		public float gpsHorSpeedVariance;
		
		/// The uncertainty of vertical speed from GPS
		@NoChart
		@Setting
		@Expose
		public float gpsVerSpeedVariance;
		
		/// The uncertainty of horizontal position from GPS
		@NoChart
		@Setting
		@Expose
		public float gpsHorPosVariance;
		
		/// The uncertainty of vertical position from GPS
		@NoChart
		@Setting
		@Expose
		public float gpsVerPosVariance;
		
		protected void parse(BinaryParser parser, byte[] packet)
		{
			navModelNoise = parser.getFloat(packet);
			accVariance = parser.getFloat(packet);
			baroAltVariance = parser.getFloat(packet);
			lidarVelVariance1 = parser.getFloat(packet);
			ofVelVariance1 = parser.getFloat(packet);
			ofVelVariance2 = parser.getFloat(packet);
			lidarVelVariance2 = parser.getFloat(packet);
			poseModelNoise = parser.getFloat(packet);
			yawRateVariance = parser.getFloat(packet);
			pitchRollRateVariance = parser.getFloat(packet);
			magHeadingVariance = parser.getFloat(packet);
			accPitchRollVariance = parser.getFloat(packet);
			gpsHorSpeedVariance = parser.getFloat(packet);
			gpsVerSpeedVariance = parser.getFloat(packet);
			gpsHorPosVariance = parser.getFloat(packet);
			gpsVerPosVariance = parser.getFloat(packet);
		}

		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	};
	
	@SettingGroup(name = "KALMAN_FILTER")
	@Expose
	@SerializedName("kalman")
	public KalmanSettings kfSettings = new KalmanSettings();
	
	/*
	public static class UserData1 implements Cloneable
	{
		@Setting
		@NoChart
		@Expose
		public float userData0;
		@Setting
		@NoChart
		@Expose
		public float userData1;
		@Setting
		@NoChart
		@Expose
		public float userData2;
		@Setting
		@NoChart
		@Expose
		public float userData3;
		@Setting
		@NoChart
		@Expose
		public float userData4;
		@Setting
		@NoChart
		@Expose
		public float userData5;
		@Setting
		@NoChart
		@Expose
		public float userData6;
		@Setting
		@NoChart
		@Expose
		public float userData7;
		@Setting
		@NoChart
		@Expose
		public float userData8;
		@Setting
		@NoChart
		@Expose
		public float userData9;
		@Setting
		@NoChart
		@Expose
		public float userData10;
		@Setting
		@NoChart
		@Expose
		public float userData11;
		
		private void parse(BinaryParser parser, DatagramPacket packet)
		{
			userData0 = parser.getFloat(packet);
			userData1 = parser.getFloat(packet);
			
			userData2 = parser.getFloat(packet);
			userData3 = parser.getFloat(packet);
			
			userData4 = parser.getFloat(packet);
			userData5 = parser.getFloat(packet);

			userData6 = parser.getFloat(packet);
			userData7 = parser.getFloat(packet);
			
			userData8 = parser.getFloat(packet);
			userData9 = parser.getFloat(packet);
			
			userData10 = parser.getFloat(packet);
			userData11 = parser.getFloat(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
	}

	@SettingGroup(name = "USER_DATA1")
	@Expose
	@SerializedName("userdata1")
	public UserData1 fUserData = new UserData1();
	
	*/
	
	@NoChart
	public double timestamp;
	@NoChart
	public long time;
	@NoChart
	public int osd;
	
	public double rssi;
	
	public double errors;
	
	@NoChart
	public boolean motorsEnabled;
	@NoChart
	public boolean stabilizationEnabled;
	
	public static enum TrickMode 
	{
		DISABLED(0),
		ACRO(10),
		GYRO(20),
		UNKNOWN(255);

		private int mCode;
		
		private TrickMode(int code)
		{
			mCode = code;
		}
		
		public int toInt()
		{
			return mCode;
		}
		
		public static TrickMode fromInt(int code)
		{
			if(code == DISABLED.toInt())
				return DISABLED;
			else if(code == ACRO.toInt())
				return ACRO;
			else if(code == GYRO.toInt())
				return GYRO;
			return UNKNOWN;
		}
	}
	
	@NoChart
	public TrickMode trickMode;
	
	public double pidFlags;
	public double headingPidFlag;
	public double veloXPidFlag;
	public double veloYPidFlag;
	public double altPidFlag;
	public double posNorthPidFlag;
	public double posEastPidFlag;
	public double pitchPidFlag;
	public double rollPidFlag;
	
	public double baseGas;
	public double motorGas[] = new double[Motors.count];
	/*
	public double motorGas0;
	public double motorGas1;
	public double motorGas2;
	public double motorGas3;
	*/
	
	public double yawRad;
	public double yawDeg;
	
	public double pitchRad;
	public double pitchDeg;
	
	public double rollRad;
	public double rollDeg;
	
	public double loopPeriod;
	
	public double gyroTaskTime;
	public double accelTaskTime;
	public double magTaskTime;
	public double lidarTaskTime;
	public double baroTaskTime;
	public double ofTaskTime;
	public double rcTaskTime;
	public double batteryTaskTime;
	public double telemetryTaskTime;
	public double escTaskTime;
	public double pidTaskTime;
	public double gpsTaskTime;
	public double loadTaskTime;
	
	public double yawRateTarget;
	public double pitchRateTarget;
	public double rollRateTarget;
	public double yawRateTargetDeg;
	public double pitchRateTargetDeg;
	public double rollRateTargetDeg;
	
	public double temperature;
	
	public double altitude;
	
	public double velNorth;
	public double velEast;
	public double velUp;
	public double velGnd;
	
	public double posNorth;
	public double posEast;
	
	public double distToHome;
	public double headToHome;
	
	@SettingGroup(name = "POS_NORTH_PID")
	@Expose
	@SerializedName("posNorthPid")
	public Pid posNorthPid = new Pid();
	
	@SettingGroup(name = "POS_EAST_PID")
	@Expose
	@SerializedName("posEastPid")
	public Pid posEastPid = new Pid();
	
	@NoChart
	public int version;
	
	@NoChart
	public boolean videoState;
	
	@Override
	public Object clone() throws CloneNotSupportedException
	{
		DroneState state = (DroneState)super.clone();
		//state.net = (Net)net.clone();
		state.motorGas = new double[DroneState.Motors.count];
		for(int i = 0; i < DroneState.Motors.count && i < motorGas.length; i++)	// user can change motor count, in this case motorGas.length can be lower new motors.count
		{
			state.motorGas[i] = motorGas[i];
		}
		state.battery = (Battery)battery.clone();
		state.accel = (Accel)accel.clone();
		state.gyro = (Gyro)gyro.clone();
		state.magneto = (Magneto)magneto.clone();
		state.opticalFlow = (OpticalFlow)opticalFlow.clone();
		state.lidar = (Lidar)lidar.clone();
		state.kfSettings = (KalmanSettings)kfSettings.clone();
		state.navUp = (NavState)navUp.clone();
		state.navNorth = (NavState)navNorth.clone();
		state.navEast = (NavState)navEast.clone();
		state.navYaw = (NavState)navYaw.clone();
		state.navPitch = (NavState)navPitch.clone();
		state.navRoll = (NavState)navRoll.clone();
		state.zRatePid = (AngularPid)zRatePid.clone();
		state.yRatePid = (AngularPid)yRatePid.clone();
		state.xRatePid = (AngularPid)xRatePid.clone();
		state.pitchPid = (AngularPid)pitchPid.clone();
		state.rollPid = (AngularPid)rollPid.clone();
		state.altPid = (Pid)altPid.clone();
		return state;
	}
	
	public double refYawRad;
	public double refPitchRad;
	public double refRollRad;
	
	public double refYawDeg;
	public double refPitchDeg;
	public double refRollDeg;
	
	public void parse(BinaryParser parser, byte[] packet)
	{
		version = parser.getUint8t(packet);
		
		if(version != SUPPORTED_PDL_VERSION)
		{
			DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_UNSUPPORTED_FIRMWARE);
		}
		else
		{
			DroneAlarmCenter.instance().clearAlarm(Alarm.ALARM_UNSUPPORTED_FIRMWARE);
		}
		
		motorsEnabled = parser.getBool(packet);
		stabilizationEnabled = parser.getBool(packet);
		pidFlags = parser.getUint8t(packet);
		
		int flags = (int)pidFlags;
		
		headingPidFlag = ( (flags & 1) > 0)?1:0;
		veloXPidFlag = ( (flags & 2) > 0)?1:0;
		veloYPidFlag = ( (flags & 4) > 0)?1:0;
		altPidFlag = ( (flags & 8) > 0)?1:0;
		posNorthPidFlag = ( (flags & 16) > 0)?1:0;
		posEastPidFlag = ( (flags & 32) > 0)?1:0;
		pitchPidFlag = ( (flags & 64) > 0)?1:0;
		rollPidFlag = ( (flags & 128) > 0)?1:0;
		
		motors.esc = parser.getUint8t(packet);
		motors.frame = parser.getUint8t(packet);
		
		videoState = (parser.getUint8t(packet) > 0)?true:false;
		
		trickMode = TrickMode.fromInt(parser.getUint8t(packet));
		
		errors = parser.getUint32t(packet);
		
		baseGas = parser.getInt32t(packet);

		timestamp = parser.getUint32t(packet);
		
		battery.parse(parser, packet);
		
		rssi = parser.getInt32t(packet);
		
		accel.parse(parser, packet);
		gyro.parse(parser, packet);
		magneto.parse(parser, packet);

		loopPeriod = parser.getInt32t(packet);
		
		gyroTaskTime = parser.getInt32t(packet);
		accelTaskTime = parser.getInt32t(packet);
		magTaskTime = parser.getInt32t(packet);
		lidarTaskTime = parser.getInt32t(packet);
		baroTaskTime = parser.getInt32t(packet);
		ofTaskTime = parser.getInt32t(packet);
		rcTaskTime = parser.getInt32t(packet);
		batteryTaskTime = parser.getInt32t(packet);
		telemetryTaskTime = parser.getInt32t(packet);;
		escTaskTime = parser.getInt32t(packet);
		pidTaskTime = parser.getInt32t(packet);
		gpsTaskTime = parser.getInt32t(packet);
		loadTaskTime = parser.getInt32t(packet);
		
		yawRateTarget = parser.getFloat(packet);
		pitchRateTarget = parser.getFloat(packet);
		rollRateTarget = parser.getFloat(packet);
		
		temperature = parser.getFloat(packet);
		
		baro.parse(parser, packet);
		opticalFlow.parse(parser, packet);
		lidar.parse(parser, packet);
		
		navNorth.parse(parser, packet);
		navEast.parse(parser, packet);
		navUp.parse(parser, packet);
		
		navRoll.parse(parser, packet);
		navPitch.parse(parser, packet);
		navYaw.parse(parser, packet);

		//fUserData.parse(parser, packet);
		
		zRatePid.parse(parser, packet);
		yRatePid.parse(parser, packet);
		xRatePid.parse(parser, packet);
		
		pitchPid.parse(parser, packet);
		rollPid.parse(parser, packet);
		
		altPid.parse(parser, packet);
		
		velocityXPid.parse(parser, packet);
		velocityYPid.parse(parser, packet);
		velocityZPid.parse(parser, packet);
		
		headingPid.parse(parser, packet);
		
		posNorthPid.parse(parser, packet);
		posEastPid.parse(parser, packet);
		
		kfSettings.parse(parser, packet);
		
		gps.parse(parser, packet);
		
		refYawRad = parser.getFloat(packet);
		refPitchRad = parser.getFloat(packet);
		refRollRad = parser.getFloat(packet);
		
		refYawDeg = Math.toDegrees(refYawRad);
		refPitchDeg = Math.toDegrees(refPitchRad);
		refRollDeg = Math.toDegrees(refRollRad);
		
		// Now reserved2 is accUpOffset 
		velocityZPid.accUpOffset = parser.getFloat(packet);
		// Now Reserved3 is telemetry.period
		telemetry.period = (int)parser.getUint32t(packet);
		// Now Reserved4 is OSD
		osd = (int)parser.getUint32t(packet);
		// Now Reserved5 and Reserved6 is time
		time = parser.getUint64t(packet);
		
		for(int i = 0; i < DroneState.Motors.count; i++)
		{
			this.motorGas[i] = parser.getInt32t(packet);
		}
		
		load1.parse(parser, packet);
		load2.parse(parser, packet);
		
		//--------------------------------------------------
		
		altitude = navUp.pos;
		
		velNorth = navNorth.vel;
		velEast = navEast.vel;
		velUp = navUp.vel;
		
		posNorth = navNorth.pos;
		posEast = navEast.pos;
		
		distToHome = Math.sqrt(posNorth*posNorth + posEast*posEast);
		headToHome = Math.toDegrees(Math.atan2(posEast, posNorth)) - 180.0;
		if(headToHome < 0)
			headToHome += 360.0;
		if(headToHome > 360)
			headToHome -= 360.0;
		
		velGnd = Math.sqrt(velNorth*velNorth + velEast*velEast);
		
		yawRad = navYaw.pos;
		yawDeg = Math.toDegrees(yawRad);
		pitchRad = navPitch.pos;
		pitchDeg = Math.toDegrees(pitchRad);
		rollRad = navRoll.pos;
		rollDeg = Math.toDegrees(rollRad);
		
		yawRateTargetDeg = Math.toDegrees(yawRateTarget);
		pitchRateTargetDeg = Math.toDegrees(pitchRateTarget);
		rollRateTargetDeg = Math.toDegrees(rollRateTarget);
	}
	
	public static class Lidar implements Cloneable
	{
		public double range;
		public double velZ;
		
		protected void parse(BinaryParser parser, byte[] packet)
		{
			range = parser.getFloat(packet);
			velZ = parser.getFloat(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
		
		public boolean settingsEquals(Lidar lidar)
		{
			return true;
		}
	}
	
	public Lidar lidar = new Lidar();
	
	public static class OpticalFlow implements Cloneable
	{
		public double rawX;
		public double rawY;
		public double squal;
		public double pureX;
		public double pureY;
		public double velX;
		public double velY;
		public double velNorth;
		public double velEast;
		public double sumX;
		public double sumY;
		
		protected void parse(BinaryParser parser, byte[] packet)
		{
			rawX = parser.getInt16t(packet);
			rawY = parser.getInt16t(packet);
			squal = parser.getUint32t(packet);
			pureX = parser.getFloat(packet);
			pureY = parser.getFloat(packet);
			velX = parser.getFloat(packet);
			velY = parser.getFloat(packet);
			velNorth = parser.getFloat(packet);
			velEast = parser.getFloat(packet);
			sumX = parser.getFloat(packet);
			sumY = parser.getFloat(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
		
		public boolean settingsEquals(OpticalFlow of)
		{
			return true;
		}
	}
	
	public OpticalFlow opticalFlow = new OpticalFlow();
	
	public static class Gps implements Cloneable
	{
		@NoChart
		@Expose
		@Setting
		public boolean enabled;
		public double fixType;
		public double numSV;
		public double lat;
		public double lon;
		public double alt;
		public double startLat;
		public double startLon;
		public double startAlt;
		public double course;
		public double velN;
		public double velE;
		public double velUp;
		public double hAcc;
		public double vAcc;
		public double sAcc;
		public double posNorth;
		public double posEast;
		public double posUp;
		
		protected void parse(BinaryParser parser, byte[] packet)
		{
			enabled = parser.getBool(packet);
			fixType = parser.getUint8t(packet);
			numSV = parser.getInt16t(packet);
			lat = parser.getFloat(packet);
			lon = parser.getFloat(packet);
			alt = parser.getFloat(packet);
			startLat = parser.getFloat(packet);
			startLon = parser.getFloat(packet);
			startAlt = parser.getFloat(packet);
			course = parser.getFloat(packet);
			velN = parser.getFloat(packet);
			velE = parser.getFloat(packet);
			velUp = parser.getFloat(packet);
			hAcc = parser.getFloat(packet);
			vAcc = parser.getFloat(packet);
			sAcc = parser.getFloat(packet);
			posNorth = parser.getFloat(packet);
			posEast = parser.getFloat(packet);
			posUp = parser.getFloat(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
		
		public boolean settingsEquals(Gps gps)
		{
			if(this.enabled == gps.enabled)
				return true;
			
			return false;
		}
	}
	
	@SettingGroup(name = "GPS")
	@Expose
	@SerializedName("gps")
	public Gps gps = new Gps();
	
	public static class Baro implements Cloneable
	{
		public double pressure;
		public double altitude;
		public double seaLevelPressure;
		
		protected void parse(BinaryParser parser, byte[] packet)
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
		
		public boolean settingsEquals(Baro baro)
		{
			return true;
		}
	}
	
	public Baro baro = new Baro();
	
	public static class Load implements Cloneable
	{
		@NoChart
		@Expose
		@Setting
		public int period;
	
		@NoChart
		public boolean enabled;
		
		@NoChart
		public double timestamp;
		
		public double state;
		
		protected void parse(BinaryParser parser, byte[] packet)
		{
			enabled = parser.getBool(packet);
			state = parser.getUint8t(packet);
			period = parser.getUint16t(packet);
			timestamp = parser.getInt32t(packet);
		}
		
		@Override
		public Object clone() throws CloneNotSupportedException
		{
			return super.clone();
		}
		
		public boolean settingsEquals(Load load)
		{
			if(	enabled == load.enabled &&
				period == load.period )
			{
				return true;
			}
			
			return false;
		}
	}
	
	@SettingGroup(name = "LOAD1")
	@Expose
	@SerializedName("load1")
	public Load load1 = new Load();
	
	@SettingGroup(name = "LOAD2")
	@Expose
	@SerializedName("load2")
	public Load load2 = new Load();
}
