package pdl.res;

public abstract class SoundProvider
{
	public static String ALT_1M = "ALT_1M";
	public static String ALT_2M = "ALT_2M";
	public static String ALT_3M = "ALT_3M";
	public static String ALT_5M = "ALT_5M";
	public static String ALT_7M = "ALT_7M";
	public static String ALT_10M = "ALT_10M";
	public static String ALT_15M = "ALT_15M";
	public static String ALT_20M = "ALT_20M";
	public static String ALT_30M = "ALT_30M";
	public static String ALT_40M = "ALT_40M";
	public static String ALT_50M = "ALT_50M";
	public static String MOTORS_ENABLED = "MOTORS_ENABLED";
	public static String MOTORS_DISABLED = "MOTORS_DISABLED";
	public static String SYSTEM_BAD = "SYSTEM_BAD";
	public static String SYSTEM_OK = "SYSTEM_OK";
	public static String STABILIZATION_ENABLED = "STABILIZATION_ENABLED";
	public static String STABILIZATION_DISABLED = "STABILIZATION_DISABLED";
	public static String TRICK_MODE_ENABLED = "TRICK_MODE_ENABLED";
	public static String TRICK_MODE_DISABLED = "TRICK_MODE_DISABLED";
	public static String VIDEO_STARTED = "VIDEO_STARTED";
	public static String VIDEO_STOPED = "VIDEO_STOPED";
	public static String PHOTO_TAKEN = "PHOTO_TAKEN";
	public static String BAT15 = "BAT15";
	public static String BAT10 = "BAT10";
	public static String BAT5 = "BAT5";
	public static String BAT1 = "BAT1";
	public static String LOW_RADIO_SIGNAL = "LOW_RADIO_SIGNAL";
	
	public abstract boolean init();	
	public abstract void play(String key);
	public abstract void playLater(String key, int delayMs);
}
