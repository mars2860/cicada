package pdl.wlan;

/**
 * The drone wlan packet structure
 * {
 *   SSID = 0 to 24 bytes
 *   '\0' = 1 byte string termination symbol
 *   packetType = 1 byte (0 - Command, 1 - Telemetry, 2 - LOG)
 *   various (Command data, DroneState size (4 bytes) + DroneState bytes, LOG size (4 bytes) + LOG bytes)
 * }
 */
public abstract class WlanPacket
{
	protected int mTypeId;
	protected String mSsid;
	
	public WlanPacket(int typeId, String ssid)
	{
		mSsid = ssid;
		mTypeId = typeId;
	}
	
	public int getTypeId()
	{
		return mTypeId;
	}
	
	public String getSsid()
	{
		return mSsid;
	}
}