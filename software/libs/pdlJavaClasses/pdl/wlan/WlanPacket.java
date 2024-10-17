package pdl.wlan;

/**
 * The drone wlan packet structure
 * {
 *   droneId = 4 bytes
 *   packetNum = 4 bytes
 *   packetType = 1 byte (0 - Command, 1 - Telemetry, 2 - LOG)
 *   various (Command data, DroneState size (4 bytes) + DroneState bytes, LOG size (4 bytes) + LOG bytes)
 * }
 */
public abstract class WlanPacket
{
	protected int mTypeId;
	protected int mDroneId;
	protected int mNum;
	
	public WlanPacket(int typeId, int droneId, int num)
	{
		mDroneId = droneId;
		mTypeId = typeId;
		mNum = num;
	}
	
	public int getTypeId()
	{
		return mTypeId;
	}
	
	public int getDroneId()
	{
		return mDroneId;
	}
	
	public int getNum()
	{
		return mNum;
	}
}