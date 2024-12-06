package pdl.wlan;

import pdl.BinaryParser;

/**
 * The drone wlan packet structure
 * {
 *   droneId = 4 bytes
 *   packetNum = 4 bytes
 *   packetType = 1 byte (0 - Command, 1 - Telemetry, 2 - LOG)
 *   various (Command data, DroneState size (4 bytes) + DroneState bytes, LOG size (4 bytes) + LOG bytes)
 * }
 */
public class WlanPacket
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
	
	public static WlanPacket parse(byte[] data)
	{
		BinaryParser parser = new BinaryParser();
		
		int pktDroneId = parser.getInt32t(data);
		int pktNum = parser.getInt32t(data);
		int packetType = parser.getUint8t(data);
		
		switch(packetType)
		{
		case WlanLogPacket.TYPE_ID:
			return WlanLogPacket.parse(data);
		case WlanTelemetryPacket.TYPE_ID:
			return WlanTelemetryPacket.parse(data);
		case WlanPictureStartPacket.TYPE_ID:
			return WlanPictureStartPacket.parse(data);
		case WlanPictureDataPacket.TYPE_ID:
			return WlanPictureDataPacket.parse(data);
		}
		
		return new WlanPacket(packetType,pktDroneId,pktNum);
	}
}