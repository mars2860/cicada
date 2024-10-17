package pdl.wlan;

import pdl.BinaryParser;
import pdl.DroneState;

public class WlanTelemetryPacket extends WlanPacket
{
	public static final int TYPE_ID = 1;
	
	protected DroneState mDroneState;
	protected int mDroneStateSize;
	
	private WlanTelemetryPacket(DroneState droneState, int droneStateSize, int droneId, int packetNum)
	{
		super(TYPE_ID, droneId, packetNum);
		mDroneStateSize = droneStateSize;
		mDroneState = droneState;
	}
	
	public static WlanTelemetryPacket parse(byte[] data)
	{
		BinaryParser parser = new BinaryParser();
		
		int pktDroneId = parser.getInt32t(data);
		int pktNum = parser.getInt32t(data);
		int packetType = parser.getUint8t(data);
		
		if(packetType != WlanTelemetryPacket.TYPE_ID)
		{
			return null;
		}
		
		int droneStateSize = (int)parser.getUint32t(data);
		DroneState droneState = new DroneState();
		droneState.parse(parser,data);
		
		return new WlanTelemetryPacket(droneState,droneStateSize,pktDroneId,pktNum);
	}
	
	public int getDroneStateSize()
	{
		return mDroneStateSize;
	}
	
	public DroneState getDroneState()
	{
		return mDroneState;
	}
}