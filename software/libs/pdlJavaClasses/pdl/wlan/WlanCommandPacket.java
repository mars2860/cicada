package pdl.wlan;

import pdl.commands.AbstractDroneCmd;

public class WlanCommandPacket extends WlanPacket
{
	public static final int TYPE_ID = 0;
	
	protected AbstractDroneCmd mCmd;
	protected byte[] mData;
	
	public WlanCommandPacket(AbstractDroneCmd cmd, int droneId, int packetNum)
	{
		super(TYPE_ID, droneId, packetNum);
		
		mCmd = cmd;
		mData = cmd.getWlanPacketData(droneId, packetNum);
	}
	
	public byte[] getDataToSend()
	{
		return mData;
	}
}