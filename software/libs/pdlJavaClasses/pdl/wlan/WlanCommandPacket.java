package pdl.wlan;

import pdl.commands.AbstractDroneCmd;

public class WlanCommandPacket extends WlanPacket
{
	public static final int TYPE_ID = 0;
	
	protected AbstractDroneCmd mCmd;
	protected int mDroneId;
	protected int mPacketNum;
	
	public WlanCommandPacket(AbstractDroneCmd cmd, int droneId, int packetNum)
	{
		super(TYPE_ID, droneId, packetNum);
		
		mCmd = cmd;
		mDroneId = droneId;
		mPacketNum = packetNum;
	}
	
	public byte[] getDataToSend()
	{
		return mCmd.getWlanPacketData(mDroneId, mPacketNum);
	}
}