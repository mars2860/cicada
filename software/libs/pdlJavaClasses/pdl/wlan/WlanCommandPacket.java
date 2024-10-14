package pdl.wlan;

import pdl.commands.AbstractDroneCmd;

public class WlanCommandPacket extends WlanPacket
{
	public static final int TYPE_ID = 0;
	
	protected AbstractDroneCmd mCmd;
	protected byte[] mData;
	
	public WlanCommandPacket(AbstractDroneCmd cmd, String ssid)
	{
		super(TYPE_ID, ssid);
		
		mCmd = cmd;
		
		byte cmdData[] = mCmd.getPacketData();
		int len = mSsid.length() + 2 + cmdData.length;
		mData = new byte[len];
		int pos = 0;
		for(byte b: mSsid.getBytes())
		{
			mData[pos++] = b;
		}
		mData[pos++] = 0;	// string termination symbol
		mData[pos++] = TYPE_ID;
		for(byte b: cmdData)
		{
			mData[pos++] = b;
		}
	}
	
	public byte[] getDataToSend()
	{
		return mData;
	}
}