package pdl.wlan;

import pdl.BinaryParser;

public class WlanLogPacket extends WlanPacket
{
	public static final int TYPE_ID = 2;
	
	protected String mText;

	private WlanLogPacket(String text, int droneId, int packetNum)
	{
		super(TYPE_ID,droneId,packetNum);
		mText = text;
	}
	
	public static WlanLogPacket parse(byte[] data)
	{
		BinaryParser parser = new BinaryParser();
		
		int pktDroneId = parser.getInt32t(data);
		int pktNum = parser.getInt32t(data);
		int packetType = parser.getUint8t(data);
		
		if(packetType != WlanLogPacket.TYPE_ID)
		{
			return null;
		}
		
		String text = parser.getString(data);
		
		return new WlanLogPacket(text,pktDroneId,pktNum);
	}
	
	public String getText()
	{
		return mText;
	}
}
