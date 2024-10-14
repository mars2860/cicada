package pdl.wlan;

import pdl.BinaryParser;

public class WlanLogPacket extends WlanPacket
{
	public static final int TYPE_ID = 2;
	
	protected String mText;

	private WlanLogPacket(String text, String ssid)
	{
		super(TYPE_ID,ssid);
		mText = text;
	}
	
	public static WlanLogPacket parse(byte[] data)
	{
		BinaryParser parser = new BinaryParser();
		
		String recSsid = parser.getString(data);
		int packetType = parser.getUint8t(data);
		
		if(packetType != WlanLogPacket.TYPE_ID)
		{
			return null;
		}
		
		String text = parser.getString(data);
		
		return new WlanLogPacket(text,recSsid);
	}
	
	public String getText()
	{
		return mText;
	}
}
