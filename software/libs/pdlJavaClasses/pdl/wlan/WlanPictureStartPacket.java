package pdl.wlan;

import pdl.BinaryParser;

public class WlanPictureStartPacket extends WlanPacket
{
	public static final int TYPE_ID = 3;
	
	private int width;
	private int height;
	private int pixelformat;
	private int quality;
	private long timestamp;
	private int len;
	private byte data[];
	
	private WlanPictureStartPacket(	int droneId,
									int num )
	{
		super(TYPE_ID, droneId, num);
	}
	
	public static WlanPictureStartPacket parse(byte[] data)
	{
		BinaryParser parser = new BinaryParser();
		
		int pktDroneId = parser.getInt32t(data);
		int pktNum = parser.getInt32t(data);
		int packetType = parser.getUint8t(data);
		
		if(packetType != WlanPictureStartPacket.TYPE_ID)
		{
			return null;
		}
		
		WlanPictureStartPacket packet = new WlanPictureStartPacket(pktDroneId,pktNum);
		
		packet.width = parser.getInt16t(data);
		packet.height = parser.getInt16t(data);
		packet.pixelformat = parser.getUint8t(data);
		packet.quality = parser.getUint8t(data);
		packet.timestamp = parser.getUint64t(data);
		packet.len = parser.getInt32t(data);
		
		int chunkSize = data.length - parser.getPos();
		
		packet.data = new byte[chunkSize];
		
		for(int i = 0; i < chunkSize; i++)
		{
			packet.data[i] = data[parser.getPos() + i];
		}
		
		return packet;
	}
	
	public int getWidth()
	{
		return width;
	}
	
	public int getHeight()
	{
		return height;
	}
	
	public int getPixelFormat()
	{
		return pixelformat;
	}
	
	public int getQuality()
	{
		return quality;
	}
	
	public long getTimestamp()
	{
		return timestamp;
	}
	
	public int getLen()
	{
		return len;
	}
	
	public byte[] getData()
	{
		return data;
	}
}
