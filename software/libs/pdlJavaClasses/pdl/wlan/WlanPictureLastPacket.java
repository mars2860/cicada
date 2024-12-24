package pdl.wlan;

import pdl.BinaryParser;

public class WlanPictureLastPacket extends WlanPacket
{
	public static final int TYPE_ID = 5;
	
	private byte data[];
	private int chunkNum;
	
	private WlanPictureLastPacket(	int droneId,
									int packetNum)
	{
		super(TYPE_ID, droneId, packetNum);
	}
	
	public static WlanPictureLastPacket parse(byte[] data)
	{
		BinaryParser parser = new BinaryParser();
		
		int pktDroneId = parser.getInt32t(data);
		int pktNum = parser.getInt32t(data);
		int packetType = parser.getUint8t(data);
		
		if(packetType != WlanPictureLastPacket.TYPE_ID)
		{
			return null;
		}
		
		WlanPictureLastPacket packet = new WlanPictureLastPacket(pktDroneId,pktNum);
		
		packet.chunkNum = parser.getUint16t(data);
		
		int chunkSize = data.length - parser.getPos();
		
		packet.data = new byte[chunkSize];
		
		for(int i = 0; i < chunkSize; i++)
		{
			packet.data[i] = data[parser.getPos() + i];
		}
		
		return packet;
	}
	
	public int getChunkNum()
	{
		return chunkNum;
	}

	public byte[] getData()
	{
		return data;
	}
}
