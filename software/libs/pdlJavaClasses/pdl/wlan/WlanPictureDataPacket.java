package pdl.wlan;

import pdl.BinaryParser;

public class WlanPictureDataPacket extends WlanPacket
{
	public static final int TYPE_ID = 4;
	
	private byte data[];
	private int chunkNum;
	
	private WlanPictureDataPacket(	int droneId,
									int packetNum)
	{
		super(TYPE_ID, droneId, packetNum);
	}
	
	public static WlanPictureDataPacket parse(byte[] data)
	{
		BinaryParser parser = new BinaryParser();
		
		int pktDroneId = parser.getInt32t(data);
		int pktNum = parser.getInt32t(data);
		int packetType = parser.getUint8t(data);
		
		if(packetType != WlanPictureDataPacket.TYPE_ID)
		{
			return null;
		}
		
		WlanPictureDataPacket packet = new WlanPictureDataPacket(pktDroneId,pktNum);
		
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
