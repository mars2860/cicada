package pdl.commands;

import pdl.wlan.WlanCommandPacket;

public abstract class AbstractDroneCmd
{
	protected int mCmdCode;
	
	public AbstractDroneCmd(int code)
	{
		mCmdCode = code;
	}
	
	public int getCode()
	{
		return mCmdCode;
	}
	
	abstract public byte[] getPacketData();
	
	public byte[] getWlanPacketData(int droneId, int packetNum)
	{
		byte cmdData[] = getPacketData();
		int len = 9 + cmdData.length;
		byte[] wlanPacketData = new byte[len];
		int pos = 0;
		
		pos = this.writeInt32(pos, wlanPacketData, droneId);
		pos = this.writeInt32(pos, wlanPacketData, packetNum);
		pos = this.writeUint8(pos, wlanPacketData, WlanCommandPacket.TYPE_ID);
		
		for(byte b: cmdData)
		{
			wlanPacketData[pos++] = b;
		}
		
		return wlanPacketData;
	}
	
	protected int writeInt16(int pos, byte data[], int value)
	{
		data[pos++] = (byte)(value & 0xFF);
		data[pos++] = (byte)((value >>> 8) & 0xFF);
		
		return pos;
	}
	
	protected int writeInt32(int pos, byte data[], int value)
	{
		data[pos++] = (byte)(value & 0xFF);
		data[pos++] = (byte)((value >>> 8) & 0xFF);
		data[pos++] = (byte)((value >>> 16) & 0xFF);
		data[pos++] = (byte)((value >>> 24) & 0xFF);
		
		return pos;
	}
	
	protected int writeUint8(int pos, byte data[], int value)
	{
		data[pos++] = (byte)(value & 0xFF);
		return pos;
	}
	
	protected int writeFloat(int pos, byte data[], float value)
	{
		int bits = Float.floatToRawIntBits(value);
		
		data[pos++] = (byte)(bits & 0xFF);
		data[pos++] = (byte)((bits >>> 8) & 0xFF);
		data[pos++] = (byte)((bits >>> 16) & 0xFF);
		data[pos++] = (byte)((bits >>> 24) & 0xFF);
		
		return pos;
	}
	
	protected int writeString(int pos, byte data[], String value)
	{
		for(byte val : value.getBytes())
		{
			data[pos++] = val;
		}
		return pos;
	}
}
