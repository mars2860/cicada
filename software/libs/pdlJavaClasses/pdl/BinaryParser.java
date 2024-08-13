package pdl;

import java.net.DatagramPacket;

public class BinaryParser
{
	private int mParsePos;
	
	public BinaryParser()
	{
		mParsePos = 0;
	}
	
	public boolean getBool(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		
		return (result > 0)? true : false;
	}
	
	public int getUint8t(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		
		return result;
	}
	
	public int getInt16t(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 8;
		result <<= 16;
		result >>= 16;
		
		return result;
	}
	
	public int getInt32t(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 8;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 16;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 24;
		
		return result;
	}
	
	public long getUint32t(DatagramPacket packet)
	{
		long result = 0;
		long t = 0;
		
		t = packet.getData()[mParsePos++] & 0xFF;
		result = t;
		t = (packet.getData()[mParsePos++] & 0xFF);
		result |= t << 8;
		t = (packet.getData()[mParsePos++] & 0xFF);
		result |= t << 16;
		t = (packet.getData()[mParsePos++] & 0xFF);
		result |= t << 24;
		
		return result;
	}
	
	public int getUint16t(DatagramPacket packet)
	{
		int result = 0;
		
		result = packet.getData()[mParsePos++] & 0xFF;
		result |= (packet.getData()[mParsePos++] & 0xFF) << 8;
		
		return result;
	}
	
	public float getFloat(DatagramPacket packet)
	{
		float result = 0;
		
		int bits = this.getInt32t(packet);
		result = Float.intBitsToFloat(bits);
		
		return result;
	}
}
