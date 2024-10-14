package pdl;

public class BinaryParser
{
	private int mParsePos;
	
	public BinaryParser()
	{
		mParsePos = 0;
	}
	
	public boolean getBool(byte[] packet)
	{
		int result = 0;
		
		result = packet[mParsePos++] & 0xFF;
		
		return (result > 0)? true : false;
	}
	
	public int getUint8t(byte[] packet)
	{
		int result = 0;
		
		result = packet[mParsePos++] & 0xFF;
		
		return result;
	}
	
	public int getInt16t(byte[] packet)
	{
		int result = 0;
		
		result = packet[mParsePos++] & 0xFF;
		result |= (packet[mParsePos++] & 0xFF) << 8;
		result <<= 16;
		result >>= 16;
		
		return result;
	}
	
	public int getInt32t(byte[] packet)
	{
		int result = 0;
		
		result = packet[mParsePos++] & 0xFF;
		result |= (packet[mParsePos++] & 0xFF) << 8;
		result |= (packet[mParsePos++] & 0xFF) << 16;
		result |= (packet[mParsePos++] & 0xFF) << 24;
		
		return result;
	}
	
	public long getUint32t(byte[] packet)
	{
		long result = 0;
		long t = 0;
		
		t = packet[mParsePos++] & 0xFF;
		result = t;
		t = (packet[mParsePos++] & 0xFF);
		result |= t << 8;
		t = (packet[mParsePos++] & 0xFF);
		result |= t << 16;
		t = (packet[mParsePos++] & 0xFF);
		result |= t << 24;
		
		return result;
	}
	
	public int getUint16t(byte[] packet)
	{
		int result = 0;
		
		result = packet[mParsePos++] & 0xFF;
		result |= (packet[mParsePos++] & 0xFF) << 8;
		
		return result;
	}
	
	public float getFloat(byte[] packet)
	{
		float result = 0;
		
		int bits = this.getInt32t(packet);
		result = Float.intBitsToFloat(bits);
		
		return result;
	}
	
	public String getString(byte[] packet)
	{
		String result = "";
		byte ch = 0;
		
		do
		{
			ch = packet[mParsePos++];
			if(ch != 0)
			{
				result += (char)ch;
			}
		}
		while(mParsePos < packet.length && ch != 0);
		
		return result;
	}
}
