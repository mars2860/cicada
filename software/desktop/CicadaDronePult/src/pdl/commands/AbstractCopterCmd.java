package pdl.commands;

public abstract class AbstractCopterCmd
{
	protected int mCmdCode;
	
	public AbstractCopterCmd(int code)
	{
		mCmdCode = code;
	}
	
	public int getCode()
	{
		return mCmdCode;
	}
	
	abstract public byte[] getPacketData();
	
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
}
