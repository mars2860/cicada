package copter.commands;

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
}
