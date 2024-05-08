package pdl.commands;

public class CmdSetMotorsDir extends AbstractDroneCmd
{
	private byte mState;
	
	public CmdSetMotorsDir(boolean dir)
	{
		super(149);
		
		if(dir)
			mState = 1;
		else
			mState = 0;
	}
	
	@Override
	public byte[] getPacketData()
	{
		byte data[] = {(byte)this.getCode(), mState};
		return data;
	}
}
