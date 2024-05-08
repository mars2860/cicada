package pdl.commands;

public class CmdSwitchMotors extends AbstractDroneCmd
{
	private byte mState;
	
	public CmdSwitchMotors(boolean motorsOn)
	{
		super(100);
		
		if(motorsOn)
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
