package pdl.commands;

/** Switch on/off led on ESP07 */
public class CmdSwitchMotors extends AbstractCopterCmd
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
