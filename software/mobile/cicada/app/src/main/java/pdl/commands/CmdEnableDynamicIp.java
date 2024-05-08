package pdl.commands;

public class CmdEnableDynamicIp extends AbstractDroneCmd
{
	private byte mEnabled;
	
	public CmdEnableDynamicIp(boolean enabled)
	{
		super(155);
		
		if(enabled)
			mEnabled = 1;
		else
			mEnabled = 0;
	}
	
	@Override
	public byte[] getPacketData()
	{
		byte data[] = {(byte)this.getCode(), mEnabled};
		return data;
	}
}
