package pdl.commands;

public class CmdEnableStabilization extends AbstractDroneCmd
{
	private int mEnabled;
	
	public CmdEnableStabilization(boolean enable)
	{
		super(113);
		mEnabled = (enable)?1:0;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[2];
		pos = this.writeUint8(pos, data, mCmdCode);
		pos = this.writeUint8(pos, data, mEnabled);
		return data;
	}

}
