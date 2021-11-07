package pdl.commands;

public class CmdSelfCalibrateAccel extends AbstractCopterCmd
{
	public CmdSelfCalibrateAccel()
	{
		super(105);
	}

	@Override
	public byte[] getPacketData()
	{
		byte data[] = {(byte)this.getCode()};
		return data;
	}

}
