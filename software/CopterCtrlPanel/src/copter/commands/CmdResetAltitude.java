package copter.commands;

public class CmdResetAltitude extends AbstractCopterCmd
{
	public CmdResetAltitude()
	{
		super(114);
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[1];
		pos = this.writeUint8(pos, data, getCode());
		return data;
	}
}
