package pdl.commands;

public class CmdSetTime extends AbstractDroneCmd
{
	public CmdSetTime()
	{
		super(163);
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte[] data = new byte[9];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeInt64(pos, data, System.currentTimeMillis());
		return data;
	}
}
