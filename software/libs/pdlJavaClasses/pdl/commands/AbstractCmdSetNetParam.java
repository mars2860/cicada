package pdl.commands;

public abstract class AbstractCmdSetNetParam extends AbstractDroneCmd
{
	protected String value;
	
	public AbstractCmdSetNetParam(int code, String paramValue)
	{
		super(code);
		value = paramValue;
	}
	
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[32];
		pos = this.writeUint8(pos, data, getCode());
		pos = this.writeString(pos, data, value);
		return data;
	}
}
