package pdl.commands;

public class CmdEnableWifiBroadcast extends AbstractDroneCmd
{
	protected boolean enabled;
	
	public CmdEnableWifiBroadcast(boolean enabled)
	{
		super(161);
		
		this.enabled = enabled;
	}
	
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[2];
		pos = this.writeUint8(pos, data, getCode());
		pos = this.writeUint8(pos, data, (enabled)?1:0);
		return data;
	}
}