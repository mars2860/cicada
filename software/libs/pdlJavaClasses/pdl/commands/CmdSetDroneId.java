package pdl.commands;

public class CmdSetDroneId extends AbstractDroneCmd
{
	protected int mDroneId;
	
	public CmdSetDroneId(int droneId)
	{
		super(162);
		
		mDroneId = droneId;
	}
	
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, getCode());
		pos = this.writeInt32(pos, data, mDroneId);
		return data;
	}
}