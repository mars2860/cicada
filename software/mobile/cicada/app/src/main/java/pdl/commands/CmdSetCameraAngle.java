package pdl.commands;

public class CmdSetCameraAngle extends AbstractDroneCmd
{
	private int mAngle;
	
	public CmdSetCameraAngle(int angle)
	{
		super(152);
		mAngle = angle;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeInt32(pos, data, mAngle);
		return data;
	}
}