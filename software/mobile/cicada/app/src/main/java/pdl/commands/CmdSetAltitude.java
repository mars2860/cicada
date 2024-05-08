package pdl.commands;

public class CmdSetAltitude extends AbstractDroneCmd
{
	private float mAltitude;
	
	public CmdSetAltitude(float altitude)
	{
		super(116);
		mAltitude = altitude;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeFloat(pos, data, mAltitude);
		return data;
	}

}
