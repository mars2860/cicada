package copter.commands;

public class CmdSetSeaLevelPressure extends AbstractCopterCmd
{
	private float mPressure;
	
	public CmdSetSeaLevelPressure(float pressure)
	{
		super(115);
		mPressure = pressure;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, getCode());
		pos = this.writeFloat(pos, data, mPressure);
		return data;
	}

}
