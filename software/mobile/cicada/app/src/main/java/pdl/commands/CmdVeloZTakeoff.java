package pdl.commands;

public class CmdVeloZTakeoff extends AbstractDroneCmd
{
	private float errSum;
	public CmdVeloZTakeoff(float veloZPidErrSum)
	{
		super(148);
		errSum = veloZPidErrSum;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeFloat(pos, data, errSum);
		return data;
	}
}
