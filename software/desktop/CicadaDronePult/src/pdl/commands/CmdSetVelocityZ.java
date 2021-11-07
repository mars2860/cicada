package pdl.commands;

public class CmdSetVelocityZ  extends AbstractCopterCmd
{
	private float mvz;
	
	public CmdSetVelocityZ(float v)
	{
		super(123);
		mvz = v;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeFloat(pos, data, mvz);
		return data;
	}
	
	public double getVelocity()
	{
		return (double)mvz;
	}
}