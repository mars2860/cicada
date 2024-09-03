package pdl.commands;

public abstract class AbstractCmdSetPidTarget extends AbstractDroneCmd
{
	private float target;
	
	public AbstractCmdSetPidTarget(int code, float v)
	{
		super(code);
		target = v;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeFloat(pos, data, target);
		return data;
	}
	
	public float getTarget()
	{
		return target;
	}
	
	public void setTarget(float v)
	{
		target = v;
	}
}
