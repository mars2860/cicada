package pdl.commands;

import pdl.DroneState;

public class CmdSetAccUpOffset extends CmdSetup
{
	protected float accUpOffset;

	public CmdSetAccUpOffset(float offset)
	{
		super(156);

		accUpOffset = offset;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = writeUint8(pos,data,this.getCode());
		pos = writeFloat(pos,data,accUpOffset);
		return data;
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		if(accUpOffset == ds.velocityZPid.accUpOffset)
		{
			return true;
		}
		return false;
	}
}
