package pdl.commands;

import pdl.DroneState;

public class CmdSetFrame extends CmdSetup
{	
	private int frame;
		
	public CmdSetFrame(int frame)
	{
		super(142);
		this.frame = frame;
	}
		
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[2];
		pos = this.writeUint8(pos, data, mCmdCode);
		pos = this.writeUint8(pos, data, frame);
		return data;
	}

	@Override
	public boolean settingsEquals(DroneState ds) 
	{
		if(ds.motors.frame == frame)
			return true;
		return false;
	}
}