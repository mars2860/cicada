package pdl.commands;

import pdl.DroneState;

public class CmdSetEsc extends CmdSetup
{	
	private int esc;
		
	public CmdSetEsc(int esc)
	{
		super(141);
		this.esc = esc;
	}
		
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[2];
		pos = this.writeUint8(pos, data, mCmdCode);
		pos = this.writeUint8(pos, data, esc);
		return data;
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		if(ds.motors.esc == esc)
			return true;
		return false;
	}
}