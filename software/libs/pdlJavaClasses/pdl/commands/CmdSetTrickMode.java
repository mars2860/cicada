package pdl.commands;

import pdl.DroneState;

public class CmdSetTrickMode extends AbstractDroneCmd
{	
	private DroneState.TrickMode mTrickMode;
		
	public CmdSetTrickMode(DroneState.TrickMode trickMode)
	{
		super(139);
		mTrickMode = trickMode;
	}
		
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[2];
		pos = this.writeUint8(pos, data, mCmdCode);
		pos = this.writeUint8(pos, data, mTrickMode.toInt());
		return data;
	}
}
