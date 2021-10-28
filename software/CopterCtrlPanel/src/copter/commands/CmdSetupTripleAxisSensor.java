package copter.commands;

import copter.DroneState.TripleAxisSensor;

public abstract class CmdSetupTripleAxisSensor extends CmdSetup
{
	protected TripleAxisSensor mSens;
	
	public CmdSetupTripleAxisSensor(TripleAxisSensor sens, int cmd)
	{
		super(cmd);
		mSens = sens;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[7];
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeInt16(pos,data,mSens.offsetX);
		pos = writeInt16(pos,data,mSens.offsetY);
		pos = writeInt16(pos,data,mSens.offsetZ);
		
		return data;
	}
}
