package pdl.commands;

import pdl.DroneState;

public class CmdSetAccel extends CmdSetupTripleAxisSensor
{
	private int dlpf;
	
	public CmdSetAccel(DroneState.Accel sens)
	{
		super(sens,102);
		
		dlpf = sens.dlpf;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[8];
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeInt16(pos,data,mSens.offsetX);
		pos = writeInt16(pos,data,mSens.offsetY);
		pos = writeInt16(pos,data,mSens.offsetZ);
		pos = writeUint8(pos,data,dlpf);
		
		return data;
	}
	
	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mSens.settingsEquals(ds.accel);
	}
}
