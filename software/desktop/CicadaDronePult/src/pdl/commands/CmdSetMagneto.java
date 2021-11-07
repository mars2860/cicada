package pdl.commands;

import pdl.DroneState;
import pdl.DroneState.TripleAxisSensor;

public class CmdSetMagneto extends CmdSetupTripleAxisSensor
{
	private float mSx;
	private float mSy;
	private float mSz;
	
	public CmdSetMagneto(TripleAxisSensor sens, float sx, float sy, float sz)
	{
		super(sens, 104);
		
		mSx = sx;
		mSy = sy;
		mSz = sz;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[19];
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeInt16(pos,data,mSens.offsetX);
		pos = writeInt16(pos,data,mSens.offsetX);
		pos = writeInt16(pos,data,mSens.offsetZ);
		pos = writeFloat(pos,data,mSx);
		pos = writeFloat(pos,data,mSy);
		pos = writeFloat(pos,data,mSz);
		
		return data;
	}

	@Override
	public boolean settingsEquals(DroneState ds) 
	{
		return mSens.settingsEquals(ds.magneto);
	}
}
