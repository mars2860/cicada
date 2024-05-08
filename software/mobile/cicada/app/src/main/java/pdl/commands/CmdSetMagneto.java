package pdl.commands;

import pdl.DroneState;

public class CmdSetMagneto extends CmdSetupTripleAxisSensor
{
	private float mSx;
	private float mSy;
	private float mSz;
	
	private float mDeclination;
	private float mInflightCorrection;
	
	public CmdSetMagneto(DroneState.Magneto mag)
	{
		super(mag, 104);
		
		mSx = mag.scaleX;
		mSy = mag.scaleY;
		mSz = mag.scaleZ;
		
		mDeclination = mag.declination;
		mInflightCorrection = mag.inflightCorrection;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[27];
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeInt16(pos,data,mSens.offsetX);
		pos = writeInt16(pos,data,mSens.offsetY);
		pos = writeInt16(pos,data,mSens.offsetZ);
		pos = writeFloat(pos,data,mSx);
		pos = writeFloat(pos,data,mSy);
		pos = writeFloat(pos,data,mSz);
		pos = writeFloat(pos,data,mDeclination);
		pos = writeFloat(pos,data,mInflightCorrection);
		
		return data;
	}

	@Override
	public boolean settingsEquals(DroneState ds) 
	{
		return mSens.settingsEquals(ds.magneto);
	}
}
