package copter.commands;

public class CmdCalibrateMagnet extends AbstractCopterCmd
{
	private int mDx;
	private int mDy;
	private int mDz;
	private float mSx;
	private float mSy;
	private float mSz;
	
	public CmdCalibrateMagnet(int dx, int dy, int dz, float sx, float sy, float sz)
	{
		super(104);
		
		mDx = dx;
		mDy = dy;
		mDz = dz;
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
		pos = writeInt16(pos,data,mDx);
		pos = writeInt16(pos,data,mDy);
		pos = writeInt16(pos,data,mDz);
		pos = writeFloat(pos,data,mSx);
		pos = writeFloat(pos,data,mSy);
		pos = writeFloat(pos,data,mSz);
		
		return data;
	}
}
