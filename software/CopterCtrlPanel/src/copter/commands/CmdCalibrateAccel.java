package copter.commands;

public class CmdCalibrateAccel extends AbstractCopterCmd
{
	private int mDx;
	private int mDy;
	private int mDz;
	
	public CmdCalibrateAccel(int dx, int dy, int dz)
	{
		super(102);
		
		mDx = dx;
		mDy = dy;
		mDz = dz;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[7];
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeInt16(pos,data,mDx);
		pos = writeInt16(pos,data,mDy);
		pos = writeInt16(pos,data,mDz);
		
		return data;
	}
}
