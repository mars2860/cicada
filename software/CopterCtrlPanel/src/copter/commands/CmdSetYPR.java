package copter.commands;

public class CmdSetYPR extends AbstractCopterCmd
{
	private float mYaw;
	private float mPitch;
	private float mRoll;
	
	/** Angles should be in radians */
	public CmdSetYPR(float yaw, float pitch, float roll)
	{
		super(111);
		
		mYaw = yaw;
		mPitch = pitch;
		mRoll = roll;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[13];
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeFloat(pos,data,mYaw);
		pos = writeFloat(pos,data,mPitch);
		pos = writeFloat(pos,data,mRoll);
		
		return data;
	}
}
