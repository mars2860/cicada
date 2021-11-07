package pdl.commands;

public class CmdSetYPR extends AbstractCopterCmd
{
	private float mYaw = 0.f;
	private float mPitch = 0.f;
	private float mRoll = 0.f;
	
	/** Angles should be in radians */
	public CmdSetYPR(float yawRad, float pitchRad, float rollRad)
	{
		super(111);
		
		mYaw = yawRad;
		mPitch = pitchRad;
		mRoll = rollRad;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[17];
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeFloat(pos,data,mYaw);
		pos = writeFloat(pos,data,mPitch);
		pos = writeFloat(pos,data,mRoll);
		
		return data;
	}
	
	public void setYawRateDeg(float yawRateDeg)
	{
		mYaw = (float)Math.toRadians(yawRateDeg);
	}
	
	public void setRollDeg(float rollDeg)
	{
		mRoll = (float)Math.toRadians(rollDeg);
	}
	
	public void setPitchDeg(float pitchDeg)
	{
		mPitch = (float)Math.toRadians(pitchDeg);
	}
	
	public boolean isPitchNull()
	{
		return (mPitch == 0.f)?true:false;
	}
	
	public boolean isRollNull()
	{
		return (mRoll == 0.f)?true:false;
	}
	
	public boolean isNull()
	{
		return (mYaw == 0.f && mRoll == 0.f && mPitch == 0.f)?true:false;
	}
	
	public float getPitchRad()
	{
		return mPitch;
	}
	
	public float getRollRad()
	{
		return mRoll;
	}
	
	public float getYawRad()
	{
		return mYaw;
	}
}
