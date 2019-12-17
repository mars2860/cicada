package copter.commands;

public class CmdSetYawPid extends AbstractCopterCmd
{
	protected boolean mEnabled;
	protected float mKp;
	protected float mKi;
	protected float mKd;
	
	public CmdSetYawPid(boolean enabled, float kp, float ki, float kd)
	{
		super(107);
		
		mEnabled = enabled;
		mKp = kp;
		mKi = ki;
		mKd = kd;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[14];
		int enabled = (mEnabled)?1:0;
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeUint8(pos,data,enabled);
		pos = writeFloat(pos,data,mKp);
		pos = writeFloat(pos,data,mKi);
		pos = writeFloat(pos,data,mKd);
		
		return data;
	}

}