package copter.commands;

import copter.DroneState.Pid;

public abstract class CmdSetupPid extends CmdSetup
{
	protected Pid mPid;

	public CmdSetupPid(Pid pid, int code)
	{
		super(code);

		mPid = pid;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[24];
		int enabled = (mPid.enabled)?1:0;
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeUint8(pos,data,enabled);
		pos = writeFloat(pos,data,mPid.kp);
		pos = writeFloat(pos,data,mPid.ki);
		pos = writeFloat(pos,data,mPid.kd);
		pos = writeFloat(pos,data,mPid.maxOut);
		pos = writeFloat(pos,data,mPid.maxErrSum);
		
		return data;
	}
}
