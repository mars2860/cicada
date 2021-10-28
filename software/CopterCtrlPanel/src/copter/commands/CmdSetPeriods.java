package copter.commands;

import copter.DroneState;

public class CmdSetPeriods extends CmdSetup
{
	private int mTelemetryPeriod;
	private int mPidPeriod;
	
	public CmdSetPeriods(int telemetryPeriod, int pidPeriod)
	{
		super(112);
		
		mTelemetryPeriod = telemetryPeriod;
		mPidPeriod = pidPeriod;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[9];
		pos = this.writeUint8(pos, data, mCmdCode);
		pos = this.writeInt32(pos, data, mTelemetryPeriod);
		pos = this.writeInt32(pos, data, mPidPeriod);
		return data;
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return (mTelemetryPeriod == ds.net.telemetryPeriod)?true:false;
	}
}
