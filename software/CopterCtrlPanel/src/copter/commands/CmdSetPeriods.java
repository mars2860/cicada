package copter.commands;

public class CmdSetPeriods extends AbstractCopterCmd
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
}
