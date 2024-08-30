package pdl.commands;

import pdl.DroneState;

public class CmdSetTelemetryPeriod extends CmdSetup
{
	private int mTelemetryPeriod;

	public CmdSetTelemetryPeriod(int telemetryPeriod)
	{
		super(112);
		
		mTelemetryPeriod = telemetryPeriod;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeInt32(pos, data, mTelemetryPeriod);
		return data;
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return (mTelemetryPeriod == ds.telemetry.period)?true:false;
	}
}
