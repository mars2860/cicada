package pdl.commands;

import pdl.DroneState;

@Deprecated
public class CmdSetSeaLevelPressure extends CmdSetup
{
	private float mPressure;
	
	public CmdSetSeaLevelPressure(float pressure)
	{
		super(115);
		mPressure = pressure;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, getCode());
		pos = this.writeFloat(pos, data, mPressure);
		return data;
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return (mPressure == ds.baro.seaLevelPressure)?true:false;
	}

}
