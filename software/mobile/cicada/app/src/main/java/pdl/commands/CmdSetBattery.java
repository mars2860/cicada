package pdl.commands;

import pdl.DroneState;

public class CmdSetBattery extends CmdSetup
{
	float mVoltScaler;
	float mCurnScaler;
	float mMinVoltage;
	float mMaxVoltage;
	
	public CmdSetBattery(float voltScaler, float curnScaler, float minVoltage, float maxVoltage)
	{
		super(137);
		
		mVoltScaler = voltScaler;
		mCurnScaler = curnScaler;
		mMinVoltage = minVoltage;
		mMaxVoltage = maxVoltage;
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		if(	ds.battery.voltScaler == mVoltScaler &&
			ds.battery.curnScaler == mCurnScaler &&
			ds.battery.minVoltage == mMinVoltage &&
			ds.battery.maxVoltage == mMaxVoltage
	      )
		{
			return true;
		}
		return false;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[17];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeFloat(pos, data, mVoltScaler);
		pos = this.writeFloat(pos, data, mCurnScaler);
		pos = this.writeFloat(pos, data, mMinVoltage);
		pos = this.writeFloat(pos, data, mMaxVoltage);
		return data;
	}

}
