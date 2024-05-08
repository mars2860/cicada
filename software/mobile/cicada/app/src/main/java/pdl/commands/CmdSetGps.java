package pdl.commands;

import pdl.DroneState;

public class CmdSetGps extends CmdSetup
{
	private DroneState.Gps mGps;
	
	public CmdSetGps(DroneState.Gps gps)
	{
		super(138);
		
		mGps = gps;
	}
	
	@Override
	public byte[] getPacketData()
	{
		byte data[] = new byte[2];
		
		int enabled = (mGps.enabled == true) ? 1 : 0;
				
		data[0] = (byte)this.getCode();
		data[1] = (byte)enabled;
		
		return data;
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		return mGps.settingsEquals(ds.gps);
	}
}
