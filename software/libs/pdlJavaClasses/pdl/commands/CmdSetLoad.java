package pdl.commands;

import pdl.DroneState;

public class CmdSetLoad extends CmdSetup
{
	private boolean enabled;
	private int period;
	private int num;
	
	public CmdSetLoad(int num, boolean enabled, int period)
	{
		super(140);
		
		this.enabled = enabled;
		this.period = period;
	}

	@Override
	public boolean settingsEquals(DroneState ds)
	{
		DroneState.Load load = new DroneState.Load();
		
		load.enabled = enabled;
		load.period = period;
		
		if(num == 0)
		{
			return ds.load1.settingsEquals(load);
		}
		
		return false;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[5];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeUint8(pos, data, num);
		pos = this.writeUint8(pos, data, (enabled)?1:0);
		pos = this.writeInt16(pos, data, period);
		return data;
	}
}
