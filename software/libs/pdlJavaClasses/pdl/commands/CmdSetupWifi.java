package pdl.commands;

public class CmdSetupWifi extends AbstractDroneCmd
{
	protected boolean sta;
	protected boolean dhcp;
	protected int chl;
	protected int tpw;
	protected int phy;
	protected int rate;
	
	public CmdSetupWifi(boolean staMode, boolean dhcpEnabled, int channel, float txPowerDbm, int phyMode, int rate)
	{
		super(160);
		
		this.sta = staMode;
		this.dhcp = dhcpEnabled;
		this.chl = channel;
		this.tpw = (int)(txPowerDbm / 0.25f);
		this.phy = phyMode;
		this.rate = rate;
	}
	
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[7];
		pos = this.writeUint8(pos, data, getCode());
		pos = this.writeUint8(pos, data, (sta)?1:0);
		pos = this.writeUint8(pos, data, (dhcp)?1:0);
		pos = this.writeUint8(pos, data, chl);
		pos = this.writeUint8(pos, data, tpw);
		pos = this.writeUint8(pos, data, phy);
		pos = this.writeUint8(pos, data, rate);
		return data;
	}
}

