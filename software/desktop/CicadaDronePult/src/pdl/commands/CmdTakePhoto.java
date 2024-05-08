package pdl.commands;

public class CmdTakePhoto extends AbstractDroneCmd
{	
	public CmdTakePhoto()
	{
		super(154);
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[1];
		pos = this.writeUint8(pos, data, this.getCode());
		return data;
	}
}