package pdl.commands;

public class CmdStartStopVideo extends AbstractDroneCmd
{	
	private boolean mStart;
	
	public CmdStartStopVideo(boolean start)
	{
		super(153);
		mStart = start;
	}
	
	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[2];
		pos = this.writeUint8(pos, data, this.getCode());
		pos = this.writeUint8(pos, data, (mStart)?1:0);
		return data;
	}
}