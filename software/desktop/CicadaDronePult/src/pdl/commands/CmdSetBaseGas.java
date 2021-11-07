package pdl.commands;

public class CmdSetBaseGas extends AbstractCopterCmd
{
	private int mBaseGas;
	
	public CmdSetBaseGas(int gas)
	{
		super(117);
		mBaseGas = gas;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[17];
		
		pos = this.writeUint8(pos, data, this.mCmdCode);
		pos = this.writeInt32(pos, data, mBaseGas);
		
		return data;
	}
}
