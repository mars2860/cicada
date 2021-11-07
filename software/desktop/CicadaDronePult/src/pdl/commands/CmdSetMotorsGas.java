package pdl.commands;

public class CmdSetMotorsGas extends AbstractCopterCmd
{
	private int mgas0;
	private int mgas1;
	private int mgas2;
	private int mgas3;
	
	public CmdSetMotorsGas()
	{
		super(101);
	}
	
	public CmdSetMotorsGas(int gas0, int gas1, int gas2, int gas3)
	{
		super(101);
		
		mgas0 = gas0;
		mgas1 = gas1;
		mgas2 = gas2;
		mgas3 = gas3;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[17];
		
		pos = this.writeUint8(pos, data, this.mCmdCode);
		pos = this.writeInt32(pos, data, mgas0);
		pos = this.writeInt32(pos, data, mgas1);
		pos = this.writeInt32(pos, data, mgas2);
		pos = this.writeInt32(pos, data, mgas3);
		
		return data;
	}
	
	public void setGas0(int gas)
	{
		mgas0 = gas;
	}
	
	public int getGas0()
	{
		return mgas0;
	}
	
	public void setGas1(int gas)
	{
		mgas1 = gas;
	}
	
	public int getGas1()
	{
		return mgas1;
	}
	
	public void setGas2(int gas)
	{
		mgas2 = gas;
	}
	
	public int getGas2()
	{
		return mgas2;
	}
	
	public void setGas3(int gas)
	{
		mgas3 = gas;
	}
	
	public int getGas3()
	{
		return mgas3;
	}
	
	public void setGas(int chl, int gas)
	{
		switch(chl)
		{
		case 0:
			mgas0 = gas;
			break;
		case 1:
			mgas1 = gas;
			break;
		case 2:
			mgas2 = gas;
			break;
		case 3:
			mgas3 = gas;
			break;
		}
	}
	
	public void setGasForAll(int gas)
	{
		mgas0 = gas;
		mgas1 = gas;
		mgas2 = gas;
		mgas3 = gas;
	}
}
