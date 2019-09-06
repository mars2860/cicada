package copter.commands;

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
		byte data[] = {(byte)this.getCode(), (byte)mgas0, (byte)mgas1, (byte)mgas2, (byte)mgas3};
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

}
