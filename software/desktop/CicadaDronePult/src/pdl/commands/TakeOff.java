package pdl.commands;

import pdl.DroneCommander;

public class TakeOff
{
	protected int mStartGas = 1000;
	protected int mEndGas = 500;
	protected int mStartTime = 700;
	//protected int mEndGas = 650;
	//protected int mStartTime = 500;
	
	public TakeOff() {}
	
	public void run()
	{
		Thread thread = new Thread(new Runnable()
		{
			@Override
			public void run()
			{
				DroneCommander.instance().addCmd(new CmdSetBaseGas(mStartGas));
				
				try
				{
					Thread.sleep(mStartTime);
				}
				catch(InterruptedException e)
				{
					e.printStackTrace();
				}
				
				DroneCommander.instance().addCmd(new CmdSetBaseGas(mEndGas));
			}
		});
		
		thread.setName("TakeOff");
		thread.start();
	}
	
	public int getEndGas()
	{
		return mEndGas;
	}
}