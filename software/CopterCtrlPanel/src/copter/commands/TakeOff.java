package copter.commands;

import copter.CopterCommander;

public class TakeOff
{
	protected int mStartGas = 1500;
	protected int mEndGas = 800;
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
				CopterCommander.instance().addCmd(new CmdSetBaseGas(mStartGas));
				
				try
				{
					Thread.sleep(mStartTime);
				}
				catch(InterruptedException e)
				{
					e.printStackTrace();
				}
				
				CopterCommander.instance().addCmd(new CmdSetBaseGas(mEndGas));
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
