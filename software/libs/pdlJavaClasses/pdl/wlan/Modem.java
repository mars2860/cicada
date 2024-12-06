package pdl.wlan;

public abstract class Modem
{
	public abstract boolean connect();
	public abstract void disconnect();
	public abstract boolean ping(int timeout);
	public abstract boolean send(byte data[]);
	public abstract byte[] receive(int timeout);
	
	protected int mRxPacketsNum;
	protected int mTxPacketsNum;
	protected int mBitrateSum;
	protected int mBitrate;
	protected long mBitrateTimestamp;
	
	protected void clearStat()
	{
		mRxPacketsNum = 0;
		mTxPacketsNum = 0;
		mBitrate = 0;
		mBitrateSum = 0;
		mBitrateTimestamp = System.currentTimeMillis();
	}
	
	public int getRxPacketsNum()
	{
		return mRxPacketsNum;
	}
	
	public int getTxPacketsNum()
	{
		return mTxPacketsNum;
	}
	
	public int getBitrate()
	{
		return mBitrate*8;
	}
	
	protected void updateBitrate(int dataSize)
	{
		mBitrateSum += dataSize;
		
		long dt = System.currentTimeMillis() - mBitrateTimestamp; 
		if(dt >= 1000)
		{
			mBitrate = (int)(((long)mBitrateSum*1000) / dt);
			mBitrateSum = 0;
			mBitrateTimestamp = System.currentTimeMillis();
			
		}
	}
}
