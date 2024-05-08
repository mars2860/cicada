package pdl;

public class Accelerator
{
	protected double mValue;
	protected double mMinValue;
	protected double mMidValue;
	protected double mMaxValue;
	protected double mUpAccel;
	protected double mDownAccel;
	protected double mTime;
	
	public Accelerator(double minValue, double midValue, double maxValue, double time)
	{
		mMinValue = minValue;
		mMidValue = midValue;
		mMaxValue = maxValue;
		mTime = time;
		
		mUpAccel = (maxValue - midValue);
		mDownAccel = (minValue - midValue);
				
		if(time > 0)
		{
			 mUpAccel /= time;
			 mDownAccel /= time;
		}
	}
	
	public double accelerate(double dt)
	{
		if(mTime == 0)
		{
			mValue = mMaxValue;
		}
		else
		{
			mValue += mUpAccel*dt;
		
			if(mValue > mMaxValue)
				mValue = mMaxValue;
		}
		
		return mValue;
	}
	
	public double decelerate(double dt)
	{
		if(mTime == 0)
		{
			mValue = mMinValue;
		}
		else
		{
			mValue += mDownAccel*dt;
		
			if(mValue < mMinValue)
				mValue = mMinValue;
		}
		
		return mValue;
	}
	
	public double normalize(double dt)
	{
		if(mTime == 0)
		{
			mValue = mMidValue;
		}
		else
		{
			if(mValue > mMidValue)
			{
				if(Math.abs(mValue - mMidValue) < Math.abs(mDownAccel / 10.0) )
				{
					mValue = mMidValue;
				}
				else
				{
					decelerate(dt);
				}
			}
		
			if(mValue < mMidValue)
			{
				if(Math.abs(mValue - mMidValue) < Math.abs(mUpAccel / 10.0) )
				{
					mValue = mMidValue;
				}
				else
				{
					accelerate(dt);
				}
			}
		}
		
		return mValue;
	}
	
	public double getValue()
	{
		return mValue;
	}
	
	@Override
	public boolean equals(Object obj)
	{
		if(obj instanceof Accelerator)
		{
			Accelerator accel = (Accelerator)obj;
			
			if(	accel.mMidValue == mMidValue &&
			    accel.mMaxValue == mMaxValue &&
			    accel.mMinValue == mMinValue &&
			    accel.mTime == mTime)
			{
				return true;
			}
		}
		
		return false;
	}
	
	public double getMidValue()
	{
		return mMidValue;
	}
	
	public double getAccelTime()
	{
		return mTime;
	}
	
	public void reset()
	{
		mValue = mMidValue;
	}
}
