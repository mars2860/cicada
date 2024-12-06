package pdl.wlan;

import java.util.Arrays;

public class PictureBuffer implements Cloneable
{
	private int width;
	private int height;
	private int pixelformat;
	private int quality;
	private long timestamp;
	private byte data[];
	private boolean isLocked;
	private boolean isFilled;
	private int pos;
	
	public PictureBuffer()
	{
		
	}
	
	public void start(int w, int h, int fmt, int qual, long tm, int len, byte dataChunk[])
	{
		if(isLocked)
			return;
		
		isFilled = true;
		
		pos = 0;
		width = w;
		height = h;
		pixelformat = fmt;
		quality = qual;
		timestamp = tm;
		data = new byte[len];
		for(int i = 0; i < dataChunk.length; i++)
		{
			append(dataChunk[i]);
		}
	}
	
	public void append(byte b)
	{
		if(!isFilled || isLocked || pos >= data.length)
			return;
		
		data[pos] = b;
		pos++;
		
		if(pos >= data.length)
		{
			isFilled = false;
		}
	}
	
	public void stop()
	{
		isFilled = false;
	}
	
	public boolean isReadyDraw()
	{
		return isFilled == false && data != null && data.length > 0 && pos == data.length;
	}
	
	public boolean isLocked()
	{
		return this.isLocked;
	}
	
	public void lock()
	{
		isLocked = true;
	}
	
	public void unlock()
	{
		isLocked = false;
	}
	
	public boolean isFilled()
	{
		return this.isFilled;
	}
	
	public int getWidth()
	{
		return width;
	}
	
	public int getHeight()
	{
		return height;
	}
	
	public int getPixelFormat()
	{
		return pixelformat;
	}
	
	public int getQuality()
	{
		return quality;
	}
	
	public long getTimestamp()
	{
		return timestamp;
	}
	
	public byte[] getData()
	{
		return data;
	}
	
	public PictureBuffer clone()
	{
		PictureBuffer buf = new PictureBuffer();
		buf.width = this.width;
		buf.height = this.height;
		buf.pixelformat = this.pixelformat;
		buf.quality = this.quality;
		buf.pos = this.pos;
		buf.timestamp = this.timestamp;
		buf.data = Arrays.copyOf(this.data, this.data.length);
		return buf;
	}
}
