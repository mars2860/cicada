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
	private int pos;
	private boolean started;
	private boolean aborted;
	
	public PictureBuffer()
	{
		
	}
	
	public void start(int w, int h, int fmt, int qual, long tm, int len, byte dataChunk[])
	{
		started = true;
		aborted = false;
		
		pos = 0;
		width = w;
		height = h;
		pixelformat = fmt;
		quality = qual;
		timestamp = tm;
		if(len == 0)
		{
			len = 240000;
		}
		data = new byte[len];
		for(int i = 0; i < dataChunk.length; i++)
		{
			append(dataChunk[i]);
		}
	}
	
	public void append(byte b)
	{
		if(started == false || aborted || pos >= data.length)
		{
			return;
		}
		
		data[pos] = b;
		pos++;
		
		if(pos >= data.length)
		{
			stop();
		}
	}
	
	public void stop()
	{
		started = false;
		
		if(pos < data.length)
		{
			data = Arrays.copyOf(data,pos);
		}
	}
	
	public void abort()
	{
		aborted = true;
	}
	
	public boolean isReadyDraw()
	{
		return !started && !aborted && data != null && data.length > 0 && pos == data.length;
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
