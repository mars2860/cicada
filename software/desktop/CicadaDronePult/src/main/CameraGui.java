package main;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.awt.image.BufferStrategy;
import java.io.ByteArrayInputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;

import javax.imageio.ImageIO;

import main.AppSettings.WndState;

import pdl.wlan.PictureBuffer;
import pdl.DroneCommander;

public class CameraGui extends JSavedFrame implements DroneCommander.PictureListener
{
	private static final long serialVersionUID = 8757014849739635545L;
	
	private int mFpsCounter;
	private float mFps;
	private long mFpsTimestamp;
	private long mFrameDelayTimestamp;
	private long mFrameDelay;
	private long mFrameDelaySum;
	private int mFrameDelayCounter;
	private Font mFont;
	private Thread mRenderThread;
	private boolean isRunned;

	private ArrayList<PictureBuffer> mImgList = new ArrayList<PictureBuffer>();
	private Object mImgListSync = new Object();
	private Object mRenderFinishedSync = new Object();
	
	private class OnWndListener implements WindowListener
	{
		@Override
		public void windowOpened(WindowEvent e)
		{
			start();
		}

		@Override
		public void windowClosing(WindowEvent e) 
		{
			stop();
		}

		@Override
		public void windowClosed(WindowEvent e) {}

		@Override
		public void windowIconified(WindowEvent e)
		{
			stop();
		}

		@Override
		public void windowDeiconified(WindowEvent e)
		{
			start();
		}

		@Override
		public void windowActivated(WindowEvent e)
		{
			start();
		}

		@Override
		public void windowDeactivated(WindowEvent e) {}
		
	}
	
	private class RenderThread implements Runnable
	{
		@Override
		public void run()
		{
			PictureBuffer buf = null;
			BufferStrategy strategy = null;
			Image imgFrame = null;
			boolean isNewFrame = false;
			int imgWidth = 0;
			int imgHeight = 0;
			int imgQuality = 0;
			long imgTimestamp = System.currentTimeMillis();
			
			CameraGui.this.createBufferStrategy(2);
			
			strategy = CameraGui.this.getBufferStrategy();

			while(isRunned)
			{
				try
				{
					synchronized(mImgListSync)
					{
						mImgListSync.wait(20);
						
						if(mImgList.isEmpty() == false)
						{
							// get only last image to draw it
							buf = mImgList.get(mImgList.size() - 1);
							// we don't need other images
							mImgList.clear();
							isNewFrame = true;
						}
						else
						{
							isNewFrame = false;
						}
					}
					// convert buffer to java image
					if(isNewFrame && buf != null)
					{
						imgFrame = ImageIO.read(new ByteArrayInputStream(buf.getData()));
						imgWidth = buf.getWidth();
						imgHeight = buf.getHeight();
						imgTimestamp = buf.getTimestamp();
						imgQuality = buf.getQuality();
					}
				}
				catch(Exception e)
				{
					e.printStackTrace();
				}
				
				// Render single frame
			    do
			    {
			    	// The following loop ensures that the contents of the drawing buffer
			        // are consistent in case the underlying surface was recreated
			    	do
			    	{
			    		// Get a new graphics context every time through the loop
			            // to make sure the strategy is validated
			            Graphics graphics = strategy.getDrawGraphics();

			            // Render to graphics
			            // ...
			            drawFrame(graphics,imgFrame,imgQuality,imgWidth,imgHeight,imgTimestamp,isNewFrame);

			            // Dispose the graphics
			            graphics.dispose();

			            // Repeat the rendering if the drawing buffer contents
			            // were restored
			        } while (strategy.contentsRestored());

			        // Display the buffer
			        strategy.show();

			        // Repeat the rendering if the drawing buffer was lost
			    } while(strategy.contentsLost());
			}

			synchronized(mRenderFinishedSync)
			{
				mRenderFinishedSync.notifyAll();
			}
		}
	}

	public CameraGui()
	{
		super("camera", 800, 600);
		
		this.setTitle(ResBox.text("CAMERA"));
		this.setIconImage(ResBox.icon("CAMERA").getImage());
		this.createUI();
	}
	
	private void createUI()
	{
		mFpsTimestamp = System.currentTimeMillis();
		mFont = new Font("Arial", 1, 18);
		
		this.addWindowListener(new OnWndListener());
	}
	
	private void start()
	{
		if(isRunned)
			return;
		
		DroneCommander.instance().setPictureListener(CameraGui.this);
		
		isRunned = true;
		
		mRenderThread = new Thread(new RenderThread(), "CameraRender");
		mRenderThread.start();
	}
	
	private void stop()
	{
		isRunned = false;
		
		synchronized(mRenderFinishedSync)
		{
			try
			{
				mRenderFinishedSync.wait(3000);
			}
			catch(Exception e)
			{
				e.printStackTrace();
			}
		}
		
		DroneCommander.instance().setPictureListener(null);
	}
	
	private void drawFrame(Graphics graphics, Image imgFrame, int quality, int width, int height, long timestamp, boolean isNewFrame)
	{	        
        graphics.setColor(Color.BLACK);
        graphics.fillRect(0,0,this.getWidth(),this.getHeight());

        // Render to graphics
        try
        {
        	if(imgFrame != null)
        	{
        		/*
        		int x = (this.getWidth() - width)/2;
        		int y = (this.getHeight() - height)/2;
       
        		graphics.drawImage(imgFrame, x, y, null);
        		*/
        		float scaleX = (float)this.getWidth() / (float)width;
        		float scaleY = (float)this.getHeight() / (float)height;
        		float scale = scaleX;
        		if(scaleY < scale)
        		{
        			scale = scaleY;
        		}
        		int nWidth = (int)((float)width*scale);
        		int nHeight = (int)((float)height*scale);
         		int x = (this.getWidth() - nWidth)/2;
        		int y = (this.getHeight() - nHeight)/2;
       
        		graphics.drawImage(imgFrame, x, y, nWidth, nHeight, null);
        	}
        	
        	if(isNewFrame)
        	{
        		mFpsCounter++;
        	}
        }
        catch(Exception e)
        {
        	e.printStackTrace();
        }
        
        if(System.currentTimeMillis() - mFpsTimestamp >= 1000)
		{
			mFps = System.currentTimeMillis() - mFpsTimestamp;
			mFps = (float)(mFpsCounter*1000) / mFps;
			mFpsTimestamp = System.currentTimeMillis();
			mFpsCounter = 0;
		}
        
        graphics.setFont(mFont);
        graphics.setColor(Color.WHITE);
        
        DecimalFormat fmt = new DecimalFormat();
        fmt.setMaximumFractionDigits(1);
        
        mFrameDelaySum += (System.currentTimeMillis() - timestamp);
        mFrameDelayCounter++;
        
        if((System.currentTimeMillis() - mFrameDelayTimestamp) >= 250 && mFrameDelayCounter > 0)
        {
        	mFrameDelay = mFrameDelaySum / mFrameDelayCounter;
        	mFrameDelayTimestamp = System.currentTimeMillis();
        	mFrameDelayCounter = 0;
        	mFrameDelaySum = 0;
        }
        
        graphics.drawString("JPEG " + width + "x" + height + " Quality:" + quality + " FPS:" + fmt.format(mFps) + " Delay:" + mFrameDelay, 15, 50);
	}
	
	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getCameraWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setCameraWnd(ws);
	}

	@Override
	public void onPictureReceived(PictureBuffer buf)
	{
		if(buf == null)
			return;
		
    	synchronized(mImgListSync)
    	{
    		mImgList.add(buf.clone());
    		mImgListSync.notifyAll();
    	}
	}
}
