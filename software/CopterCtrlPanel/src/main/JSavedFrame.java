package main;

import java.awt.Point;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;

import javax.swing.JFrame;

/** Saves own position to settings */
public class JSavedFrame extends JFrame
{
	private static final long serialVersionUID = 1L;
	
	private String KEY_X;
	private String KEY_Y;
	private String KEY_WIDTH;
	private String KEY_HEIGHT;
	
	public JSavedFrame(String title, int width, int height)
	{
		super(title);
		
		KEY_X = title + ".x";
		KEY_Y = title + ".y";
		KEY_WIDTH = title + ".width";
		KEY_HEIGHT = title + ".height";
		
		int x = Settings.instance().getIntProperty(KEY_X);
		int y = Settings.instance().getIntProperty(KEY_Y);
		int w = Settings.instance().getIntProperty(KEY_WIDTH);
		int h = Settings.instance().getIntProperty(KEY_HEIGHT);
		
		if(w != 0 && h != 0)
		{
			width = w;
			height = h;
			this.setLocation(x, y);
		}
		
		this.setSize(width, height);
		this.setAlwaysOnTop(true);
		this.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		
		this.addWindowListener(new WindowListener()
		{
			@Override
			public void windowOpened(WindowEvent e) {}

			@Override
			public void windowClosing(WindowEvent e)
			{
				Point location = JSavedFrame.this.getLocation();
				int width = JSavedFrame.this.getWidth();
				int height = JSavedFrame.this.getHeight();
				
				Settings.instance().setProperty(KEY_X, Integer.toString(location.x));
				Settings.instance().setProperty(KEY_Y, Integer.toString(location.y));
				Settings.instance().setProperty(KEY_WIDTH, Integer.toString(width));
				Settings.instance().setProperty(KEY_HEIGHT, Integer.toString(height));
			}

			@Override
			public void windowClosed(WindowEvent e) {}

			@Override
			public void windowIconified(WindowEvent e) {}

			@Override
			public void windowDeiconified(WindowEvent e) {}

			@Override
			public void windowActivated(WindowEvent e) {}

			@Override
			public void windowDeactivated(WindowEvent e) {}
		});
	}
}
