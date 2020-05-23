package main;

import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
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

	public ImageIcon rotateImageIcon(ImageIcon picture, double angle)
	{
		int w = picture.getIconWidth();
		int h = picture.getIconHeight();
		int type = BufferedImage.TYPE_INT_RGB; // other options, see api
		BufferedImage image = new BufferedImage(h, w, type);
		Graphics2D g2 = image.createGraphics();
		double x = (h - w) / 2.0;
		double y = (w - h) / 2.0;
		AffineTransform at = AffineTransform.getTranslateInstance(x, y);
		at.rotate(Math.toRadians(angle), w / 2.0, h / 2.0);
		g2.setColor(this.getBackground());
		g2.fillRect(0, 0, w, h);
		g2.drawImage(picture.getImage(), at, null);
		g2.dispose();
		picture = new ImageIcon(image);

		return picture;
	}
}
