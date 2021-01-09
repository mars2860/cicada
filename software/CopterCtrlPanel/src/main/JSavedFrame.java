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
public abstract class JSavedFrame extends JFrame
{
	private static final long serialVersionUID = 1L;

	public JSavedFrame(String title, int width, int height)
	{
		super(title);
		
		Settings.WndState ws = loadWndState();
		
		if(ws.w != 0 && ws.h != 0)
		{
			width = ws.w;
			height = ws.h;
			this.setLocation(ws.x, ws.y);
		}
		else
		{
			this.setLocationByPlatform(true);
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
				
				Settings.WndState ws = new Settings.WndState();
				ws.x = location.x;
				ws.y = location.y;
				ws.w = width;
				ws.h = height;
				
				saveWndState(ws);
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
	
	protected abstract Settings.WndState loadWndState();
	protected abstract void saveWndState(Settings.WndState ws);
}
