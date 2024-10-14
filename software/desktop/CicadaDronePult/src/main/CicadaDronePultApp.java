package main;

import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.File;
import java.util.Locale;

import javax.swing.JFrame;
import javax.swing.JOptionPane;

import pdl.DroneAlarmCenter;
import pdl.DroneCommander;
import pdl.DroneTelemetry;
import pdl.res.Profile;

public class CicadaDronePultApp
{
	private StartGui mMainFrame;
	
	private class OnMainWndListener implements WindowListener
	{
		@Override
		public void windowOpened(WindowEvent e) {}

		@Override
		public void windowClosing(WindowEvent e)
		{
			CicadaDronePultApp.this.stop();
		}

		@Override
		public void windowClosed(WindowEvent e) 
		{
			//CicadaDronePultApp.this.stop();	// sometimes doesn't work ?!
		}

		@Override
		public void windowIconified(WindowEvent e) {}

		@Override
		public void windowDeiconified(WindowEvent e) {}

		@Override
		public void windowActivated(WindowEvent e) {}

		@Override
		public void windowDeactivated(WindowEvent e) {}	
	}
	
	public JFrame getMainFrame()
	{
		return mMainFrame;
	}

	public void start()
	{
		if(mMainFrame != null)
			return;
		
		DroneAlarmCenter.instance().deleteObservers();
		DroneTelemetry.instance().deleteObservers();
		
		AppSettings.load();
		ResBox.load();
		
		File file = AppSettings.instance().getCurProfileFile();
		if(Profile.load(file) == false)
		{
			JOptionPane.showMessageDialog(	null,
											ResBox.text("CANT_LOAD_PROFILE") + ":" + file.getAbsolutePath(),
											ResBox.text("ERROR"),
											JOptionPane.ERROR_MESSAGE);
			return;
		}
		
		mMainFrame = new StartGui();
		mMainFrame.setVisible(true);
		mMainFrame.addWindowListener(new OnMainWndListener());
	}
	
	public void stop()
	{
		DroneAlarmCenter.instance().deleteObservers();
		DroneTelemetry.instance().deleteObservers();
		
		DroneCommander.instance().disconnect();
		
		if(mMainFrame != null)
		{
			mMainFrame.setVisible(false);
			mMainFrame = null;
		}
		
		AppSettings.instance().save();
	}
	
	public static void showErrorMsg(java.awt.Component parent, String text)
	{
		JOptionPane.showMessageDialog(parent, text, ResBox.text("ERROR"), JOptionPane.ERROR_MESSAGE);
	}

	public static void main(String[] args)
	{
		CicadaDronePultApp app = new CicadaDronePultApp();
		
		Locale.setDefault(Locale.US);
		
		app.start();
	}
}
