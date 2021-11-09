package main;

import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Locale;

import javax.swing.JFrame;
import javax.swing.JOptionPane;

import pdl.DroneAlarmCenter;
import pdl.DroneCommander;
import pdl.DroneTelemetry;
import pdl.DroneState;

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
		public void windowClosed(WindowEvent e) {}

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
		
		Settings.instance().load();
		ResBox.load();
		
		mMainFrame = new StartGui();
		mMainFrame.setVisible(true);
		mMainFrame.addWindowListener(new OnMainWndListener());
	
		try
		{
			DroneState ds = Settings.instance().getDroneSettings();
			DroneCommander.instance().start(ds.net.ip, ds.net.cmdPort);
			DroneTelemetry.instance().start(ds.net.ip, ds.net.telemetryPort);
		}
		catch(UnknownHostException e)
		{
			showErrorMsg(ResBox.text("INVALID_HOST"));
			e.printStackTrace();
		}
		catch(SocketException e)
		{
			showErrorMsg(ResBox.text("SOCKET_NOT_OPEN"));
			e.printStackTrace();
		}
	}
	
	public void stop()
	{
		DroneCommander.instance().stop();
		DroneTelemetry.instance().stop();
		
		if(mMainFrame != null)
		{
			mMainFrame.setVisible(false);
			mMainFrame = null;
		}
		
		Settings.instance().save();
	}
	
	private void showErrorMsg(String text)
	{
		JOptionPane.showMessageDialog(mMainFrame, text, ResBox.text("ERROR"), JOptionPane.ERROR_MESSAGE);
	}

	public static void main(String[] args)
	{
		CicadaDronePultApp app = new CicadaDronePultApp();
		
		Locale.setDefault(Locale.US);
		
		app.start();
	}
}
