package main;

import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Locale;

import javax.swing.JFrame;
import javax.swing.JOptionPane;

import copter.AlarmCenter;
import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.DroneState;

public class CopterCtrlPanel
{
	private StartGui mMainFrame;
	
	private class OnMainWndListener implements WindowListener
	{
		@Override
		public void windowOpened(WindowEvent e) {}

		@Override
		public void windowClosing(WindowEvent e)
		{
			CopterCtrlPanel.this.stop();
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
		
		AlarmCenter.instance().deleteObservers();
		CopterTelemetry.instance().deleteObservers();
		
		Settings.instance().load();
		ResBox.load();
		
		mMainFrame = new StartGui();
		mMainFrame.setVisible(true);
		mMainFrame.addWindowListener(new OnMainWndListener());
	
		try
		{
			DroneState ds = Settings.instance().getDroneSettings();
			CopterCommander.instance().start(ds.net.ip, ds.net.cmdPort);
			CopterTelemetry.instance().start(ds.net.ip, ds.net.telemetryPort);
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
		CopterCommander.instance().stop();
		CopterTelemetry.instance().stop();
		
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
		CopterCtrlPanel app = new CopterCtrlPanel();
		
		Locale.setDefault(Locale.US);
		
		app.start();
	}
}
