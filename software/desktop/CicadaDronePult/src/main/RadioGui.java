package main;

import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JLabel;
import javax.swing.JPanel;

import main.AppSettings.WndState;
import net.miginfocom.swing.MigLayout;
import pdl.DroneCommander;
import pdl.DroneTelemetry;

public class RadioGui extends JSavedFrame
{
	private static final long serialVersionUID = 283210269049690555L;
	
	private JLabel mlbRssi;
	private JLabel mlbRxPackets;
	private JLabel mlbLostRxPackets;
	private JLabel mlbTxPackets;
	private JLabel mlbLatency;
	
	private Timer mTimer;
	
	private class OnUpdate extends TimerTask
	{
		@Override
		public void run()
		{
			if(DroneTelemetry.instance().isDroneConnected())
			{
				mlbRssi.setText(Integer.toString((int)DroneTelemetry.instance().getDroneState().rssi));
			}
			else
			{
				mlbRssi.setText("0");
			}
			
			mlbRxPackets.setText(Integer.toString(DroneCommander.instance().getRxPacketCounter()));
			mlbLostRxPackets.setText(Integer.toString(DroneCommander.instance().getLostRxPacketCounter()));
			mlbTxPackets.setText(Integer.toString(DroneCommander.instance().getTxPacketCounter()));
			mlbLatency.setText(Integer.toString(DroneCommander.instance().getWlanLatency()));
		}
	}
	
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
		public void windowIconified(WindowEvent e) {}

		@Override
		public void windowDeiconified(WindowEvent e) {}

		@Override
		public void windowActivated(WindowEvent e) {}

		@Override
		public void windowDeactivated(WindowEvent e) {}
		
	}
	
	public RadioGui()
	{
		super("radio",330,330);
		this.setTitle(ResBox.text("RADIO_STATUS"));
		this.setIconImage(ResBox.icon("RADIO").getImage());
		this.createUI();
		
		this.addWindowListener(new OnWndListener());
	}
	
	private void createUI()
	{
		mlbRssi = new JLabel();
		mlbRxPackets = new JLabel();
		mlbLostRxPackets = new JLabel();
		mlbTxPackets = new JLabel();
		mlbLatency = new JLabel();
		
		JPanel pnl = new JPanel(new MigLayout("","[][100]"));

		pnl.add(new JLabel("RSSI: "));
		pnl.add(mlbRssi,"wrap");
		pnl.add(new JLabel("RxPackets: "));
		pnl.add(mlbRxPackets,"wrap");
		pnl.add(new JLabel("LostRxPackets: "));
		pnl.add(mlbLostRxPackets,"wrap");
		pnl.add(new JLabel("TxPackets: "));
		pnl.add(mlbTxPackets,"wrap");
		pnl.add(new JLabel("Latency: "));
		pnl.add(mlbLatency,"wrap");

		this.add(pnl);
	}
	
	@Override
	public void setVisible(boolean b)
	{
		super.setVisible(b);
		
		if(b)
		{
			start();
		}
		else
		{
			stop();
		}
	}
	
	public void start()
	{
		if(mTimer != null)
		{
			mTimer.cancel();
		}
		
		mTimer = new Timer();
		mTimer.scheduleAtFixedRate(new OnUpdate(),0,1000);
	}
	
	public void stop()
	{
		if(mTimer != null)
		{
			mTimer.cancel();
			mTimer = null;
		}
	}
	
	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getRadioWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setRadioWnd(ws);	
	}
}
