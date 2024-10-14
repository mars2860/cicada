import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JToggleButton;
import javax.swing.event.PopupMenuEvent;
import javax.swing.event.PopupMenuListener;

import com.fazecast.jSerialComm.SerialPort;

import net.miginfocom.swing.MigLayout;
import pdl.wlan.WifiBroadcastModem;
import pdl.wlan.WifiBroadcastModem.Message;
import pdl.wlan.WifiBroadcastModem.ModemState;

public class TestWifiBroadcastModemApp
{
	private JFrame wndMain;
	private JComboBox<SerialPort> cbPorts;
	private JToggleButton btnConnect;
	private JButton btnCmdVersion;
	private JButton btnCmdStop;
	private JButton btnCmdStartSniffer;
	private JButton btnCmdStartNormal;
	private JButton btnCmdSetChannel;
	private JComboBox<Integer> cbChannel;
	private JButton btnCmdSetMaxTpw;
	private JComboBox<Double> cbMaxTpw;
	private JButton btnCmdSetWifiRate;
	private JComboBox<WifiBroadcastModem.WifiRate> cbWifiRate;
	private JButton btnCmdSetWifiPhy;
	private JComboBox<WifiBroadcastModem.WifiPhy> cbWifiPhy;
	private JTextArea taConsole;
	private JCheckBox chbDebug;
	private JButton btnClear;
	private JButton btnSayHello;
	
	private WifiBroadcastModem modem;
	private Thread thRx;
	
	private class OnSerialRx implements Runnable
	{
		@Override
		public void run()
		{
			while(modem != null && modem.getState() != ModemState.DISCONNECTED)
			{
				modem.receive(100);
				try
				{
					Thread.sleep(10);
				}
				catch(Exception e)
				{
					e.printStackTrace();
				}
			}
		}	
	}
	
	public class OnSelectComPort implements PopupMenuListener
	{
		@Override
		public void popupMenuWillBecomeVisible(PopupMenuEvent e)
		{
			findComPorts();
		}

		@Override
		public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {}

		@Override
		public void popupMenuCanceled(PopupMenuEvent e) {}
	}
	
	public class MyModemListener implements WifiBroadcastModem.ModemListener
	{
		@Override
		public void onRawDataReceived(byte[] data)
		{
			String text = "Received " + data.length + " bytes = {" +
					WifiBroadcastModem.byteArrayToHex(data) + "}\n";
			text += new String(data);
			text += "\n";
			
			if(chbDebug.isSelected())
			{
				taConsole.append(text);
			}
		}
		
		@Override
		public void onMessageReceived(Message msg)
		{
			String text = "Received modem msg: " + msg.toString() + "\n";
			taConsole.append(text);
			
			if(msg instanceof WifiBroadcastModem.ModemStateMsg)
			{
				cbChannel.setSelectedIndex(modem.getWifiChannel() - 1);
				cbMaxTpw.setSelectedIndex(modem.getWifiMaxTpw());
				cbWifiRate.setSelectedItem(modem.getWifiRate());
				cbWifiPhy.setSelectedItem(modem.getWifiPhyMode());
			}
		}
	}
	
	public class OnBtnConnect implements ItemListener
	{
		@Override
		public void itemStateChanged(ItemEvent e)
		{
			if(e.getStateChange() == ItemEvent.SELECTED)
			{
				connect();
			}
			else if(e.getStateChange() == ItemEvent.DESELECTED)
			{
				disconnect();
			}
		}
	}
		
	public class OnBtnCmdVersion implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(modem == null)
				return;
			
			modem.ping(100);
		}
	}
	
	public class OnBtnCmdStop implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(modem == null)
				return;
			
			modem.stop();
		}
	}
	
	public class OnBtnCmdStartSniffer implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(modem == null)
				return;
			
			modem.startSniffer();
		}
	}
	
	public class OnBtnCmdStartNormal implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(modem == null)
				return;
			
			modem.startNormal();
		}
	}
	
	public class OnBtnCmdSetChannel implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(modem == null)
				return;
			
			modem.setChannel(cbChannel.getSelectedIndex() + 1);
		}
	}
	
	public class OnBtnCmdSetMaxTpw implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(modem == null)
				return;
			
			modem.setMaxTpw(cbMaxTpw.getSelectedIndex());
		}
	}
	
	public class OnBtnCmdSetWifiRate implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(modem == null)
				return;
			
			modem.setWifiRate((WifiBroadcastModem.WifiRate)cbWifiRate.getSelectedItem());
		}
	}
	
	public class OnBtnCmdSetWifiPhy implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(modem == null)
				return;
			
			modem.setWifiPhy((WifiBroadcastModem.WifiPhy)cbWifiPhy.getSelectedItem());
		}
	}
	
	public class OnBtnClear implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			taConsole.setText("");
		}
	}
	
	public class OnBtnSayHello implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(modem == null)
				return;
			
			String hello = "Hello World!";
			
			modem.send(hello.getBytes());
		}
	}
	
	public class OnWndListener implements WindowListener
	{
		@Override
		public void windowOpened(WindowEvent e) {}

		@Override
		public void windowClosing(WindowEvent e) {}

		@Override
		public void windowClosed(WindowEvent e) 
		{
			disconnect();
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
		
	public void findComPorts()
	{
		cbPorts.removeAllItems();
		
		for(SerialPort port : SerialPort.getCommPorts())
		{
			cbPorts.addItem(port);
		}
	}
	
	public void createUI()
	{
		wndMain = new JFrame();
		wndMain.setTitle("TestWifiBroadcastModemApp");
		wndMain.setSize(800,600);
		wndMain.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		wndMain.setLocationByPlatform(true);
		
		JPanel pnlMain = new JPanel(new MigLayout("","[grow]","[][][grow]"));
		
		taConsole = new JTextArea();
		JScrollPane spConsole = new JScrollPane(	taConsole,
													JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED,
													JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
		
		cbPorts = new JComboBox<SerialPort>();
		cbPorts.addPopupMenuListener(new OnSelectComPort());
		btnConnect = new JToggleButton("CONNECT");
		btnConnect.addItemListener(new OnBtnConnect());

		JPanel pnlPorts = new JPanel(new MigLayout("","[][]"));
		pnlPorts.add(cbPorts,"w 280");
		pnlPorts.add(btnConnect);
		
		chbDebug = new JCheckBox("DEBUG");
		
		btnCmdVersion = new JButton("Version");
		btnCmdVersion.addActionListener(new OnBtnCmdVersion());
		
		btnCmdStop = new JButton("Stop");
		btnCmdStop.addActionListener(new OnBtnCmdStop());
		
		btnCmdStartSniffer = new JButton("Start Sniffer");
		btnCmdStartSniffer.addActionListener(new OnBtnCmdStartSniffer());
		
		btnCmdStartNormal = new JButton("Start Normal");
		btnCmdStartNormal.addActionListener(new OnBtnCmdStartNormal());
		
		btnCmdSetChannel = new JButton("Set Channel");
		btnCmdSetChannel.addActionListener(new OnBtnCmdSetChannel());
		
		btnCmdSetMaxTpw = new JButton("Set Tx Power");
		btnCmdSetMaxTpw.addActionListener(new OnBtnCmdSetMaxTpw());
		
		btnCmdSetWifiRate = new JButton("Set Rate");
		btnCmdSetWifiRate.addActionListener(new OnBtnCmdSetWifiRate());
		
		btnCmdSetWifiPhy = new JButton("Set PHY");
		btnCmdSetWifiPhy.addActionListener(new OnBtnCmdSetWifiPhy());
		
		btnClear = new JButton("Clear");
		btnClear.addActionListener(new OnBtnClear());
		
		btnSayHello = new JButton("Say Hello");
		btnSayHello.addActionListener(new OnBtnSayHello());
		
		cbChannel = new JComboBox<Integer>();
		for(int i = 1; i <= 14; i++)
		{
			cbChannel.addItem(new Integer(i));
		}
		
		cbMaxTpw = new JComboBox<Double>();
		for(double i = 0; i <= 20.5; i += 0.25)
		{
			cbMaxTpw.addItem(new Double(i));
		}
		
		cbWifiRate = new JComboBox<WifiBroadcastModem.WifiRate>();
		for(WifiBroadcastModem.WifiRate rate : WifiBroadcastModem.WifiRate.values())
		{
			if(rate != WifiBroadcastModem.WifiRate.UNKNOWN)
			{
				cbWifiRate.addItem(rate);
			}
		}
		
		cbWifiPhy = new JComboBox<WifiBroadcastModem.WifiPhy>();
		for(WifiBroadcastModem.WifiPhy phy : WifiBroadcastModem.WifiPhy.values())
		{
			if(phy != WifiBroadcastModem.WifiPhy.UNKNOWN)
			{
				cbWifiPhy.addItem(phy);
			}
		}
		
		JPanel pnlButtons = new JPanel(new MigLayout("","[][]"));

		pnlButtons.add(new JLabel("MODEM COMMANDS:"),"wrap");
		pnlButtons.add(chbDebug,"wrap");
		pnlButtons.add(btnCmdVersion,"growx,wrap");
		pnlButtons.add(btnCmdStop,"growx,wrap");
		pnlButtons.add(btnCmdStartSniffer,"growx,wrap");
		pnlButtons.add(btnCmdStartNormal,"growx,wrap");
		pnlButtons.add(btnCmdSetChannel,"growx");
		pnlButtons.add(cbChannel,"growx,wrap");
		pnlButtons.add(btnCmdSetMaxTpw,"growx");
		pnlButtons.add(cbMaxTpw,"growx,wrap");
		pnlButtons.add(btnCmdSetWifiRate,"growx");
		pnlButtons.add(cbWifiRate,"growx,wrap");
		pnlButtons.add(btnCmdSetWifiPhy,"growx");
		pnlButtons.add(cbWifiPhy,"growx,wrap");
		pnlButtons.add(btnSayHello,"growx,wrap");
		pnlButtons.add(btnClear,"growx,wrap");
		
		pnlMain.add(pnlPorts,"wrap");
		pnlMain.add(pnlButtons,"wrap");
		pnlMain.add(spConsole,"grow");
		
		wndMain.add(pnlMain);
		wndMain.addWindowListener(new OnWndListener());
	}
	
	private void connect()
	{
		disconnect();
		
		SerialPort comPort = (SerialPort)cbPorts.getSelectedItem();
		
		if(comPort != null)
		{
			modem = new WifiBroadcastModem(comPort);
			if(modem.connect() == false)
			{
				modem = null;
				btnConnect.setSelected(false);
				return;
			}
			modem.setModemListener(new MyModemListener());
			thRx = new Thread(new OnSerialRx(), "WifiBroadcastModemRx");
			thRx.start();
			btnConnect.setText("DISCONNECT");
		}
		else
		{
			modem = null;
			btnConnect.setSelected(false);
		}	
	}
	
	private void disconnect()
	{
		if(modem != null)
		{
			modem.disconnect();
			
			try
			{
				if(thRx != null)
				{
					thRx.join(3000);
				}
			}
			catch(Exception ex)
			{
				ex.printStackTrace();
			}
		}
		modem = null;
		thRx = null;
		btnConnect.setText("CONNECT");
	}
	
	public void go()
	{
		createUI();
		findComPorts();
		wndMain.setVisible(true);
	}

	public static void main(String[] args)
	{
		TestWifiBroadcastModemApp app = new TestWifiBroadcastModemApp();
		
		app.go();
	}
}
