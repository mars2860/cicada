package pdl.wlan;

import java.text.DecimalFormat;
import java.util.Arrays;

import com.fazecast.jSerialComm.SerialPort;

/**
 *  @note Maximum bitrate 53 kbyte/s (can receive telemetry packets every 20ms)
 */
public class WifiBroadcastModem extends Modem
{
	private static final int BAUDRATE = 921600;
	
	private static final byte MSG_FIRST_BYTE = 0x3D;
	private static final byte MSG_SECOND_BYTE = 0x62;
	private static final byte MSG_FUNC_MODEM_STATE = 0x01;
	private static final byte MSG_FUNC_START_NORMAL = 0x02;
	private static final byte MSG_FUNC_START_SNIFFER = 0x03;
	private static final byte MSG_FUNC_STOP = 0x04;
	private static final byte MSG_FUNC_DATA = 0x05;
	private static final byte MSG_FUNC_SET_CHANNEL = 0x06;
	private static final byte MSG_FUNC_SET_TX_POWER = 0x07;
	private static final byte MSG_FUNC_SET_TX_RATE = 0x08;
	private static final byte MSG_FUNC_SET_PHY_MODE = 0x09;
	private static final byte MSG_FUNC_LOG = 0x0A;
	
	// change these constants if you have added new modem function
	private static final byte MSG_FIRST_FUNC = MSG_FUNC_MODEM_STATE;
	private static final byte MSG_LAST_FUNC = MSG_FUNC_LOG;
	
	private SerialPort comPort;
	private int version;
	private int wifiChannel;
	private int wifiMaxTpw;
	private WifiPhy wifiPhy;
	private WifiRate wifiRate;
	
	private ModemListener listener;
	
	private ModemState mdmState = ModemState.DISCONNECTED;
	
	private Object rxLock = new Object();
	private Object txLock = new Object();
	
	public enum ModemState
	{
		DISCONNECTED(0),
		STOPPED(1),
		NORMAL(2),
		SNIFFER(3),
		UNKNOWN(0xFF);
		
		private int code;
		
		private ModemState(int i)
		{
			code = i;
		}
		
		public static ModemState fromeInt(int i)
		{
			ModemState st = ModemState.UNKNOWN;
			for(ModemState s : ModemState.values())
			{
				if(s.code == i)
				{
					st = s;
					break;
				}
			}
			return st;
		}
		
		public int toInt()
		{
			return code;
		}
	}
	
	public enum WifiRate
	{
		CCK_1M_11B(0x0),
		CCK_2M_11B(0x1),
		CCKS_2M_11G(0x5),
		CCK_5D5M_11B(0x2),
		CCKS_5D5M_11G(0x6),
		CCK_11M_11B(0x3),
		CCKS_11M_11G(0x7),
		ODFM_6M_11G(0xB),
		ODFM_9M_11G(0xF),
		ODFM_12M_11G(0xA),
		ODFM_18M_11G(0xE),
		ODFM_24M_11G(0x9),
		ODFM_36M_11G(0xD),
		ODFM_48M_11G(0x8),
		ODFM_54M_11G(0xC),
		UNKNOWN(0xFF);
		
		private int code;
		
		private WifiRate(int val)
		{
			code = val;
		}
		
		public static WifiRate fromeInt(int i)
		{
			WifiRate rate = WifiRate.UNKNOWN;
			for(WifiRate r : WifiRate.values())
			{
				if(r.code == i)
				{
					rate = r;
					break;
				}
			}
			return rate;
		}
		
		public int toInt()
		{
			return code;
		}
	}
	
	public enum WifiPhy
	{
		IEEE_802_11B(1),
		IEEE_802_11G(2),
		IEEE_802_11N(3),
		UNKNOWN(0xFF);
		
		private int code;
		
		private WifiPhy(int i)
		{
			code = i;
		}
		
		public static WifiPhy fromeInt(int i)
		{
			WifiPhy phy = WifiPhy.UNKNOWN;
			for(WifiPhy p : WifiPhy.values())
			{
				if(p.code == i)
				{
					phy = p;
					break;
				}
			}
			return phy;
		}
		
		public int toInt()
		{
			return code;
		}
	}
	
	public interface ModemListener
	{
		public void onMessageReceived(Message msg);
		public void onRawDataReceived(byte[] data);
	}
	
	public abstract class Message
	{
		protected int mFunc;
		
		public Message(int func)
		{
			mFunc = func;
		}
		
		public int getFunc()
		{
			return mFunc;
		}
	}
	
	public class ModemStateMsg extends Message
	{
		public ModemStateMsg(int ver, int state, int chl, int maxTpw, int rate, int phy)
		{
			super(MSG_FUNC_MODEM_STATE);
			version = ver;
			mdmState = ModemState.fromeInt(state);
			wifiChannel = chl;
			wifiMaxTpw = maxTpw;
			wifiRate = WifiRate.fromeInt(rate);
			wifiPhy = WifiPhy.fromeInt(phy);
		}
				
		@Override
		public String toString()
		{
			DecimalFormat fmt = new DecimalFormat();
			fmt.setMaximumFractionDigits(2);
			fmt.setGroupingUsed(false);
			double txPwr = (double)(wifiMaxTpw)*0.25;
			txPwr = dbmToWatts(txPwr)*1000.0;
			return "MODEM_STATE: fwVer=" +  version + ", state=" + mdmState + 
					", channel=" + wifiChannel +
					", txPower=" + fmt.format(txPwr) + "mW" +
					", rate=" + wifiRate + ", phy=" + wifiPhy;
		}
	}
	
	public class ModemDataMsg extends Message
	{
		private byte data[];
		
		public ModemDataMsg(byte data[])
		{
			super(MSG_FUNC_DATA);
			this.data = data;
		}
		
		@Override
		public String toString()
		{
			// this method is needed only for debug
			
			String result = "DATA_PACKET len=" + data.length;
			
			if(mdmState == ModemState.SNIFFER)
			{
				result = "SNIFFER_PACKET len=" + data.length;
				if(data.length == 128)
				{
					result += "; WIFI_MANAGMENT_FRAME, addr1=";
					for(int i = 0; i < 6; i++)
					{
						result += Integer.toHexString(data[16 + i]) + " ";
					}
				}
			}
			else if(data.length <= 30)
			{
				String dataAscii = new String(this.data);
				result += ",dataAscii=" + dataAscii;
			}
			
			return result;
		}
		
		public byte[] getData()
		{
			return this.data;
		}
	}
	
	public class ModemLogMsg extends Message
	{
		private byte data[];
		
		public ModemLogMsg(byte data[])
		{
			super(MSG_FUNC_LOG);
			this.data = data;
		}
		
		@Override
		public String toString()
		{		
			String result = new String(this.data);
			return result;
		}
		
		public byte[] getData()
		{
			return this.data;
		}
	}
	
	public WifiBroadcastModem(SerialPort port)
	{
		comPort = port;
		
		wifiPhy = WifiPhy.IEEE_802_11G;
		wifiRate = WifiRate.ODFM_6M_11G;
		wifiChannel = 7;
		wifiMaxTpw = 80;
	}
		
	public void setModemListener(ModemListener listener)
	{
		this.listener = listener;
	}
	
	public int getVersion()
	{
		return version;
	}
	
	public int getWifiChannel()
	{
		return wifiChannel;
	}
	
	public int getWifiMaxTpw()
	{
		return wifiMaxTpw;
	}
	
	public WifiRate getWifiRate()
	{
		return wifiRate;
	}
	
	public WifiPhy getWifiPhyMode()
	{
		return wifiPhy;
	}
	
	public ModemState getState()
	{
		return mdmState;
	}
	
	@Override
	public boolean connect()
	{
		boolean conn = false;
		if(mdmState != ModemState.DISCONNECTED)
		{
			this.disconnect();
		}
		comPort.setBaudRate(BAUDRATE);
		conn = comPort.openPort();
		if(conn)
		{
			mdmState = ModemState.STOPPED;
			
			this.startNormal();
			this.setWifiPhy(wifiPhy);
			this.setWifiRate(wifiRate);
			this.setChannel(wifiChannel);
			this.setMaxTpw(wifiMaxTpw);
		}
		return conn;
	}

	@Override
	public void disconnect()
	{
		this.stop();
		
		// wait for txrx is finished
		synchronized(rxLock)	
		{
			synchronized(txLock)
			{
				mdmState = ModemState.DISCONNECTED;
				comPort.closePort();
			}
		}
	}

	@Override
	public boolean ping(int timeout)
	{
		synchronized(txLock)
		{
			if(mdmState == ModemState.DISCONNECTED)
			{
				return false;
			}
		
			version = 0;
		
			byte mdmFrame[] = {MSG_FIRST_BYTE,MSG_SECOND_BYTE,MSG_FUNC_MODEM_STATE,0,0};
		
			int crc = crc16(mdmFrame, mdmFrame.length - 2);
			mdmFrame[mdmFrame.length - 2] = (byte)(crc & 0xFF);
			mdmFrame[mdmFrame.length - 1] = (byte)((crc>>8) & 0xFF);
		
			comPort.writeBytes(mdmFrame, mdmFrame.length);
		
			// wait for the modem response
			this.receive(timeout);
		
			if(version >= 1)
			{
				return true;
			}
		}
		
		return false;
	}
	
	public boolean isConnected()
	{
		return (this.getVersion() > 0) ? true:false;
	}

	/**
	 * @note data.length have to be less than 1372
	 */
	@Override
	public boolean send(byte[] data)
	{
		synchronized(txLock)
		{
			if(mdmState == ModemState.DISCONNECTED)
				return false;
		
			int pos = 0;
		
			byte mdmFrame[] = new byte[data.length + 7];
		
			mdmFrame[pos++] = MSG_FIRST_BYTE;
			mdmFrame[pos++] = MSG_SECOND_BYTE;
			mdmFrame[pos++] = MSG_FUNC_DATA;
			mdmFrame[pos++] = (byte)(data.length & 0xFF);
			mdmFrame[pos++] = (byte)((data.length >> 8) & 0xFF);
		
			for(int i = 0; i < data.length; i++)
			{
				mdmFrame[pos++] = data[i];
			}
		
			int crc = crc16(mdmFrame, mdmFrame.length - 2);
			mdmFrame[mdmFrame.length - 2] = (byte)(crc & 0xFF);
			mdmFrame[mdmFrame.length - 1] = (byte)((crc>>8) & 0xFF);
			int written = comPort.writeBytes(mdmFrame, mdmFrame.length);
			if(written != mdmFrame.length)
				return false;
		}
		
		return true;
	}

	/*
	private int rxPos = 0;
	private int rxBufLen = 0;
	private byte rxBuf[] = new byte[2048];
	
	@Override
	public byte[] receive(int timeout)
	{
		byte[] result = null;
		
		synchronized(rxLock)
		{		
			Message msg = null;
			int numRead = 0;
			byte[] readBuffer = new byte[256];
			
			if(mdmState == ModemState.DISCONNECTED)
			{
				return null;
			}
		
			comPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, timeout, 0);
		
			try
			{
				do
				{
					numRead = comPort.readBytes(readBuffer, readBuffer.length);
					// notify listeners
					if(listener != null && numRead > 0)
					{	
						listener.onRawDataReceived(Arrays.copyOf(readBuffer,numRead));
					}
					// copy and parse read bytes to our buffer
					for(int i = 0; i < numRead; i++)
					{
						byte chr = readBuffer[i];
					
						rxBuf[rxPos++] = chr;
						
						if(rxPos >= rxBuf.length)
						{
							rxPos = 0;
							uartMsgDetected = false;
							System.out.println("WifiBroadcastModem: rx buffer is overflowed");
							continue;
						}
					
						msg = parse(chr);
					
						if(msg != null && listener != null)
						{
							listener.onMessageReceived(msg);
						}
					
						if(msg instanceof ModemDataMsg)
						{
							ModemDataMsg dataMsg = (ModemDataMsg)msg;
							result = dataMsg.getData();
							return result;	// this we can lose data but if data stream is too active we never exit from this func without return result
						}
						
						if(msg instanceof ModemLogMsg)
						{
							System.out.println("WifiBroadcastModem: " + msg.toString());
						}
					}
				}
				while(numRead > 0);
			}
			catch(Exception e)
			{
				e.printStackTrace();
			}
		}
		
		return result;
	}
	
	private int msgFirstPos = 0;
	private boolean uartMsgDetected = false;
	private byte msgPrevChr = 0;
	
	private Message parse(byte chr)
	{
		int func = 0;
		Message msg = null;
		
		byte receivedFrame[];
		int receivedFrameLen;
		int dataFrameLen;

		if( chr == MSG_SECOND_BYTE &&
			msgPrevChr == MSG_FIRST_BYTE &&
			uartMsgDetected == false)
		{
			msgFirstPos = rxPos - 2;
			uartMsgDetected = true;
			return null;
		}
		
		msgPrevChr = chr;
			
		if(uartMsgDetected)
		{
			func = rxBuf[msgFirstPos + 2];
			receivedFrameLen = rxPos - msgFirstPos;
			
			if(func < MSG_FIRST_FUNC || func > MSG_LAST_FUNC)
			{
				rxPos = 0;
				uartMsgDetected = false;
				System.out.println("WifiBroadcastModem.parse: received modem message has wrong FUNC");
				return null;
			}
		
			if(receivedFrameLen < 6) // 6 is minimum message length
				return null;
				
			if(func == MSG_FUNC_DATA)
			{
				dataFrameLen = ( ((int)(rxBuf[msgFirstPos + 4])&0xFF) << 8 ) | ( (int)(rxBuf[msgFirstPos + 3]) & 0xFF );
					
				if( (receivedFrameLen - 7) < dataFrameLen ) // 7 is (header + crc) length
				{
					return null;
				}
			}
			
			if(func == MSG_FUNC_MODEM_STATE && receivedFrameLen < 11)
			{
				return null;
			}
				
			receivedFrame = Arrays.copyOfRange(rxBuf, msgFirstPos, rxPos);
			int crc = crc16(receivedFrame, receivedFrameLen - 2); // 2 is CRC length
			int msgCrc = ( ((int)(rxBuf[rxPos-1])&0xFF) << 8 ) | ( (int)(rxBuf[rxPos-2])&0xFF) ;
				
			if(crc == msgCrc)
			{
				func = receivedFrame[2];
					
				switch(func)
				{
				case MSG_FUNC_MODEM_STATE:
					msg = new ModemStateMsg(	(int)receivedFrame[3],
												(int)receivedFrame[4],
												(int)receivedFrame[5],
												(int)receivedFrame[6],
												(int)receivedFrame[7],
												(int)receivedFrame[8] );
					break;
				case MSG_FUNC_DATA:
					msg = new ModemDataMsg(Arrays.copyOfRange(receivedFrame, 5, receivedFrame.length - 2));
					break;
				case MSG_FUNC_LOG:
					msg = new ModemLogMsg(Arrays.copyOfRange(receivedFrame, 5, receivedFrame.length - 2));
					break;
				default:
					System.out.println("WifiBroadcastModem.parse: received unknown modem message");
					break;
				}
			}
			else
			{
				System.out.println("WifiBroadcastModem.parse: received modem message has wrong CRC");
			}
			
			rxPos = 0;
			uartMsgDetected = false;
		}
		
		return msg;
	}
	*/
	
	private int rxFifoIn = 0;
	private int rxFifoOut = 0;
	private byte rxBuf[] = new byte[4096];
	
	public byte[] receive(int timeout)
	{
		Message msg = null;
		int numRead = 0;
		byte[] result = null;
		byte[] readBuffer = new byte[256];
		
		synchronized(rxLock)
		{		
			if(mdmState == ModemState.DISCONNECTED)
			{
				return null;
			}
		
			comPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, timeout, 0);
		
			try
			{
				do
				{					
					// receive new data if we have not parsed data
					if(rxFifoOut >= rxFifoIn)
					{
						numRead = comPort.readBytes(readBuffer, readBuffer.length);
					}
					// notify listeners
					if(listener != null && numRead > 0)
					{	
						listener.onRawDataReceived(Arrays.copyOf(readBuffer,numRead));
					}
					// copy new bytes to out buffer
					for(int i = 0; i < numRead; i++)
					{
						if(rxFifoIn >= rxBuf.length)
						{
							rxFifoIn = 0;
							rxFifoOut = 0;
							resetParser();
							System.out.println("WifiBroadcastModem: rx buffer of parser is overflowed");
						}
						rxBuf[rxFifoIn++] = readBuffer[i];
					}
					// parse
					while(rxFifoOut < rxFifoIn)
					{
						uartMsgPrevChr = uartMsgChr;
						uartMsgChr = rxBuf[rxFifoOut++];
						
						msg = parse();
					
						if(msg != null && listener != null)
						{
							listener.onMessageReceived(msg);
						}
					
						if(msg instanceof ModemDataMsg)
						{
							ModemDataMsg dataMsg = (ModemDataMsg)msg;
							result = dataMsg.getData();
							break;
						}
						
						if(msg instanceof ModemLogMsg)
						{
							System.out.println("WifiBroadcastModem: received log msg = " + msg.toString());
						}
					}
					// remove parsed bytes
					if(	rxFifoOut >= rxFifoIn &&
						!uartMsgDetected && 
						uartMsgChr != MSG_FIRST_BYTE )
					{
						rxFifoOut = 0;
						rxFifoIn = 0;
					}
					// return result
					if(result != null)
					{
						return result;
					}
				}
				while(numRead > 0);
			}
			catch(Exception e)
			{
				e.printStackTrace();
			}
		}
		
		return result;
	}
	
	private int uartMsgFirstPos;
	private boolean uartMsgDetected;
	private byte uartMsgChr;
	private byte uartMsgPrevChr;
	private int uartMsgDataFrameLen;
	private int uartMsgFunc = 0;
	
	private void resetParser()
	{
		uartMsgDetected = false;
		uartMsgDataFrameLen = 0;
		uartMsgPrevChr = 0;
		uartMsgChr = 0;
		uartMsgFunc = 0;
		uartMsgFirstPos = 0;
		// remove parsed bytes
		if(rxFifoOut > 0)
		{
			int len = rxFifoIn - rxFifoOut;
			System.arraycopy(rxBuf,rxFifoOut,rxBuf,0,len);
			rxFifoOut = 0;
			rxFifoIn = len;
		}
	}
	
	private Message parse()
	{
		Message msg = null;
		
		byte receivedFrame[];
		int receivedFrameLen;

		if( uartMsgChr == MSG_SECOND_BYTE &&
			uartMsgPrevChr == MSG_FIRST_BYTE &&
			uartMsgDetected == false)
		{
			uartMsgFirstPos = rxFifoOut - 2;
			uartMsgDetected = true;
			uartMsgDataFrameLen = 0;
		    uartMsgFunc = 0;
		    
			return null;
		}

		if(uartMsgDetected)
		{
			uartMsgFunc = rxBuf[uartMsgFirstPos + 2];
			receivedFrameLen = rxFifoOut - uartMsgFirstPos;
			
			if(uartMsgFunc < MSG_FIRST_FUNC || uartMsgFunc > MSG_LAST_FUNC)
			{
				resetParser();
				System.out.println("WifiBroadcastModem.parse: received modem message has wrong FUNC");
				return null;
			}
		
			if(receivedFrameLen < 6) // 6 is minimum message length
				return null;
				
			if(uartMsgFunc == MSG_FUNC_DATA || uartMsgFunc == MSG_FUNC_LOG)
			{
				if(uartMsgDataFrameLen == 0)
				{
					uartMsgDataFrameLen = ( ((int)(rxBuf[uartMsgFirstPos + 4])&0xFF) << 8 ) | ( (int)(rxBuf[uartMsgFirstPos + 3]) & 0xFF );
				}
					
				if( (receivedFrameLen - 7) < uartMsgDataFrameLen ) // 7 is (header + crc) length
				{
					return null;
				}
			}
			
			if(uartMsgFunc == MSG_FUNC_MODEM_STATE && receivedFrameLen < 11)
			{
				return null;
			}
				
			receivedFrame = Arrays.copyOfRange(rxBuf, uartMsgFirstPos, rxFifoOut);
			int crc = crc16(receivedFrame, receivedFrameLen - 2); // 2 is CRC length
			int msgCrc = ( ((int)(rxBuf[rxFifoOut-1])&0xFF) << 8 ) | ( (int)(rxBuf[rxFifoOut-2])&0xFF) ;
				
			if(crc == msgCrc)
			{	
				switch(uartMsgFunc)
				{
				case MSG_FUNC_MODEM_STATE:
					msg = new ModemStateMsg(	(int)receivedFrame[3],
												(int)receivedFrame[4],
												(int)receivedFrame[5],
												(int)receivedFrame[6],
												(int)receivedFrame[7],
												(int)receivedFrame[8] );
					break;
				case MSG_FUNC_DATA:
					msg = new ModemDataMsg(Arrays.copyOfRange(receivedFrame, 5, receivedFrame.length - 2));
					break;
				case MSG_FUNC_LOG:
					msg = new ModemLogMsg(Arrays.copyOfRange(receivedFrame, 5, receivedFrame.length - 2));
					break;
				default:
					System.out.println("WifiBroadcastModem.parse: received unknown modem message");
					break;
				}
			}
			else
			{
				System.out.println("WifiBroadcastModem.parse: received modem message has wrong CRC");
			}
			
			resetParser();
		}
		
		return msg;
	}
	
	private void waitCmdExecution()
	{
		try
		{
			Thread.sleep(10);
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}
	
	public void startSniffer()
	{
		synchronized(txLock)
		{
			if(mdmState == ModemState.DISCONNECTED)
				return;
		
			byte mdmFrame[] = {MSG_FIRST_BYTE,MSG_SECOND_BYTE,MSG_FUNC_START_SNIFFER,0,0};
		
			int crc = crc16(mdmFrame, mdmFrame.length - 2);
			mdmFrame[mdmFrame.length - 2] = (byte)(crc & 0xFF);
			mdmFrame[mdmFrame.length - 1] = (byte)((crc>>8) & 0xFF);
			comPort.writeBytes(mdmFrame, mdmFrame.length);
			mdmState = ModemState.SNIFFER;
			waitCmdExecution();
		}
	}
	
	public void startNormal()
	{
		synchronized(txLock)
		{
			if(mdmState == ModemState.DISCONNECTED)
				return;
		
			byte mdmFrame[] = {MSG_FIRST_BYTE,MSG_SECOND_BYTE,MSG_FUNC_START_NORMAL,0,0};
		
			int crc = crc16(mdmFrame, mdmFrame.length - 2);
			mdmFrame[mdmFrame.length - 2] = (byte)(crc & 0xFF);
			mdmFrame[mdmFrame.length - 1] = (byte)((crc>>8) & 0xFF);
			comPort.writeBytes(mdmFrame, mdmFrame.length);
			mdmState = ModemState.NORMAL;
			waitCmdExecution();
		}
	}
	
	public void stop()
	{
		synchronized(txLock)
		{
			if(mdmState == ModemState.DISCONNECTED)
				return;
		
			byte mdmFrame[] = {MSG_FIRST_BYTE,MSG_SECOND_BYTE,MSG_FUNC_STOP,0,0};
		
			int crc = crc16(mdmFrame, mdmFrame.length - 2);
			mdmFrame[mdmFrame.length - 2] = (byte)(crc & 0xFF);
			mdmFrame[mdmFrame.length - 1] = (byte)((crc>>8) & 0xFF);
		
			comPort.writeBytes(mdmFrame, mdmFrame.length);
			mdmState = ModemState.STOPPED;
			waitCmdExecution();
		}
	}
	
	public void setChannel(int channel)
	{
		synchronized(txLock)
		{
			if(mdmState == ModemState.DISCONNECTED)
				return;
		
			byte mdmFrame[] = {MSG_FIRST_BYTE,MSG_SECOND_BYTE,MSG_FUNC_SET_CHANNEL,(byte)channel,0,0};
		
			int crc = crc16(mdmFrame, mdmFrame.length - 2);
			mdmFrame[mdmFrame.length - 2] = (byte)(crc & 0xFF);
			mdmFrame[mdmFrame.length - 1] = (byte)((crc>>8) & 0xFF);
			comPort.writeBytes(mdmFrame, mdmFrame.length);
			waitCmdExecution();
		}
	}
	
	public void setMaxTpwDbm(float dbm)
	{
		this.setMaxTpw((int)(dbm / 0.25f));
	}
	
	public void setMaxTpw(int tpwLevel)
	{
		synchronized(txLock)
		{
			if(mdmState == ModemState.DISCONNECTED)
				return;
		
			byte mdmFrame[] = {MSG_FIRST_BYTE,MSG_SECOND_BYTE,MSG_FUNC_SET_TX_POWER,(byte)tpwLevel,0,0};
		
			int crc = crc16(mdmFrame, mdmFrame.length - 2);
			mdmFrame[mdmFrame.length - 2] = (byte)(crc & 0xFF);
			mdmFrame[mdmFrame.length - 1] = (byte)((crc>>8) & 0xFF);
		
			comPort.writeBytes(mdmFrame, mdmFrame.length);
			waitCmdExecution();
		}
	}
	
	public void setWifiRate(WifiRate rate)
	{
		synchronized(txLock)
		{
			if(mdmState == ModemState.DISCONNECTED)
				return;
		
			byte mdmFrame[] = {MSG_FIRST_BYTE,MSG_SECOND_BYTE,MSG_FUNC_SET_TX_RATE,(byte)rate.toInt(),0,0};
		
			int crc = crc16(mdmFrame, mdmFrame.length - 2);
			mdmFrame[mdmFrame.length - 2] = (byte)(crc & 0xFF);
			mdmFrame[mdmFrame.length - 1] = (byte)((crc>>8) & 0xFF);
			comPort.writeBytes(mdmFrame, mdmFrame.length);
			waitCmdExecution();
		}
	}
	
	public void setWifiPhy(WifiPhy phy)
	{
		synchronized(txLock)
		{
			if(mdmState == ModemState.DISCONNECTED)
				return;
		
			byte mdmFrame[] = {MSG_FIRST_BYTE,MSG_SECOND_BYTE,MSG_FUNC_SET_PHY_MODE,(byte)phy.toInt(),0,0};
		
			int crc = crc16(mdmFrame, mdmFrame.length - 2);
			mdmFrame[mdmFrame.length - 2] = (byte)(crc & 0xFF);
			mdmFrame[mdmFrame.length - 1] = (byte)((crc>>8) & 0xFF);
			comPort.writeBytes(mdmFrame, mdmFrame.length);
			waitCmdExecution();
		}
	}
	
	private static final int wCRCTable[] =
	{
		0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
		0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
		0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
		0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
		0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
		0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
		0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
		0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
		0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
		0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
		0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
		0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
		0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
		0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
		0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
		0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
		0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
		0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
		0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
		0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
		0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
		0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
		0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
		0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
		0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
		0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
		0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
		0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
		0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
		0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
		0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
		0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
	};
	
	public static int crc16(byte packet[], int wLength)
	{
	  int nTemp;
	  int pos = 0;
	  int wCRCWord = 0xFFFF;

	  while(wLength-- > 0)
	  {
	    nTemp = ((int)packet[pos++] ^ wCRCWord) & 0xFF;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[nTemp];
	  }
	  return wCRCWord;
	}
	
	public static String byteArrayToHex(byte[] a)
	{
		StringBuilder sb = new StringBuilder(a.length * 2);
		for(byte b: a)
			sb.append(String.format("%02x ", b));
		return sb.toString();
	}
	
	public static double dbmToWatts(double dbm)
	{
		return Math.pow(10.0, dbm/10.0)/1000.0;
	}
}
