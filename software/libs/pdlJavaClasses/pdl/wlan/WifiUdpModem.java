package pdl.wlan;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.util.Arrays;

public class WifiUdpModem extends Modem
{
	public static final int TIMEOUT = 3000;
	
	private int mPort;
	private String mIp;
	private InetAddress mAddr;
	private DatagramSocket mSocket;
	
	private Object rxLock = new Object();
	private Object txLock = new Object();
	
	public WifiUdpModem(String ip, int port)
	{
		mPort = port;
		mIp = ip;
	}
	
	@Override
	public boolean connect()
	{
		boolean result = false;
		
		if(mSocket != null)
		{
			this.disconnect();
		}
		
		try
		{
			mAddr = InetAddress.getByName(mIp);
			mSocket = new DatagramSocket(mPort);
			mSocket.setSoTimeout(TIMEOUT);
			result = true;
		}
		catch(UnknownHostException | SocketException e)
		{
			e.printStackTrace();
			result = false;
		}
		
		return result;
	}

	@Override
	public void disconnect()
	{	
		synchronized(rxLock)
		{
			synchronized(txLock)
			{
				if(mSocket != null)
				{
					mSocket.close();
					mSocket = null;
				}
			}
		}
	}

	@Override
	public boolean ping(int timeout)
	{
		synchronized(txLock)
		{
			if(mSocket != null)
			{
				return true;
			}
		}
		
		return false;
	}

	@Override
	public boolean send(byte[] data)
	{
		boolean result = false;
		
		synchronized(txLock)
		{
			if(mSocket == null)
				return false;
		
			DatagramPacket packet = new DatagramPacket(data, data.length, mAddr, mPort);
			try
			{
				mSocket.send(packet);
				result = true;
			}
			catch(IOException e)
			{
				result = false;
				e.printStackTrace();
			}
		}
		
		return result;
	}

	@Override
	public byte[] receive(int timeout)
	{
		byte[] result = null;
		
		synchronized(rxLock)
		{
			if(mSocket == null)
				return null;
			
			byte[] receivedData = new byte[2048];
		
			DatagramPacket receivedPacket = new DatagramPacket(receivedData, receivedData.length);
		
			try
			{
				mSocket.setSoTimeout(timeout);
				mSocket.receive(receivedPacket);
			
				if(receivedPacket.getAddress().equals(mAddr))
				{	
					result = Arrays.copyOf(receivedPacket.getData(),receivedPacket.getLength());
				}
			
			}
			catch(SocketTimeoutException e)
			{
				// timeout
			}
			catch(IOException e)
			{
				e.printStackTrace();
			}
		}
		
		return result;
	}
}
