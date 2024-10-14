package pdl.wlan;

public abstract class Modem
{
	public abstract boolean connect();
	public abstract void disconnect();
	public abstract boolean ping(int timeout);
	public abstract boolean send(byte data[]);
	public abstract byte[] receive(int timeout);
}
