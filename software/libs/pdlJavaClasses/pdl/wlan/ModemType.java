package pdl.wlan;

public enum ModemType
{
	WIFI_UDP_MODEM(0),
	WIFI_BROADCAST_MODEM(1);
	
	private int type;
	
	private ModemType(int t)
	{
		type = t;
	}
	
	public int toInt()
	{
		return type;
	}
	
	public static ModemType fromInt(int t)
	{
		ModemType result = null;
		for(ModemType mt : ModemType.values())
		{
			if(mt.toInt() == t)
			{
				result = mt;
				break;
			}
		}
		return result;
	}
}
