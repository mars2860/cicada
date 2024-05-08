package pdl.commands;

import pdl.DroneState;

public class CmdSetUserData1 extends CmdSetup
{
	//DroneState.UserData1 ud;
	
	public CmdSetUserData1()//DroneState.UserData1 data)
	{
		super(131);
		//ud = data;
	}
	
	public boolean settingsEquals(DroneState ds)
	{
		/*if( ds.fUserData.userData0 == ud.userData0 &&
		    ds.fUserData.userData1 == ud.userData1 &&
		    ds.fUserData.userData2 == ud.userData2 &&
		    ds.fUserData.userData3 == ud.userData3 &&
		    ds.fUserData.userData4 == ud.userData4 &&
		    ds.fUserData.userData5 == ud.userData5 &&
		    
		    ds.fUserData.userData6 == ud.userData6 &&
		    ds.fUserData.userData7 == ud.userData7 &&
		    ds.fUserData.userData8 == ud.userData8 &&
		    ds.fUserData.userData9 == ud.userData9 &&
		    ds.fUserData.userData10 == ud.userData10 &&
		    ds.fUserData.userData11 == ud.userData11 )
		{
			return true;
		}
		
		return false;*/
		return true;
	}

	@Override
	public byte[] getPacketData()
	{
		//int pos = 0;
		byte data[] = new byte[49];
		
		/*pos = writeUint8(pos,data,this.getCode());
		pos = writeFloat(pos,data,ud.userData0);
		pos = writeFloat(pos,data,ud.userData1);
		pos = writeFloat(pos,data,ud.userData2);
		pos = writeFloat(pos,data,ud.userData3);
		pos = writeFloat(pos,data,ud.userData4);
		pos = writeFloat(pos,data,ud.userData5);
		pos = writeFloat(pos,data,ud.userData6);
		pos = writeFloat(pos,data,ud.userData7);
		pos = writeFloat(pos,data,ud.userData8);
		pos = writeFloat(pos,data,ud.userData9);
		pos = writeFloat(pos,data,ud.userData10);
		pos = writeFloat(pos,data,ud.userData11);*/
		
		return data;
	}
}