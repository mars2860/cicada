package pdl.commands;

import pdl.DroneState;

public class CmdSetKalman extends CmdSetup
{
	DroneState.KalmanSettings ks;
	
	public CmdSetKalman(DroneState.KalmanSettings data)
	{
		super(136);
		ks = data;
	}
	
	public boolean settingsEquals(DroneState ds)
	{
		if( ds.kfSettings.navModelNoise == ks.navModelNoise			&&
		    ds.kfSettings.accVariance == ks.accVariance				&&
		    ds.kfSettings.lidarVelVariance1 == ks.lidarVelVariance1	&&
		    ds.kfSettings.baroAltVariance == ks.baroAltVariance		&&
		    ds.kfSettings.ofVelVariance1 == ks.ofVelVariance1		&&
		    ds.kfSettings.ofVelVariance2 == ks.ofVelVariance2		&&
		    ds.kfSettings.lidarVelVariance2 == ks.lidarVelVariance2 &&
		    ds.kfSettings.poseModelNoise == ks.poseModelNoise		&&
		    ds.kfSettings.yawRateVariance == ks.yawRateVariance		&&
		    ds.kfSettings.pitchRollRateVariance == ks.pitchRollRateVariance &&
		    ds.kfSettings.magHeadingVariance == ks.magHeadingVariance &&
		    ds.kfSettings.accPitchRollVariance == ks.accPitchRollVariance &&
		    ds.kfSettings.gpsHorSpeedVariance == ks.gpsHorSpeedVariance &&
		    ds.kfSettings.gpsVerSpeedVariance == ks.gpsVerSpeedVariance &&
		    ds.kfSettings.gpsHorPosVariance == ks.gpsHorPosVariance &&
		    ds.kfSettings.gpsVerPosVariance == ks.gpsVerPosVariance)
		{
			return true;
		}
		
		return false;
	}

	@Override
	public byte[] getPacketData()
	{
		int pos = 0;
		byte data[] = new byte[65];
		
		pos = writeUint8(pos,data,this.getCode());
		pos = writeFloat(pos,data,ks.navModelNoise);
		pos = writeFloat(pos,data,ks.accVariance);
		pos = writeFloat(pos,data,ks.baroAltVariance);
		pos = writeFloat(pos,data,ks.lidarVelVariance1);
		pos = writeFloat(pos,data,ks.ofVelVariance1);
		pos = writeFloat(pos,data,ks.ofVelVariance2);
		pos = writeFloat(pos,data,ks.lidarVelVariance2);
		pos = writeFloat(pos,data,ks.poseModelNoise);
		pos = writeFloat(pos,data,ks.yawRateVariance);
		pos = writeFloat(pos,data,ks.pitchRollRateVariance);
		pos = writeFloat(pos,data,ks.magHeadingVariance);
		pos = writeFloat(pos,data,ks.accPitchRollVariance);
		pos = writeFloat(pos,data,ks.gpsHorSpeedVariance);
		pos = writeFloat(pos,data,ks.gpsVerSpeedVariance);
		pos = writeFloat(pos,data,ks.gpsHorPosVariance);
		pos = writeFloat(pos,data,ks.gpsVerPosVariance);

		return data;
	}
}