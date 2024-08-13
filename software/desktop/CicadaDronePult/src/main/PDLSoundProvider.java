package main;

import java.util.Timer;
import java.util.TimerTask;

import pdl.res.SoundProvider;

public class PDLSoundProvider extends SoundProvider
{
	private Timer tm = new Timer("Sounds");
	
	private class PlayLater extends TimerTask
	{
		private Sound snd;
		
		private PlayLater(String key)
		{
			snd = ResBox.sound(key);
			
			if(snd == null)
			{
				System.out.println("Sound is not found " + key);
			}
		}

		@Override
		public void run()
		{
			if(snd != null)
			{
				snd.play();
			}
		}
	}
	
	@Override
	public boolean init()
	{
		return true;
	}

	@Override
	public void play(String key)
	{
		Sound snd = ResBox.sound(key);
		
		if(snd != null)
		{
			snd.play();
		}
		else
		{
			System.out.println("Sound is not found " + key);
		}
	}

	@Override
	public void playLater(String key, int delayMs)
	{
		tm.schedule(new PlayLater(key), delayMs);
	}
}
