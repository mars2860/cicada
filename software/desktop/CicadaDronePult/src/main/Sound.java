package main;

import java.io.IOException;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.LineEvent;
import javax.sound.sampled.LineListener;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.UnsupportedAudioFileException;

public class Sound implements LineListener
{
   protected boolean playCompleted = true;
   protected Clip audioClip;
   
   public Sound(java.net.URL url)
   {
       try
       {
    	   AudioInputStream audioStream = AudioSystem.getAudioInputStream(url);
           AudioFormat format = audioStream.getFormat();
           DataLine.Info info = new DataLine.Info(Clip.class, format);
           audioClip = (Clip)AudioSystem.getLine(info);
           audioClip.addLineListener(this);
           audioClip.open(audioStream);
       }
       catch (UnsupportedAudioFileException ex)
       {
           System.out.println("The specified audio file is not supported.");
           ex.printStackTrace();
       }
       catch(LineUnavailableException ex)
       {
           System.out.println("Audio line for playing back is unavailable.");
           ex.printStackTrace();
       }
       catch(IOException ex)
       {
           ex.printStackTrace();
       }
       catch(Exception e)
       {
		e.printStackTrace();
       }   
   }
   
   public void play()
   {
	   if(playCompleted == false)
		   return;
	   
	   try
	   {
		   audioClip.setFramePosition(0);
		   audioClip.start();
	   }
	   catch(Exception ex)
	   {
		   System.out.println("Error playing the audio file.");
		   ex.printStackTrace();
	   }
   }

   @Override
   public void update(LineEvent event)
   {
	   LineEvent.Type type = event.getType();
        
       if (type == LineEvent.Type.START)
       {
    	   playCompleted = false;
            
       }
       else if(type == LineEvent.Type.STOP)
       {
           playCompleted = true;
       }
   }
}