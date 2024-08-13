package ru.liftelectronica.cicada;

import android.content.Context;
import android.media.MediaPlayer;
import android.provider.MediaStore;

import java.util.HashMap;
import java.util.Timer;
import java.util.TimerTask;

import pdl.DroneAlarmCenter;
import pdl.res.SoundProvider;

public class PDLSoundProvider extends SoundProvider {

    private HashMap<String, MediaPlayer> mSounds;

    Context context;

    private Timer tm = new Timer("Sounds");

    private class PlayLater extends TimerTask
    {
        private MediaPlayer snd;

        private PlayLater(String key)
        {
            snd = mSounds.get(key);

            if(snd == null)
            {
                System.out.println("Sound is not found " + key);
            }
        }

        @Override
        public void run() {
            if (snd != null) {
                try {
                    if (snd.isPlaying() == true) {
                        return;
                    }
                    snd.setLooping(false);
                    snd.start();
                } catch (IllegalStateException ex) {
                    System.out.println("Can't play sound");
                }
            }
        }
    }


    public PDLSoundProvider(Context ctx) {
        context = ctx;
        mSounds = new HashMap<String,MediaPlayer>();
    }

    @Override
    public boolean init() {
        mSounds.put(BAT15,MediaPlayer.create(context, R.raw.bat15));
        mSounds.put(BAT10,MediaPlayer.create(context, R.raw.bat10));
        mSounds.put(BAT5,MediaPlayer.create(context,R.raw.bat5));
        mSounds.put(BAT1,MediaPlayer.create(context,R.raw.bat1));
        mSounds.put(LOW_RADIO_SIGNAL,MediaPlayer.create(context,R.raw.low_radio_signal));
        mSounds.put(ALT_1M,MediaPlayer.create(context,R.raw.alt1m));
        mSounds.put(ALT_2M,MediaPlayer.create(context,R.raw.alt2m));
        mSounds.put(ALT_3M,MediaPlayer.create(context,R.raw.alt3m));
        mSounds.put(ALT_5M,MediaPlayer.create(context,R.raw.alt5m));
        mSounds.put(ALT_7M,MediaPlayer.create(context,R.raw.alt7m));
        mSounds.put(ALT_10M,MediaPlayer.create(context,R.raw.alt10m));
        mSounds.put(ALT_15M,MediaPlayer.create(context,R.raw.alt15m));
        mSounds.put(ALT_20M,MediaPlayer.create(context,R.raw.alt20m));
        mSounds.put(ALT_30M,MediaPlayer.create(context,R.raw.alt30m));
        mSounds.put(ALT_40M,MediaPlayer.create(context,R.raw.alt40m));
        mSounds.put(ALT_50M,MediaPlayer.create(context,R.raw.alt50m));
        mSounds.put(MOTORS_ENABLED,MediaPlayer.create(context,R.raw.motors_enabled));
        mSounds.put(MOTORS_DISABLED,MediaPlayer.create(context,R.raw.motors_disabled));
        mSounds.put(STABILIZATION_ENABLED,MediaPlayer.create(context,R.raw.stabilization_enabled));
        mSounds.put(STABILIZATION_DISABLED,MediaPlayer.create(context,R.raw.stabilization_disabled));
        mSounds.put(SYSTEM_BAD,MediaPlayer.create(context,R.raw.system_bad));
        mSounds.put(SYSTEM_OK,MediaPlayer.create(context,R.raw.system_ok));
        mSounds.put(TRICK_MODE_ENABLED,MediaPlayer.create(context,R.raw.trick_mode_enabled));
        mSounds.put(TRICK_MODE_DISABLED,MediaPlayer.create(context,R.raw.trick_mode_disabled));
        mSounds.put(VIDEO_STARTED,MediaPlayer.create(context,R.raw.video_started));
        mSounds.put(VIDEO_STOPED,MediaPlayer.create(context,R.raw.video_stoped));
        mSounds.put(PHOTO_TAKEN,MediaPlayer.create(context,R.raw.photo_taken));

        return true;
    }

    public void deinit() {
        for(MediaPlayer snd : mSounds.values()) {
            //snd.stop();
            snd.release();
        }
        mSounds.clear();
    }

    @Override
    public void play(String key) {
        MediaPlayer snd = mSounds.get(key);

        if(snd == null) {
            System.out.println("Sound is not found " + key);
            return;
        }

        try {
            if (snd.isPlaying() == true) {
                return;
            }
            snd.setLooping(false);
            snd.start();
        }
        catch(IllegalStateException ex) {
            System.out.println("Can't play sound");
        }
    }

    @Override
    public void playLater(String key, int delayMs) {
        tm.schedule(new PlayLater(key), delayMs);
    }
}
