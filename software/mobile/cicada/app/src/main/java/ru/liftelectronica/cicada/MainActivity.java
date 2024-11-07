package ru.liftelectronica.cicada;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.util.Log;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.appcompat.app.AppCompatActivity;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.text.DecimalFormat;
import java.util.Locale;
import java.util.Observable;
import java.util.Observer;
import java.util.Timer;
import java.util.TimerTask;

import com.fazecast.jSerialComm.SerialPort;
import com.fazecast.jSerialComm.android.AndroidPort;

import io.github.controlwear.virtual.joystick.android.JoystickView;
import pdl.Accelerator;
import pdl.commands.CmdEnableStabilization;
import pdl.commands.CmdSetLoad;
import pdl.commands.CmdSwitchMotors;
import pdl.res.Profile;
import pdl.res.TextBox;
import pdl.Alarm;
import pdl.DroneAlarmCenter;
import pdl.DroneCommander;
import pdl.DroneState;
import pdl.DroneTelemetry;
import pdl.commands.CmdResetAltitude;

public class MainActivity extends AppCompatActivity {

    public static final int JOYSTICK_UPDATE_TIME = 40;

    Timer mtmDroneMoveUpdater;

    TextView mtvAlarm;
    TextView mtvStatus1;
    TextView mtvStatus2;

    ToggleButton mbtnDisarm;

    PDLSoundProvider pdlSoundProvider;
    OnAlarmUpdate alarmObserver;
    OnTelemetryUpdate telemetryObserver;

    DPad dpad = new DPad();

    private boolean keyUpPressed;
    private boolean keyDownPressed;
    private boolean keyTurnCwPressed;
    private boolean keyTurnCcwPressed;
    private boolean keyLeftPressed;
    private boolean keyRightPressed;
    private boolean keyForwardPressed;
    private boolean keyBackwardPressed;
    private boolean leftStickIsMoved;
    private boolean rightStickIsMoved;

    private double liftCtrl = 0;
    private double rollCtrl = 0;
    private double pitchCtrl = 0;
    private double rotateCtrl = 0;

    private static final double STICK_FLAT = 0.09;

    private class OnAlarmUpdate implements Observer {
        @Override
        public void update(Observable o, Object arg) {
            MainActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Alarm alarm = DroneAlarmCenter.instance().getAlarm();
                    printAlarm(alarm);
                    DroneState ds = DroneTelemetry.instance().getDroneState();
                    DroneTelemetry.instance().speakDroneState(ds,pdlSoundProvider);
                }
            });
        }
    }

    private class OnTelemetryUpdate implements Observer {
        long timestamp;
        @Override
        public void update(Observable observable, Object data) {

            MainActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    DroneState ds = DroneTelemetry.instance().getDroneState();
                    // update wifi rssi for AP mode (we can't do that at the firmware side)
                    if( DroneCommander.instance().isWifiBroadcastActive() == false &&
                        DroneState.net.wifiStaMode == false ) {
                        WifiManager wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
                        WifiInfo info = wifiManager.getConnectionInfo();
                        ds.rssi = info.getRssi();
                        // check min rssi level
                        DroneTelemetry.instance().checkDroneStateForAlarms(ds);
                    }
                    printDroneState(ds);
                    DroneTelemetry.instance().speakDroneState(ds,pdlSoundProvider);
                    mbtnDisarm.setChecked(DroneTelemetry.instance().getDroneState().motorsEnabled);
                }
            });
        }
    }

    private class OnLeftJoystick implements JoystickView.OnMoveListener {

        @Override
        public void onMove(int angle, int strength) {
            double s = strength;
            double a = Math.toRadians(angle);

            s /= 100.0;

            rotateCtrl = s*Math.cos(a);
            liftCtrl = s*Math.sin(a);

            if( Math.abs(rotateCtrl) > STICK_FLAT ||
                Math.abs(liftCtrl) > STICK_FLAT ) {
                leftStickIsMoved = true;
            }
            else {
                leftStickIsMoved = false;
            }
        }
    }

    private class OnRightJoystick implements JoystickView.OnMoveListener {

        @Override
        public void onMove(int angle, int strength) {
            double s = strength;
            double a = Math.toRadians(angle);

            s /= 100.0;

            rollCtrl = s*Math.cos(a);
            pitchCtrl = s*Math.sin(a);

            if( Math.abs(rollCtrl) > STICK_FLAT ||
                Math.abs(pitchCtrl) > STICK_FLAT ) {
                rightStickIsMoved = true;
            }
            else {
                rightStickIsMoved = false;
            }
        }
    }

    private class OnUpdateDroneMove extends TimerTask {
        private Accelerator mLiftAccelerator;
        private Accelerator mPitchAccelerator;
        private Accelerator mRollAccelerator;
        private Accelerator mRotateAccelerator;

        private long oldMillis = 0;

        @Override
        public void run() {
            double dt = 0;

            DroneState.RemoteCtrl rc = DroneState.rc;
            DroneState ds = DroneTelemetry.instance().getDroneState();

            if(oldMillis > 0)
            {
                dt = (System.currentTimeMillis() - oldMillis);
                dt /= 1000.0;
            }
            else
            {
                dt = 50;
                dt /= 1000.0;
            }

            oldMillis = System.currentTimeMillis();

            /// grab accelerator settings

            if(	mLiftAccelerator == null ||
                    Math.abs(mLiftAccelerator.getAccelTime() - rc.liftAccelTime) > 0.01) {
                mLiftAccelerator = new Accelerator(-1.0,0,1.0,rc.liftAccelTime);
            }

            if(	mPitchAccelerator == null ||
                    Math.abs(mPitchAccelerator.getAccelTime() - rc.moveAccelTime) > 0.01) {
                mPitchAccelerator = new Accelerator(-1.0,0,1.0,rc.moveAccelTime);
            }

            if(	mRollAccelerator == null ||
                    Math.abs(mRollAccelerator.getAccelTime() - rc.moveAccelTime) > 0.01) {
                mRollAccelerator = new Accelerator(-1.0,0,1.0,rc.moveAccelTime);
            }

            if(	mRotateAccelerator == null ||
                    Math.abs(mRotateAccelerator.getAccelTime() - rc.rotateAccelTime) > 0.01) {
                mRotateAccelerator = new Accelerator(-1.0,0,1.0,rc.rotateAccelTime);
            }

            // get up
            if(keyUpPressed) {
                liftCtrl = mLiftAccelerator.accelerate(dt);
            }
            else if(keyDownPressed) {
                liftCtrl = mLiftAccelerator.decelerate(dt);
            }
            else if(leftStickIsMoved == false) {
                liftCtrl = mLiftAccelerator.normalize(dt);
            }

            // roll left
            if(keyLeftPressed) {
                rollCtrl = mRollAccelerator.decelerate(dt);
            }
            else if(keyRightPressed) {
                rollCtrl = mRollAccelerator.accelerate(dt);
            }
            else if(rightStickIsMoved == false) {
                rollCtrl = mRollAccelerator.normalize(dt);
            }

            // pitch fwd
            if(keyForwardPressed) {
                pitchCtrl = mPitchAccelerator.accelerate(dt);
            }
            else if(keyBackwardPressed) {
                pitchCtrl = mPitchAccelerator.decelerate(dt);
            }
            else if(rightStickIsMoved == false) {
                pitchCtrl = mPitchAccelerator.normalize(dt);
            }

            // rotate cw
            if(keyTurnCwPressed) {
                rotateCtrl = mRotateAccelerator.accelerate(dt);
            }
            else if(keyTurnCcwPressed) {
                rotateCtrl = mRotateAccelerator.decelerate(dt);
            }
            else if(leftStickIsMoved == false) {
                rotateCtrl = mRotateAccelerator.normalize(dt);
            }

            if(DroneCommander.instance().isSendCmdsAllowed() == false) {
                return;
            }

            DroneCommander.instance().liftDrone(liftCtrl);
            DroneCommander.instance().rotateDrone(rotateCtrl);
            DroneCommander.instance().moveDrone(pitchCtrl, rollCtrl);
        }
    }

    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent ev) {

        // Check that the event came from a game controller
        if ((ev.getSource() & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK &&
                ev.getAction() == MotionEvent.ACTION_MOVE) {

            // Check if this event if from a D-pad and process accordingly.
            if (DPad.isDpadDevice(ev)) {

                int press = dpad.getDirectionPressed(ev);
                switch (press) {
                    case DPad.LEFT:
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_LEFT,true);
                        return true;
                    case DPad.RIGHT:
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_RIGHT,true);
                        return true;
                    case DPad.UP:
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_UP,true);
                        return true;
                    case DPad.DOWN:
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_DOWN,true);
                        return true;
                    case DPad.CENTER:
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_CENTER,true);
                        return true;
                    case DPad.ANY_KEY_RELEASED:
                        System.out.println("DPAD released");
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_LEFT, false);
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_RIGHT,false);
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_UP,false);
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_DOWN,false);
                        processGamepadKey(KeyEvent.KEYCODE_DPAD_CENTER,false);
                        return true;
                }
            }

            rotateCtrl = ev.getAxisValue(MotionEvent.AXIS_X);
            liftCtrl = -ev.getAxisValue(MotionEvent.AXIS_Y);

            rollCtrl = ev.getAxisValue(MotionEvent.AXIS_Z);
            pitchCtrl = -ev.getAxisValue(MotionEvent.AXIS_RZ);

            if( Math.abs(rotateCtrl) > STICK_FLAT ||
                Math.abs(liftCtrl) > STICK_FLAT ) {
                leftStickIsMoved = true;
            }
            else {
                leftStickIsMoved = false;
            }

            if( Math.abs(rollCtrl) > STICK_FLAT ||
                Math.abs(pitchCtrl) > STICK_FLAT ) {
                rightStickIsMoved = true;
            }
            else {
                rightStickIsMoved = false;
            }

            return true;
        }

        return super.dispatchGenericMotionEvent(ev);
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {

        if ((event.getSource() & InputDevice.SOURCE_GAMEPAD) == InputDevice.SOURCE_GAMEPAD) {
            if (event.getRepeatCount() == 0) {

                processGamepadKey(  event.getKeyCode(),
                                    (event.getAction()==KeyEvent.ACTION_DOWN)?true:false);
                return true;
            }
        }

        return super.dispatchKeyEvent(event);
    }

    private void processGamepadKey(int keyCode, boolean pressed) {
        if(keyCode == keyTurnCw) {
            keyTurnCwPressed = pressed;
        }
        else if(keyCode == keyTurnCcw) {
            keyTurnCcwPressed = pressed;
        }

        if(keyCode == keyUp) {
            keyUpPressed = pressed;
        }

        if(keyCode == keyDown) {
            keyDownPressed = pressed;
        }

        if(keyCode == keyLeft) {
            keyLeftPressed = pressed;
        }

        if(keyCode == keyRight) {
            keyRightPressed = pressed;
        }

        if(keyCode == keyForward) {
            keyForwardPressed = pressed;
        }

        if(keyCode == keyBackward) {
            keyBackwardPressed = pressed;
        }

        if(DroneCommander.instance().isSendCmdsAllowed() == false) {
            return;
        }

        DroneState ds = DroneTelemetry.instance().getDroneState();

        if(keyCode == keyDisarm && pressed) {
            CmdSwitchMotors cmd = new CmdSwitchMotors(!ds.motorsEnabled);
            DroneCommander.instance().addCmd(cmd);

            CmdEnableStabilization cmd2 = new CmdEnableStabilization(false);
            DroneCommander.instance().addCmd(cmd2);
        }

        if(keyCode == keyTrickGyro && pressed) {
            if(ds.trickMode == DroneState.TrickMode.DISABLED)
                DroneCommander.instance().setTrickMode(DroneState.TrickMode.GYRO);
            else
                DroneCommander.instance().setTrickMode(DroneState.TrickMode.DISABLED);
        }

        if(keyCode == keyTrickAcro && pressed) {
            if(ds.trickMode == DroneState.TrickMode.DISABLED)
                DroneCommander.instance().setTrickMode(DroneState.TrickMode.ACRO);
            else
                DroneCommander.instance().setTrickMode(DroneState.TrickMode.DISABLED);
        }

        if(keyCode == keyTurtle) {
            DroneCommander.instance().antiTurtle(pressed,DroneState.Motors.antiTurtleGas);
        }

        if(keyCode == keyLoad1) {
            if(pressed && ds.load1.enabled == false)
            {
                CmdSetLoad cmd = new CmdSetLoad(0,true,ds.load1.period);
                DroneCommander.instance().addCmd(cmd);
            }
            if(pressed == false && ds.load1.enabled)
            {
                CmdSetLoad cmd = new CmdSetLoad(0,false,ds.load1.period);
                DroneCommander.instance().addCmd(cmd);
            }
        }

        if(keyCode == keyLoad2) {
            if(pressed && ds.load2.enabled == false)
            {
                CmdSetLoad cmd = new CmdSetLoad(1,true,ds.load2.period);
                DroneCommander.instance().addCmd(cmd);
            }
            if(pressed == false && ds.load2.enabled)
            {
                CmdSetLoad cmd = new CmdSetLoad(1,false,ds.load2.period);
                DroneCommander.instance().addCmd(cmd);
            }
        }

        if(keyCode == keyCam0 && pressed) {
            DroneCommander.instance().setCameraAngle(0);
        }

        if(keyCode == keyCam45 && pressed) {
            DroneCommander.instance().setCameraAngle(45);
        }

        if(keyCode == keyCam90 && pressed) {
            DroneCommander.instance().setCameraAngle(90);
        }

        if(keyCode == keyPhoto && pressed) {
            DroneCommander.instance().takePhoto();
            pdlSoundProvider.play("PHOTO_TAKEN");
        }

        if(keyCode == keyVideo && pressed) {
            if(ds.videoState) {
                DroneCommander.instance().stopVideo();
            } else {
                DroneCommander.instance().startVideo();
            }
        }
    }

    protected void myResDestroy() {
        mtmDroneMoveUpdater.cancel();

        if(pdlSoundProvider != null) {
            pdlSoundProvider.deinit();
            pdlSoundProvider = null;
        }

        DroneAlarmCenter.instance().deleteObserver(alarmObserver);
        DroneTelemetry.instance().deleteObserver(telemetryObserver);
    }

    private void myResCreate() {

        Context context = this.getApplicationContext();

        if(pdlSoundProvider == null)
        {
            pdlSoundProvider = new PDLSoundProvider(context);
            pdlSoundProvider.init();
        }

        // get profile name
        String profile = SettingsActivity.loadCurProfileName(this.getApplicationContext());
        // copy default profile from resources to disk if there is no any profile
        if(profile.isEmpty()) {
            profile = "default";
            SettingsActivity.saveCurProfileName(this.getApplicationContext(),profile);
            // copy default settings from resources to disk
            File file = new File(context.getFilesDir(),profile+".json");
            InputStream is = context.getResources().openRawResource(R.raw.default_profile);
            FileOutputStream fos = null;
            int n;
            try {
                byte buffer[] = new byte[1024];
                fos = new FileOutputStream(file);
                while( (n = is.read(buffer)) > 0) {
                    fos.write(buffer,0,n);
                }
                profile = "default";
            } catch (Exception e) {
                Log.e("", "Can't write default.json");
            } finally {
                try {
                    if(fos != null)
                        fos.close();
                } catch (Exception e) {
                    Log.e("", "Can't close default.json");
                }
                try {
                    if(is != null)
                        is.close();
                } catch (Exception e) {
                    Log.e("","Can't close raw/default_profile.json");
                }
            }
        }
        // load a profile
        if(profile.isEmpty() == false) {
            File file = new File(context.getFilesDir(),profile + ".json");
            if(Profile.load(file) == false) {
                Log.e("","Can't load settings from " + file.getPath());
            } else {
                Toast.makeText(this,profile,Toast.LENGTH_LONG).show();
            }
        }

        // load localization
        Locale.setDefault(Locale.ENGLISH);
        TextBox.load(new Locale(Locale.ENGLISH.getLanguage()));

        alarmObserver = new OnAlarmUpdate();
        telemetryObserver = new OnTelemetryUpdate();
        DroneAlarmCenter.instance().addObserver(alarmObserver);
        DroneTelemetry.instance().addObserver(telemetryObserver);

        if(DroneTelemetry.instance().isDroneConnected() == false) { // for first run
            DroneAlarmCenter.instance().setAlarm(Alarm.ALARM_DRONE_NOT_FOUND);
        }
        printAlarm(DroneAlarmCenter.instance().getAlarm());
        printDroneState(DroneTelemetry.instance().getDroneState());
        updateJoysticks();
        loadGamepadSettings(this);
        // to implement acceleration/deceleration by key_down we need to poll key state
        mtmDroneMoveUpdater = new Timer("DroneMoveUpdater");
        mtmDroneMoveUpdater.schedule(new OnUpdateDroneMove(),50, JOYSTICK_UPDATE_TIME);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        SerialPort.setAndroidContext(this);

        // app version
        try {
            String versionName = getApplicationContext().
                                    getPackageManager().
                                        getPackageInfo(getApplicationContext().getPackageName(),0).versionName;
            TextView tvAppVer = (TextView)findViewById(R.id.tvAppVersion);
            tvAppVer.setText(versionName);
        } catch (Exception e) {
            e.printStackTrace();
        }
        // find widgets
        mtvAlarm = (TextView)findViewById(R.id.tvAlarm);
        mbtnDisarm = (ToggleButton)findViewById(R.id.btnDisarm);

        mtvStatus1 = (TextView)findViewById(R.id.tvStatus1);
        mtvStatus2 = (TextView)findViewById(R.id.tvStatus2);
    }

    @Override
    protected void onStart() {
        super.onStart();
        myResCreate();
    }

    @Override
    protected void onStop() {
        super.onStop();
        myResDestroy();
    }

    @Override
    protected void onDestroy() {
        DroneCommander.instance().disconnect();
        SerialPort.setAndroidContext(null);
        super.onDestroy();
    }

    public void updateJoysticks() {
        DroneState ds = Profile.instance().getDroneSettings();
        JoystickView leftJoystick = (JoystickView)findViewById(R.id.joystickView_left);
        JoystickView rightJoystick = (JoystickView)findViewById(R.id.joystickView_right);

        //leftJoystick.setFixedCenter(DroneState.rc.leftJoystickAutoCenter);
        //rightJoystick.setFixedCenter(DroneState.rc.rightJoystickAutoCenter);
        leftJoystick.setAutoReCenterButton(DroneState.rc.leftJoystickAutoCenter);
        rightJoystick.setAutoReCenterButton(DroneState.rc.rightJoystickAutoCenter);

        leftJoystick.setOnMoveListener(new OnLeftJoystick(), JOYSTICK_UPDATE_TIME);
        rightJoystick.setOnMoveListener(new OnRightJoystick(), JOYSTICK_UPDATE_TIME);

        if(DroneState.rc.virtualGamepad) {
            leftJoystick.setEnabled(true);
            leftJoystick.setVisibility(View.VISIBLE);
            rightJoystick.setEnabled(true);
            rightJoystick.setVisibility(View.VISIBLE);
        }
        else {
            leftJoystick.setEnabled(false);
            leftJoystick.setVisibility(View.INVISIBLE);
            rightJoystick.setEnabled(false);
            rightJoystick.setVisibility(View.INVISIBLE);
        }
    }

    @Override
    protected void onResume() {
        super.onResume();

        updateJoysticks();
    }

    public void onBtnConnect(View v) {
        DroneCommander.instance().connect();
    }

    public void onBtnDisarm(View v) {
        CmdSwitchMotors cmd = new CmdSwitchMotors(mbtnDisarm.isChecked());
        DroneCommander.instance().addCmd(cmd);

        CmdEnableStabilization cmd2 = new CmdEnableStabilization(false);
        DroneCommander.instance().addCmd(cmd2);

        v.postDelayed(new Runnable() {
            @Override
            public void run() {
                mbtnDisarm.setChecked(DroneTelemetry.instance().getDroneState().motorsEnabled);
            }
        }, 250);
    }

    public void onBtnSettings(View v) {
        Intent intent = new Intent(v.getContext(), SettingsActivity.class);
        v.getContext().startActivity(intent);
    }

    public void onBtnGamepad(View v) {
        Intent intent = new Intent(v.getContext(), GamepadActivity.class);
        v.getContext().startActivity(intent);
    }

    private int appendState(String text, int rowCount) {
        if(rowCount < DroneState.widgets.maxRowCount) {
            mtvStatus1.append(text);
        }
        else if(rowCount < DroneState.widgets.maxRowCount*2) {
            mtvStatus2.append(text);
        }
        rowCount++;
        return rowCount;
    }

    private void printDroneState(DroneState ds) {
        int rowCount = 0;
        DecimalFormat fmt1 = new DecimalFormat();
        fmt1.setMaximumFractionDigits(2);
        fmt1.setMinimumFractionDigits(0);
        fmt1.setGroupingUsed(false);

        DecimalFormat fmt2 = new DecimalFormat();
        fmt2.setMaximumFractionDigits(0);
        fmt2.setGroupingUsed(false);

        DecimalFormat fmt3 = new DecimalFormat();
        fmt3.setMaximumFractionDigits(3);
        fmt3.setGroupingUsed(false);

        DecimalFormat fmt5 = new DecimalFormat();
        fmt5.setMinimumIntegerDigits(2);

        DecimalFormat fmt6 = new DecimalFormat();
        fmt6.setMinimumIntegerDigits(2);

        mtvStatus1.setText("");
        mtvStatus2.setText("");

        if(DroneState.widgets.flyTime) {
            int secs = (int)(DroneTelemetry.instance().getFlyTime() / 1000000.0);
            int mins = secs / 60;
            secs = secs - mins*60;
            String flyTime = "flyTime: " + fmt5.format(mins) + ":" + fmt6.format(secs) + "\n";
            rowCount = appendState(flyTime,rowCount);
        }

        if(DroneState.widgets.battery) {
            String bat = "battery: " + fmt1.format(ds.battery.voltage) + "V/" + fmt2.format(ds.battery.percent) + "%" + "\n";
            rowCount = appendState(bat,rowCount);
        }

        if(DroneState.widgets.rssi) {
            String rssi = "rssi: " + fmt2.format(ds.rssi) + "\n";
            rowCount = appendState(rssi,rowCount);
        }

        if(DroneState.widgets.latency) {
            String latency = "delay: " + fmt2.format(DroneCommander.instance().getWlanLatency()) + "\n";
            rowCount = appendState(latency,rowCount);
        }

        if(DroneState.widgets.lostRxPackets) {
            String lostRxPackets = "lostRx: " + fmt2.format(DroneCommander.instance().getLostRxPacketCounter()) + "\n";
            rowCount = appendState(lostRxPackets,rowCount);
        }

        if(DroneState.widgets.home) {
            String home = "home: " + fmt1.format(ds.distToHome) + "m/" + fmt2.format(ds.headToHome) + "\n";
            rowCount = appendState(home,rowCount);
        }

        if(DroneState.widgets.yaw) {
            String yaw = "yaw: " + fmt2.format(ds.yawDeg) + "\n";
            rowCount = appendState(yaw,rowCount);
        }

        if(DroneState.widgets.pitch) {
            String pitch = "pitch: " + fmt1.format(ds.pitchDeg) + "\n";
            rowCount = appendState(pitch,rowCount);
        }

        if(DroneState.widgets.roll) {
            String roll = "roll: " + fmt1.format(ds.rollDeg) + "\n";
            rowCount = appendState(roll,rowCount);
        }

        if(DroneState.widgets.alt) {
            String alt = "alt: " + fmt1.format(ds.altitude) + "\n";
            rowCount = appendState(alt,rowCount);
        }

        if(DroneState.widgets.vertSpeed) {
            String vertSpeed = "verSpd: " + fmt1.format(ds.velUp) + "\n";
            rowCount = appendState(vertSpeed,rowCount);
        }

        if(DroneState.widgets.horSpeed) {
            String horSpeed = "horSpd: " + fmt1.format(ds.velGnd) + "\n";
            rowCount = appendState(horSpeed,rowCount);
        }

        if(DroneState.widgets.temperature) {
            String temperature = "temp: " + fmt1.format(ds.temperature) + "\n";
            rowCount = appendState(temperature,rowCount);
        }

        if(DroneState.widgets.pressure) {
            String pressure = "press: " + fmt1.format(ds.baro.pressure) + "\n";
            rowCount = appendState(pressure,rowCount);
        }

        if(DroneState.widgets.gx) {
            String gx = "gx: " + fmt2.format(ds.gyro.rawX) + "\n";
            rowCount = appendState(gx,rowCount);
        }

        if(DroneState.widgets.gy) {
            String gy = "gy: " + fmt2.format(ds.gyro.rawY) + "\n";
            rowCount = appendState(gy,rowCount);
        }

        if(DroneState.widgets.gz) {
            String gz = "gz: " + fmt2.format(ds.gyro.rawZ) + "\n";
            rowCount = appendState(gz,rowCount);
        }

        if(DroneState.widgets.ax) {
            String ax = "ax: " + fmt2.format(ds.accel.rawX) + "\n";
            rowCount = appendState(ax,rowCount);
        }

        if(DroneState.widgets.ay) {
            String ay = "ay: " + fmt2.format(ds.accel.rawY) + "\n";
            rowCount = appendState(ay,rowCount);
        }

        if(DroneState.widgets.az) {
            String az = "az: " + fmt2.format(ds.accel.rawZ) + "\n";
            rowCount = appendState(az,rowCount);
        }

        DecimalFormat fmtLatLon = new DecimalFormat();
        fmtLatLon.setMinimumIntegerDigits(3);
        fmtLatLon.setMinimumFractionDigits(6);
        fmtLatLon.setMaximumFractionDigits(6);
        if(DroneState.widgets.lat) {
            String lat = "lat: " + fmtLatLon.format(ds.gps.lat) + "\n";
            rowCount = appendState(lat,rowCount);
        }

        if(DroneState.widgets.lon) {
            String lon = "lon: " + fmtLatLon.format(ds.gps.lon) + "\n";
            rowCount = appendState(lon,rowCount);
        }

        if(DroneState.widgets.sattels) {
            String sat = "sat: " + fmt2.format(ds.gps.numSV) + "\n";
            rowCount = appendState(sat,rowCount);
        }
    }

    private void printAlarm(Alarm alarm) {
        if(DroneAlarmCenter.instance().getAlarm(Alarm.ALARM_CONNECTING)) {
            mtvAlarm.setText(TextBox.get("ALARM_CONNECTING"));
            mtvAlarm.setTextColor(Color.YELLOW);
            return;
        }

        if(DroneAlarmCenter.instance().getAlarm(Alarm.ALARM_UNSUPPORTED_FIRMWARE)) {
            mtvAlarm.setText(TextBox.get("ALARM_UNSUPPORTED_FIRMWARE"));
            mtvAlarm.setTextColor(Color.RED);
        }

        if (alarm == null) {
            mtvAlarm.setText(TextBox.get("SYSTEM_OK"));
            mtvAlarm.setTextColor(Color.GREEN);
            return;
        }

        mtvAlarm.setText(TextBox.get(alarm.name()));
        mtvAlarm.setTextColor(Color.RED);
    }

    public static int keyTurnCw;
    public static int keyTurnCcw;
    public static int keyForward;
    public static int keyBackward;
    public static int keyLeft;
    public static int keyRight;
    public static int keyUp;
    public static int keyDown;
    public static int keyDisarm;
    public static int keyLoad1;
    public static int keyLoad2;
    public static int keyTrickGyro;
    public static int keyTrickAcro;
    public static int keyTurtle;
    public static int keyCam0;
    public static int keyCam45;
    public static int keyCam90;
    public static int keyVideo;
    public static int keyPhoto;

    public static void loadGamepadSettings(Context context) {
        SharedPreferences sharedPref = context.getSharedPreferences(
                context.getString(R.string.gamepad_file_key), Context.MODE_PRIVATE);

        int defKeyTurnCw = context.getResources().getInteger(R.integer.key_turn_cw);
        keyTurnCw = sharedPref.getInt(context.getString(R.string.key_turn_cw), defKeyTurnCw);

        int defKeyTurnCcw = context.getResources().getInteger(R.integer.key_turn_ccw);
        keyTurnCcw = sharedPref.getInt(context.getString(R.string.key_turn_ccw), defKeyTurnCcw);

        int defKeyUp = context.getResources().getInteger(R.integer.key_lift_up);
        keyUp = sharedPref.getInt(context.getString(R.string.key_lift_up), defKeyUp);

        int defKeyDown = context.getResources().getInteger(R.integer.key_lift_down);
        keyDown = sharedPref.getInt(context.getString(R.string.key_lift_down), defKeyDown);

        int defKeyLeft = context.getResources().getInteger(R.integer.key_left);
        keyLeft = sharedPref.getInt(context.getString(R.string.key_left), defKeyLeft);

        int defKeyRight = context.getResources().getInteger(R.integer.key_right);
        keyRight = sharedPref.getInt(context.getString(R.string.key_right), defKeyRight);

        int defKeyForward = context.getResources().getInteger(R.integer.key_forward);
        keyForward = sharedPref.getInt(context.getString(R.string.key_forward), defKeyForward);

        int defKeyBackward = context.getResources().getInteger(R.integer.key_backward);
        keyBackward = sharedPref.getInt(context.getString(R.string.key_backward), defKeyBackward);

        int defKeyDisarm = context.getResources().getInteger(R.integer.key_disarm_motors);
        keyDisarm = sharedPref.getInt(context.getString(R.string.key_disarm_motors), defKeyDisarm);

        int defKeyLoad1 = context.getResources().getInteger(R.integer.key_load1);
        keyLoad1 = sharedPref.getInt(context.getString(R.string.key_load1), defKeyLoad1);

        int defKeyLoad2 = context.getResources().getInteger(R.integer.key_load2);
        keyLoad2 = sharedPref.getInt(context.getString(R.string.key_load2), defKeyLoad2);

        int defKeyTrickGyro = context.getResources().getInteger(R.integer.key_trick_gyro);
        keyTrickGyro = sharedPref.getInt(context.getString(R.string.key_trick_gyro), defKeyTrickGyro);

        int defKeyTrickAcro = context.getResources().getInteger(R.integer.key_trick_acro);
        keyTrickAcro = sharedPref.getInt(context.getString(R.string.key_trick_acro), defKeyTrickAcro);

        int defKeyTurtle = context.getResources().getInteger(R.integer.key_turtle);
        keyTurtle = sharedPref.getInt(context.getString(R.string.key_turtle), defKeyTurtle);

        int defKeyCam0 = context.getResources().getInteger(R.integer.key_cam0);
        keyCam0 = sharedPref.getInt(context.getString(R.string.key_cam0), defKeyCam0);

        int defKeyCam45 = context.getResources().getInteger(R.integer.key_cam45);
        keyCam45 = sharedPref.getInt(context.getString(R.string.key_cam45), defKeyCam45);

        int defKeyCam90 = context.getResources().getInteger(R.integer.key_cam90);
        keyCam90 = sharedPref.getInt(context.getString(R.string.key_cam90), defKeyCam90);

        int defKeyVideo = context.getResources().getInteger(R.integer.key_video);
        keyVideo = sharedPref.getInt(context.getString(R.string.key_video), defKeyVideo);

        int defKeyPhoto = context.getResources().getInteger(R.integer.key_photo);
        keyPhoto = sharedPref.getInt(context.getString(R.string.key_photo), defKeyPhoto);
    }

    public static void saveGamepadSettings(Context context) {
        SharedPreferences sharedPref = context.getSharedPreferences(
                context.getString(R.string.gamepad_file_key), Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPref.edit();
        editor.putInt(context.getString(R.string.key_turn_cw), keyTurnCw);
        editor.putInt(context.getString(R.string.key_turn_ccw), keyTurnCcw);
        editor.putInt(context.getString(R.string.key_lift_up), keyUp);
        editor.putInt(context.getString(R.string.key_lift_down), keyDown);
        editor.putInt(context.getString(R.string.key_forward), keyForward);
        editor.putInt(context.getString(R.string.key_backward), keyBackward);
        editor.putInt(context.getString(R.string.key_left), keyLeft);
        editor.putInt(context.getString(R.string.key_right), keyRight);
        editor.putInt(context.getString(R.string.key_disarm_motors), keyDisarm);
        editor.putInt(context.getString(R.string.key_load1), keyLoad1);
        editor.putInt(context.getString(R.string.key_load2), keyLoad2);
        editor.putInt(context.getString(R.string.key_trick_gyro), keyTrickGyro);
        editor.putInt(context.getString(R.string.key_trick_acro), keyTrickAcro);
        editor.putInt(context.getString(R.string.key_turtle), keyTurtle);
        editor.putInt(context.getString(R.string.key_cam0), keyCam0);
        editor.putInt(context.getString(R.string.key_cam45), keyCam45);
        editor.putInt(context.getString(R.string.key_cam90), keyCam90);
        editor.putInt(context.getString(R.string.key_video), keyVideo);
        editor.putInt(context.getString(R.string.key_photo), keyPhoto);
        editor.apply();
    }
}
