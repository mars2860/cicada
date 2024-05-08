package ru.liftelectronica.cicada;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ProgressBar;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.appcompat.app.AppCompatActivity;

public class GamepadActivity extends AppCompatActivity {

    ToggleButton mbtnTurnCw;
    ToggleButton mbtnTurnCcw;
    ToggleButton mbtnForward;
    ToggleButton mbtnBackward;
    ToggleButton mbtnLeft;
    ToggleButton mbtnRight;
    ToggleButton mbtnUp;
    ToggleButton mbtnDown;

    ToggleButton mbtnDisarm;
    ToggleButton mbtnTrick;
    ToggleButton mbtnTurtle;
    ToggleButton mbtnLoad1;
    ToggleButton mbtnLoad2;
    ToggleButton mbtnCam0;
    ToggleButton mbtnCam45;
    ToggleButton mbtnCam90;
    ToggleButton mbtnVideo;
    ToggleButton mbtnPhoto;

    ProgressBar mpbTurnCwCcw;
    ProgressBar mpbFwdBck;
    ProgressBar mpbLeftRight;
    ProgressBar mpbUpDown;

    DPad dpad = new DPad();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_gamepad);

        mbtnTurnCw = (ToggleButton)findViewById(R.id.btnTurnCw);
        mbtnTurnCcw = (ToggleButton)findViewById(R.id.btnTurnCcw);
        mbtnForward = (ToggleButton)findViewById(R.id.btnForward);
        mbtnBackward = (ToggleButton)findViewById(R.id.btnBackward);
        mbtnLeft = (ToggleButton)findViewById(R.id.btnLeft);
        mbtnRight = (ToggleButton)findViewById(R.id.btnRight);
        mbtnUp = (ToggleButton)findViewById(R.id.btnUp);
        mbtnDown = (ToggleButton)findViewById(R.id.btnDown);

        mbtnDisarm = (ToggleButton)findViewById(R.id.btnDisarm);
        mbtnTrick = (ToggleButton)findViewById(R.id.btnTrick);
        mbtnTurtle = (ToggleButton)findViewById(R.id.btnTurtle);
        mbtnLoad1 = (ToggleButton)findViewById(R.id.btnLoad1);
        mbtnLoad2 = (ToggleButton)findViewById(R.id.btnLoad2);
        mbtnCam0 = (ToggleButton)findViewById(R.id.btnCam0);
        mbtnCam45 = (ToggleButton)findViewById(R.id.btnCam45);
        mbtnCam90 = (ToggleButton)findViewById(R.id.btnCam90);
        mbtnVideo = (ToggleButton)findViewById(R.id.btnVideo);
        mbtnPhoto = (ToggleButton)findViewById(R.id.btnPhoto);

        mpbTurnCwCcw = (ProgressBar)findViewById(R.id.pbTurnCwCcw);
        mpbFwdBck = (ProgressBar)findViewById(R.id.pbFwdBck);
        mpbLeftRight = (ProgressBar)findViewById(R.id.pbLeftRight);
        mpbUpDown = (ProgressBar)findViewById(R.id.pbUpDown);
    }

    @Override
    protected void onResume() {
        updateToggleBtns();
        super.onResume();
    }

    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent ev) {

        // Check that the event came from a game controller
        if ((ev.getSource() & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK &&
             ev.getAction() == MotionEvent.ACTION_MOVE) {

            int turnValue = 50 + (int)(50.0*ev.getAxisValue(MotionEvent.AXIS_X));
            int upDownValue = 50 + (int)(50.0*ev.getAxisValue(MotionEvent.AXIS_Y));
            int leftRightValue = 50 + (int)(50.0*ev.getAxisValue(MotionEvent.AXIS_Z));
            int fwdBckValue = 50 + (int)(50.0*ev.getAxisValue(MotionEvent.AXIS_RZ));

            mpbTurnCwCcw.setProgress(turnValue);
            mpbUpDown.setProgress(upDownValue);
            mpbLeftRight.setProgress(leftRightValue);
            mpbFwdBck.setProgress(fwdBckValue);

            // Check if this event if from a D-pad and process accordingly.
            if (DPad.isDpadDevice(ev)) {

                int press = dpad.getDirectionPressed(ev);
                switch (press) {
                    case DPad.LEFT:
                        mapGamepadKey(KeyEvent.KEYCODE_DPAD_LEFT);
                        break;
                    case DPad.RIGHT:
                        mapGamepadKey(KeyEvent.KEYCODE_DPAD_RIGHT);
                        break;
                    case DPad.UP:
                        mapGamepadKey(KeyEvent.KEYCODE_DPAD_UP);
                        break;
                    case DPad.DOWN:
                        mapGamepadKey(KeyEvent.KEYCODE_DPAD_DOWN);
                        break;
                    case DPad.CENTER:
                        mapGamepadKey(KeyEvent.KEYCODE_DPAD_CENTER);
                        break;
                }
                uncheckAllToggleBtns();
            }

            return true;
        }

        return super.dispatchGenericMotionEvent(ev);
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {

        if ((event.getSource() & InputDevice.SOURCE_GAMEPAD) == InputDevice.SOURCE_GAMEPAD) {
            if (event.getRepeatCount() == 0) {

                mapGamepadKey(event.getKeyCode());

                uncheckAllToggleBtns();

                return true;
            }
        }

        return super.dispatchKeyEvent(event);
    }

    public void onBtnCancel(View v) {
        super.onBackPressed();
    }

    public void onBtnApply(View v) {
        MainActivity.saveGamepadSettings(this);
        Toast.makeText(this, R.string.saved_successfully, Toast.LENGTH_SHORT).show();
    }

    public void onToggleBtn(View v) {
        mapGamepadKey(0);
    }

    private void uncheckAllToggleBtns() {
        mbtnTurnCw.setChecked(false);
        mbtnTurnCcw.setChecked(false);
        mbtnUp.setChecked(false);
        mbtnDown.setChecked(false);
        mbtnLeft.setChecked(false);
        mbtnRight.setChecked(false);
        mbtnForward.setChecked(false);
        mbtnBackward.setChecked(false);
        mbtnDisarm.setChecked(false);
        mbtnTrick.setChecked(false);
        mbtnTurtle.setChecked(false);
        mbtnLoad1.setChecked(false);
        mbtnLoad2.setChecked(false);
        mbtnCam0.setChecked(false);
        mbtnCam45.setChecked(false);
        mbtnCam90.setChecked(false);
        mbtnVideo.setChecked(false);
        mbtnPhoto.setChecked(false);
    }

    private void mapGamepadKey(int keyCode) {
        int resetValue = keyCode;

        if(mbtnTurnCw.isChecked()) {
            MainActivity.keyTurnCw = resetValue;
        }

        if(mbtnTurnCcw.isChecked()) {
            MainActivity.keyTurnCcw = resetValue;
        }

        if(mbtnUp.isChecked()) {
            MainActivity.keyUp = resetValue;
        }

        if(mbtnDown.isChecked()) {
            MainActivity.keyDown = resetValue;
        }

        if(mbtnLeft.isChecked()) {
            MainActivity.keyLeft = resetValue;
        }

        if(mbtnRight.isChecked()) {
            MainActivity.keyRight = resetValue;
        }

        if(mbtnForward.isChecked()) {
            MainActivity.keyForward = resetValue;
        }

        if(mbtnBackward.isChecked()) {
            MainActivity.keyBackward = resetValue;
        }

        if(mbtnDisarm.isChecked()) {
            MainActivity.keyDisarm = resetValue;
        }

        if(mbtnTrick.isChecked()) {
            MainActivity.keyTrick = resetValue;
        }

        if(mbtnTurtle.isChecked()) {
            MainActivity.keyTurtle = resetValue;
        }

        if(mbtnLoad1.isChecked()) {
            MainActivity.keyLoad1 = resetValue;
        }

        if(mbtnLoad2.isChecked()) {
            MainActivity.keyLoad2 = resetValue;
        }

        if(mbtnCam0.isChecked()) {
            MainActivity.keyCam0 = resetValue;
        }

        if(mbtnCam45.isChecked()) {
            MainActivity.keyCam45 = resetValue;
        }

        if(mbtnCam90.isChecked()) {
            MainActivity.keyCam90 = resetValue;
        }

        if(mbtnVideo.isChecked()) {
            MainActivity.keyVideo = resetValue;
        }

        if(mbtnPhoto.isChecked()) {
            MainActivity.keyPhoto = resetValue;
        }

        updateToggleBtns();
    }

    private String keyCodeToString(int key) {
        if(key == 0)
            return " ";
        String name = KeyEvent.keyCodeToString(key);
        name = name.substring(8); // remove KEYCODE_
        if(name.startsWith("DPAD_"))
            name = name.substring(5);
        if(name.startsWith("BUTTON_"))
            name = name.substring(7);
        return name;
    }

    private void updateToggleBtns() {

        mbtnTurnCw.setText(keyCodeToString(MainActivity.keyTurnCw));
        mbtnTurnCw.setTextOn(keyCodeToString(MainActivity.keyTurnCw));
        mbtnTurnCw.setTextOff(keyCodeToString(MainActivity.keyTurnCw));

        mbtnTurnCcw.setText(keyCodeToString(MainActivity.keyTurnCcw));
        mbtnTurnCcw.setTextOn(keyCodeToString(MainActivity.keyTurnCcw));
        mbtnTurnCcw.setTextOff(keyCodeToString(MainActivity.keyTurnCcw));

        mbtnUp.setText(keyCodeToString(MainActivity.keyUp));
        mbtnUp.setTextOn(keyCodeToString(MainActivity.keyUp));
        mbtnUp.setTextOff(keyCodeToString(MainActivity.keyUp));

        mbtnDown.setText(keyCodeToString(MainActivity.keyDown));
        mbtnDown.setTextOn(keyCodeToString(MainActivity.keyDown));
        mbtnDown.setTextOff(keyCodeToString(MainActivity.keyDown));

        mbtnLeft.setText(keyCodeToString(MainActivity.keyLeft));
        mbtnLeft.setTextOn(keyCodeToString(MainActivity.keyLeft));
        mbtnLeft.setTextOff(keyCodeToString(MainActivity.keyLeft));

        mbtnRight.setText(keyCodeToString(MainActivity.keyRight));
        mbtnRight.setTextOn(keyCodeToString(MainActivity.keyRight));
        mbtnRight.setTextOff(keyCodeToString(MainActivity.keyRight));

        mbtnForward.setText(keyCodeToString(MainActivity.keyForward));
        mbtnForward.setTextOn(keyCodeToString(MainActivity.keyForward));
        mbtnForward.setTextOff(keyCodeToString(MainActivity.keyForward));

        mbtnBackward.setText(keyCodeToString(MainActivity.keyBackward));
        mbtnBackward.setTextOn(keyCodeToString(MainActivity.keyBackward));
        mbtnBackward.setTextOff(keyCodeToString(MainActivity.keyBackward));

        mbtnDisarm.setText(keyCodeToString(MainActivity.keyDisarm));
        mbtnDisarm.setTextOn(keyCodeToString(MainActivity.keyDisarm));
        mbtnDisarm.setTextOff(keyCodeToString(MainActivity.keyDisarm));

        mbtnTrick.setText(keyCodeToString(MainActivity.keyTrick));
        mbtnTrick.setTextOn(keyCodeToString(MainActivity.keyTrick));
        mbtnTrick.setTextOff(keyCodeToString(MainActivity.keyTrick));

        mbtnTurtle.setText(keyCodeToString(MainActivity.keyTurtle));
        mbtnTurtle.setTextOn(keyCodeToString(MainActivity.keyTurtle));
        mbtnTurtle.setTextOff(keyCodeToString(MainActivity.keyTurtle));

        mbtnLoad1.setText(keyCodeToString(MainActivity.keyLoad1));
        mbtnLoad1.setTextOn(keyCodeToString(MainActivity.keyLoad1));
        mbtnLoad1.setTextOff(keyCodeToString(MainActivity.keyLoad1));

        mbtnLoad2.setText(keyCodeToString(MainActivity.keyLoad2));
        mbtnLoad2.setTextOn(keyCodeToString(MainActivity.keyLoad2));
        mbtnLoad2.setTextOff(keyCodeToString(MainActivity.keyLoad2));

        mbtnCam0.setText(keyCodeToString(MainActivity.keyCam0));
        mbtnCam0.setTextOn(keyCodeToString(MainActivity.keyCam0));
        mbtnCam0.setTextOff(keyCodeToString(MainActivity.keyCam0));

        mbtnCam45.setText(keyCodeToString(MainActivity.keyCam45));
        mbtnCam45.setTextOn(keyCodeToString(MainActivity.keyCam45));
        mbtnCam45.setTextOff(keyCodeToString(MainActivity.keyCam45));

        mbtnCam90.setText(keyCodeToString(MainActivity.keyCam90));
        mbtnCam90.setTextOn(keyCodeToString(MainActivity.keyCam90));
        mbtnCam90.setTextOff(keyCodeToString(MainActivity.keyCam90));

        mbtnVideo.setText(keyCodeToString(MainActivity.keyVideo));
        mbtnVideo.setTextOn(keyCodeToString(MainActivity.keyVideo));
        mbtnVideo.setTextOff(keyCodeToString(MainActivity.keyVideo));

        mbtnPhoto.setText(keyCodeToString(MainActivity.keyPhoto));
        mbtnPhoto.setTextOn(keyCodeToString(MainActivity.keyPhoto));
        mbtnPhoto.setTextOff(keyCodeToString(MainActivity.keyPhoto));
    }
}
