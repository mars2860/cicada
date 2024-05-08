package ru.liftelectronica.cicada;

import android.view.InputDevice;
import android.view.InputEvent;
import android.view.KeyEvent;
import android.view.MotionEvent;

public class DPad {
    final static int ANY_KEY_RELEASED = -2;
    final static int DEFAULT = -1;
    final static int UP       = 0;
    final static int LEFT     = 1;
    final static int RIGHT    = 2;
    final static int DOWN     = 3;
    final static int CENTER   = 4;

    int directionPressed = DEFAULT;

    public int getDirectionPressed(InputEvent event) {
        if (!isDpadDevice(event)) {
            return DEFAULT;
        }

        // If the input event is a MotionEvent, check its hat axis values.
        if (event instanceof MotionEvent) {

            // Use the hat axis value to find the D-pad direction
            MotionEvent motionEvent = (MotionEvent) event;
            float xaxis = motionEvent.getAxisValue(MotionEvent.AXIS_HAT_X);
            float yaxis = motionEvent.getAxisValue(MotionEvent.AXIS_HAT_Y);

            // Check if the AXIS_HAT_X value is -1 or 1, and set the D-pad
            // LEFT and RIGHT direction accordingly.
            if (Float.compare(xaxis, -1.0f) == 0) {
                directionPressed =  DPad.LEFT;
            } else if (Float.compare(xaxis, 1.0f) == 0) {
                directionPressed =  DPad.RIGHT;
            }
            // Check if the AXIS_HAT_Y value is -1 or 1, and set the D-pad
            // UP and DOWN direction accordingly.
            else if (Float.compare(yaxis, -1.0f) == 0) {
                directionPressed =  DPad.UP;
            } else if (Float.compare(yaxis, 1.0f) == 0) {
                directionPressed =  DPad.DOWN;
            } else if(directionPressed >= 0){
                directionPressed = DPad.ANY_KEY_RELEASED;
            } else if(directionPressed == DPad.ANY_KEY_RELEASED) {
                directionPressed = DEFAULT;
            }
        }

        // If the input event is a KeyEvent, check its key code.
        else if (event instanceof KeyEvent) {

            // Use the key code to find the D-pad direction.
            KeyEvent keyEvent = (KeyEvent) event;
            if (keyEvent.getKeyCode() == KeyEvent.KEYCODE_DPAD_LEFT) {
                directionPressed = DPad.LEFT;
            } else if (keyEvent.getKeyCode() == KeyEvent.KEYCODE_DPAD_RIGHT) {
                directionPressed = DPad.RIGHT;
            } else if (keyEvent.getKeyCode() == KeyEvent.KEYCODE_DPAD_UP) {
                directionPressed = DPad.UP;
            } else if (keyEvent.getKeyCode() == KeyEvent.KEYCODE_DPAD_DOWN) {
                directionPressed = DPad.DOWN;
            } else if (keyEvent.getKeyCode() == KeyEvent.KEYCODE_DPAD_CENTER) {
                directionPressed = DPad.CENTER;
            } else if(directionPressed >= 0){
                directionPressed = DPad.ANY_KEY_RELEASED;
            } else if(directionPressed == DPad.ANY_KEY_RELEASED) {
                directionPressed = DEFAULT;
            }

        }
        return directionPressed;
    }

    public static boolean isDpadDevice(InputEvent event) {
        // Check that input comes from a device with directional pads.
        if ((event.getSource() & InputDevice.SOURCE_DPAD) != InputDevice.SOURCE_DPAD) {
            return true;
        } else {
            return false;
        }
    }
}
