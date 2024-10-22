/*
 * AndroidPort.java
 *
 *       Created on:  Feb 15, 2022
 *  Last Updated on:  Jul 27, 2023
 *           Author:  Will Hedgecock
 *
 * Copyright (C) 2022-2023 Fazecast, Inc.
 *
 * This file is part of jSerialComm.
 *
 * jSerialComm is free software: you can redistribute it and/or modify
 * it under the terms of either the Apache Software License, version 2, or
 * the GNU Lesser General Public License as published by the Free Software
 * Foundation, version 3 or above.
 *
 * jSerialComm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of both the GNU Lesser General Public
 * License and the Apache Software License along with jSerialComm. If not,
 * see <http://www.gnu.org/licenses/> and <http://www.apache.org/licenses/>.
 */

package com.fazecast.jSerialComm.android;

import com.fazecast.jSerialComm.SerialPort;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;
import android.widget.Toast;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.util.List;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

public class AndroidPort
{
	private static final String LOG_TAG = "jSerialComm";

	private static final int WRITE_TIMEOUT = 100;

	// Static shared port parameters
	protected static Context context = null;
	protected static BroadcastReceiver broadcastReceiver = null;
	protected static UsbManager usbManager = null;

	// Static private port parameters
	//private static final String ACTION_USB_PERMISSION =  BuildConfig.APPLICATION_ID + ".USB_PERMISSION";
	private static final String ACTION_USB_PERMISSION = "com.fazecast.jSerialComm.USB_PERMISSION";
	private boolean userPermissionGranted = false, awaitingUserPermission = false;

	// Shared serial port parameters
	protected UsbSerialPort usbSerialPort;
	protected UsbSerialDriver usbSerialDriver;
	protected UsbDeviceConnection usbConnection = null;

	// Private constructor so that class can only be created by the enumeration method
	protected AndroidPort(UsbSerialDriver drv) {
		usbSerialDriver = drv;

		// Register a listener to handle permission request responses
		if(broadcastReceiver != null) {
			try {
				context.unregisterReceiver(broadcastReceiver);
			}
			catch (Exception ex) {
				ex.printStackTrace();
			}
			broadcastReceiver = null;
		}
		broadcastReceiver = new BroadcastReceiver() {
			public void onReceive(Context context, Intent intent) {
				String action = intent.getAction();
				if(action == null)
					return;
				if (action.equals(ACTION_USB_PERMISSION)) {
					synchronized (AndroidPort.class) {
						userPermissionGranted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
						awaitingUserPermission = false;
						AndroidPort.class.notifyAll();
					}
				} else if (action.equals(UsbManager.ACTION_USB_DEVICE_DETACHED)) {
					UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
					if (device != null) {
						Log.i(LOG_TAG, "Port was disconnected");
						//AndroidPort.this.closePortNative();
					}
				}
			}
		};

		IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
		filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);

		context.registerReceiver(broadcastReceiver, filter);
	}

	// Method to set the Android application context
	public static void setAndroidContext(Object androidContext) {
		if(androidContext == null && context != null && broadcastReceiver != null) {
			try {
				context.unregisterReceiver(broadcastReceiver);
			}
			catch (Exception ex) {
				ex.printStackTrace();
			}
			broadcastReceiver = null;
		}
		context = (Context) androidContext;
	}

	// Port enumeration method
	public static SerialPort[] getCommPortsNative()
	{
		// Ensure that the Android application context has been specified 
		if (context == null)
			throw new RuntimeException("The Android application context must be specified using 'setAndroidContext()' before making any jSerialComm library calls.");

		// Ensure that the device has a USB Manager
		if (!context.getPackageManager().hasSystemFeature(PackageManager.FEATURE_USB_HOST)) {
			Log.e(LOG_TAG,"App doesn't have FEATURE_USB_HOST");
			return new SerialPort[0];
		}

		// Enumerate all serial ports on the device
		usbManager = (UsbManager)context.getApplicationContext().getSystemService(Context.USB_SERVICE);
		List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);
		if (availableDrivers.isEmpty()) {
			return new SerialPort[0];
		}
		SerialPort[] portsList = new SerialPort[availableDrivers.size()];

		// Create and return the SerialPort port listing
		int i = 0;
		for (UsbSerialDriver drv : availableDrivers) {
			UsbDevice device = drv.getDevice();
			// Create a new serial port object and add it to the port listing
			SerialPort serialPort;
			try {
				AndroidPort androidPort = new AndroidPort(drv);
				Constructor<SerialPort> serialPortConstructor = SerialPort.class.getDeclaredConstructor(String.class, String.class, String.class, String.class, String.class, String.class, int.class, int.class);
				serialPortConstructor.setAccessible(true);
				serialPort = serialPortConstructor.newInstance("COM" + (i+1), device.getDeviceName(), device.getDeviceName(), device.getProductName(), device.getSerialNumber(), device.getSerialNumber(), device.getVendorId(), device.getProductId());
				Field privateAndroidPort = SerialPort.class.getDeclaredField("androidPort");
				privateAndroidPort.setAccessible(true);
				privateAndroidPort.set(serialPort, androidPort);
				portsList[i++] = serialPort;
			} catch (Exception e) {
				Log.e(LOG_TAG,e.toString());
				continue;
			}

			Log.i(LOG_TAG, "System Port Name: " + serialPort.getSystemPortName());
			Log.i(LOG_TAG, "System Port Path: " + serialPort.getSystemPortPath());
			Log.i(LOG_TAG, "Descriptive Port Name: " + serialPort.getDescriptivePortName());
			Log.i(LOG_TAG, "Port Description: " + serialPort.getPortDescription());
			Log.i(LOG_TAG, "Serial Number: " + serialPort.getSerialNumber());
			Log.i(LOG_TAG, "Location: " + serialPort.getPortLocation());
			Log.i(LOG_TAG, "Vendor ID: " + serialPort.getVendorID());
			Log.i(LOG_TAG, "Product ID: " + serialPort.getProductID());
		}
		return portsList;
	}

	// Native port opening method
	public long openPortNative(SerialPort serialPort)
	{
		UsbDevice usbDevice = usbSerialDriver.getDevice();
		// Obtain user permission to open the port
		if (!usbManager.hasPermission(usbDevice)) {
			synchronized (AndroidPort.class) {
				awaitingUserPermission = true;

				Intent intent = new Intent(ACTION_USB_PERMISSION);
				intent.setPackage(context.getPackageName());
				PendingIntent usbPermissionIntent = PendingIntent.getBroadcast(context, 0, intent, PendingIntent.FLAG_MUTABLE);
				usbManager.requestPermission(usbDevice, usbPermissionIntent);

				try {
					AndroidPort.class.wait(5000);
				}
				catch (InterruptedException ignored) {
					// ignored
				}
				// FIXME userPermissionGranted = false after first PopUp is showed and user clicks Ok. Only if we request permissions again It gets Ok
				if (!userPermissionGranted) {
					Log.i(LOG_TAG,"Usb permission is not granted");
					return 0L;
				}
			}
		}

		try {
			if(usbManager.getDeviceList().size() == 0)
				return 0L;
			// Open and configure the port using chip-specific methods
			usbConnection = usbManager.openDevice(usbDevice);

			if ((usbConnection == null) || !openPort() || !configPort(serialPort))
				closePortNative();
		}
		catch (Exception ex) {
			Log.e(LOG_TAG,"Can't open usb device");
			usbConnection = null;
		}

		// Return whether the port was successfully opened
		return (usbConnection != null) ? 1L: 0L;
	}

	// Native port closing method
	public long closePortNative()
	{
		try {
			// Close the port using chip-specific methods
			if ((usbConnection != null) && closePort()) {
				usbConnection.close();
				usbConnection = null;
			}
		}
		catch (Exception ex) {
			Log.e(LOG_TAG,"Can't close usb connection");
			usbConnection = null;
		}

		// Return whether the port was successfully closed
		return (usbConnection == null) ? 0L : 1L;
	}

	// Shared VID/PID-to-long creation method
	//protected static long makeVidPid(int vid, int pid) { return (((long)vid << 16) & 0xFFFF0000) | ((long)pid & 0x0000FFFF); }

	// Android Port required interface
	public boolean openPort() {
		if(usbSerialPort != null) {
			closePort();
		}
		usbSerialPort = usbSerialDriver.getPorts().get(0); // Most devices have just one port (port 0)
		try {
			usbSerialPort.open(usbConnection);
			return true;
		}
		catch(IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return false;
	}
	public boolean closePort() {
		boolean result = true;
		if(usbSerialPort != null) {
			try {
				usbSerialPort.close();
			}
			catch(IOException ex) {
				Log.e(LOG_TAG,ex.toString());
				result = false;
			}
		}
		usbSerialPort = null;
		return result;
	}
	public boolean configPort(SerialPort serialPort) {
		if(usbSerialPort == null) {
			return false;
		}

		int baudRate = serialPort.getBaudRate();
		int numDataBits = serialPort.getNumDataBits();
		int numStopBits = serialPort.getNumStopBits();
		int parity = serialPort.getParity();
		// convert stopbits value
		switch(numStopBits) {
			case SerialPort.ONE_STOP_BIT:
				numStopBits = UsbSerialPort.STOPBITS_1;
				break;
			case SerialPort.ONE_POINT_FIVE_STOP_BITS:
				numStopBits = UsbSerialPort.STOPBITS_1_5;
				break;
			case SerialPort.TWO_STOP_BITS:
				numStopBits = UsbSerialPort.STOPBITS_2;
				break;
		}
		// convert parity value
		switch (parity) {
			case SerialPort.NO_PARITY:
				parity = UsbSerialPort.PARITY_NONE;
				break;
			case SerialPort.ODD_PARITY:
				parity = UsbSerialPort.PARITY_ODD;
				break;
			case SerialPort.EVEN_PARITY:
				parity = UsbSerialPort.PARITY_EVEN;
				break;
			case SerialPort.MARK_PARITY:
				parity = UsbSerialPort.PARITY_MARK;
				break;
			case SerialPort.SPACE_PARITY:
				parity = UsbSerialPort.PARITY_SPACE;
				break;
		}

		try {
			usbSerialPort.setParameters(baudRate,
					numDataBits,
					numStopBits,
					parity);
			return true;
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return false;
	}
	public boolean flushRxTxBuffers() {
		if(usbSerialPort == null)
			return false;

		try {
			usbSerialPort.purgeHwBuffers(true, true);
			return true;
		}
		catch(IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return false;
	}
	public int waitForEvent() {
		return 0;
	}

	public int bytesAvailable() {
		return 0;
	}
	public int bytesAwaitingWrite() {
		return 0;
	}
	public int readBytes(byte[] buffer, long bytesToRead, long offset, int timeoutMode, int readTimeout) {
		if(usbSerialPort == null)
			return 0;
		int result = 0;
		try {
			result = usbSerialPort.read(buffer,(int)bytesToRead,readTimeout);
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return result;
	}
	public int writeBytes(byte[] buffer, long bytesToWrite, long offset, int timeoutMode) {
		if(usbSerialPort == null)
			return 0;
		int result = 0;
		try {
			usbSerialPort.write(buffer,(int)bytesToWrite,WRITE_TIMEOUT);
			result = (int)bytesToWrite;
		}
		catch(IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return result;
	}
	public void setEventListeningStatus(boolean eventListenerRunning) {

	}
	public boolean setBreak() {
		if(usbSerialPort == null)
			return false;

		boolean result = true;
		try {
			usbSerialPort.setBreak(true);
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
			result = false;
		}
		return result;
	}
	public boolean clearBreak() {
		if(usbSerialPort == null)
			return false;
		boolean result = true;
		try {
			usbSerialPort.setBreak(false);
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
			result = false;
		}
		return result;
	}
	public boolean setRTS() {
		if(usbSerialPort == null)
			return false;
		boolean result = true;
		try {
			usbSerialPort.setRTS(true);
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
			result = false;
		}
		return result;
	}
	public boolean clearRTS() {
		if(usbSerialPort == null)
			return false;
		boolean result = true;
		try {
			usbSerialPort.setRTS(false);
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
			result = false;
		}
		return result;
	}
	public boolean setDTR() {
		if(usbSerialPort == null)
			return false;
		boolean result = true;
		try {
			usbSerialPort.setDTR(true);
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
			result = false;
		}
		return result;
	}
	public boolean clearDTR() {
		if(usbSerialPort == null)
			return false;
		boolean result = true;
		try {
			usbSerialPort.setDTR(false);
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
			result = false;
		}
		return result;
	}
	public boolean getCTS() {
		if(usbSerialPort == null)
			return false;
		boolean result = false;
		try {
			result = usbSerialPort.getCTS();
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return result;
	}
	public boolean getDSR() {
		if(usbSerialPort == null)
			return false;
		boolean result = false;
		try {
			result = usbSerialPort.getDSR();
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return  result;
	}
	public boolean getDCD() {
		return false;
	}
	public boolean getDTR() {
		if(usbSerialPort == null)
			return false;
		boolean result = false;
		try {
			result = usbSerialPort.getDTR();
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return result;
	}
	public boolean getRTS() {
		if(usbSerialPort == null)
			return false;
		boolean result = false;
		try {
			result = usbSerialPort.getRTS();
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return result;
	}
	public boolean getRI() {
		if(usbSerialPort == null)
			return false;
		boolean result = false;
		try {
			result = usbSerialPort.getRI();
		}
		catch (IOException ex) {
			Log.e(LOG_TAG,ex.toString());
		}
		return result;
	}
	public int getLastErrorLocation() {
		return 0;
	}
	public int getLastErrorCode() {
		return 0;
	}
}
