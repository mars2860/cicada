package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JCheckBox;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.DroneState;
import copter.commands.CmdEnableStabilization;
import copter.commands.CmdSetBaseGas;
import copter.commands.CmdSetMotorsGas;
import copter.commands.CmdSwitchMotors;
import main.Settings.WndState;
import net.miginfocom.swing.MigLayout;

public class MotorsGui extends JSavedFrame
{
	private static final long serialVersionUID = -6961786603255564187L;
	
	private JCheckBox mcbMotorsEnabled;
	private JCheckBox mcbStabilizationEnabled;
	private MotorGasSlider mgas0;
	private MotorGasSlider mgas1;
	private MotorGasSlider mgas2;
	private MotorGasSlider mgas3;
	private MotorGasSlider mgasBase;
	private CmdSetMotorsGas mCmdSetGas = new CmdSetMotorsGas();
	
	private class MotorGasChanged implements ChangeListener
	{
		private int mChl;
		
		public MotorGasChanged(int channel)
		{
			mChl = channel;
		}
		
		@Override
		public void stateChanged(ChangeEvent e)
		{
			JSlider slider = (JSlider)e.getSource();
	        
			if(!slider.getValueIsAdjusting())
			{
				int value = slider.getValue();
				mCmdSetGas.setGas(mChl,value);
				
				CopterCommander.instance().addCmd(mCmdSetGas);
	        }	
		}
	}
	
	private class SetAllMotorGas implements ChangeListener
	{
		@Override
		public void stateChanged(ChangeEvent e)
		{
			JSlider slider = (JSlider)e.getSource();
	        
			if(!slider.getValueIsAdjusting())
			{
				int value = slider.getValue();
				CopterCommander.instance().addCmd(new CmdSetBaseGas(value));
				// также меняем газ в одиночной команде, чтобы потом
				// когда изменили газ одного двигателя, остальные двигатели не падали
				mCmdSetGas.setGasForAll(value);
	        }
		}	
	}
	
	public static class OnMotorsEnabled implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			boolean state = !CopterTelemetry.instance().getDroneState().motorsEnabled;
			CopterCommander.instance().addCmd(new CmdSwitchMotors(state));		
		}
	}
	
	private class OnStabilizationEnabled implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			boolean state = !CopterTelemetry.instance().getDroneState().stabilizationEnabled;
			CopterCommander.instance().addCmd(new CmdEnableStabilization(state));
		}
	}
	
	private class OnTelemetryUpdate implements Observer
	{
		private long timestamp;
		
		@Override
		public void update(Observable o, Object arg)
		{
			if(	System.currentTimeMillis() - timestamp < 100 ||
				MotorsGui.this.isVisible() == false )
				return;
			
			timestamp = System.currentTimeMillis();
			
			DroneState droneState = CopterTelemetry.instance().getDroneState();
			
			mgas0.setGas((int)droneState.motorGas0,false);
			mgas1.setGas((int)droneState.motorGas1,false);
			mgas2.setGas((int)droneState.motorGas2,false);
			mgas3.setGas((int)droneState.motorGas3,false);
			mgasBase.setGas((int)droneState.baseGas, false);
			
			mcbMotorsEnabled.setSelected(droneState.motorsEnabled);
			mcbStabilizationEnabled.setSelected(droneState.stabilizationEnabled);	
		}
	}

	public MotorsGui()
	{
		super("Motors",230,540);
		
		this.setTitle(ResBox.text("MOTORS"));
		this.setIconImage(ResBox.icon("PROPELLER").getImage());
		this.createUI();
		
		CopterTelemetry.instance().addObserver(new OnTelemetryUpdate());
	}
	
	private void createUI()
	{
		JPanel pnlMotors = new JPanel(new MigLayout("","","[][][grow]"));
		
		mcbMotorsEnabled = new JCheckBox(ResBox.text("MOTORS_ENABLED"));
		mcbMotorsEnabled.addActionListener(new OnMotorsEnabled());
		
		mcbStabilizationEnabled = new JCheckBox(ResBox.text("STABILIZATION_ENABLED"));
		mcbStabilizationEnabled.addActionListener(new OnStabilizationEnabled());

		mgas0 = new MotorGasSlider("M1");
		mgas1 = new MotorGasSlider("M2");
		mgas2 = new MotorGasSlider("M3");
		mgas3 = new MotorGasSlider("M4");
		mgasBase = new MotorGasSlider("BASE");
				
		mgas0.addChangeListener(new MotorGasChanged(0));
		mgas1.addChangeListener(new MotorGasChanged(1));
		mgas2.addChangeListener(new MotorGasChanged(2));
		mgas3.addChangeListener(new MotorGasChanged(3));
		mgasBase.addChangeListener(new SetAllMotorGas());
		
		/*
		MotorGasSlider yawSlider = new MotorGasSlider("Yaw");
		yawSlider.setGas((MotorGasSlider.MIN_GAS + MotorGasSlider.MAX_GAS)/2,false);
		yawSlider.addChangeListener(new ChangeListener()
		{
			int oldValue;
			
			@Override
			public void stateChanged(ChangeEvent e)
			{
				JSlider sl = (JSlider)e.getSource();
				if(!sl.getValueIsAdjusting())
				{
					int dx = sl.getValue() - oldValue;
					oldValue = sl.getValue();
					
					mgas0.setGas(mgas0.getGas() + dx,true);
					mgas2.setGas(mgas2.getGas() + dx,true);
					mgas1.setGas(mgas1.getGas() - dx,true);
					mgas3.setGas(mgas3.getGas() - dx,true);
				}
			}
		});
		*/
		
		pnlMotors.add(mcbMotorsEnabled,"span,wrap");
		pnlMotors.add(mcbStabilizationEnabled,"span,wrap");
		pnlMotors.add(mgas0,"grow");
		pnlMotors.add(mgas1,"grow");
		pnlMotors.add(mgas2,"grow");
		pnlMotors.add(mgas3,"grow");
		pnlMotors.add(mgasBase,"grow");
		//pnlMotorsGas.add(yawSlider,"grow");
		
		this.add(pnlMotors);
	}

	@Override
	protected WndState loadWndState()
	{
		return Settings.instance().getMotorsWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		Settings.instance().setMotorsWnd(ws);
	}
}
