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

import main.AppSettings.WndState;
import net.miginfocom.swing.MigLayout;
import pdl.DroneCommander;
import pdl.DroneTelemetry;
import pdl.DroneState;
import pdl.commands.CmdEnableStabilization;
import pdl.commands.CmdSetBaseGas;
import pdl.commands.CmdSetMotorsDir;
import pdl.commands.CmdSetMotorsGas;
import pdl.commands.CmdSwitchMotors;

public class MotorsGui extends JSavedFrame
{
	private static final long serialVersionUID = -6961786603255564187L;
	
	private JCheckBox mcbMotorsEnabled;
	private JCheckBox mcbStabilizationEnabled;
	private JCheckBox mcbReverse;
	private JCheckBox mcbHoldPosEnabled;
	private JCheckBox mcbTrickMode;
	/*private MotorGasSlider mgas0;
	private MotorGasSlider mgas1;
	private MotorGasSlider mgas2;
	private MotorGasSlider mgas3;*/
	private MotorGasSlider mgas[];
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
				
				DroneCommander.instance().addCmd(mCmdSetGas);
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
				DroneCommander.instance().addCmd(new CmdSetBaseGas(value));
				// также меняем газ в одиночной команде, чтобы потом
				// когда изменили газ одного двигателя, остальные двигатели не падали
				mCmdSetGas.setGasForAll(value);
	        }
		}	
	}
	
	public class OnMotorsEnabled implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			boolean state = !DroneTelemetry.instance().getDroneState().motorsEnabled;
			DroneCommander.instance().addCmd(new CmdSwitchMotors(state));		
		}
	}
	
	public class OnMotorsReverse implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			DroneCommander.instance().addCmd(new CmdSetMotorsDir(mcbReverse.isSelected()));		
		}
	}
	
	private class OnStabilizationEnabled implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			boolean state = !DroneTelemetry.instance().getDroneState().stabilizationEnabled;
			DroneCommander.instance().addCmd(new CmdEnableStabilization(state));
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
			
			DroneState droneState = DroneTelemetry.instance().getDroneState();
			
			//mgas0.setGas((int)droneState.motorGas0,false);
			//mgas1.setGas((int)droneState.motorGas1,false);
			//mgas2.setGas((int)droneState.motorGas2,false);
			//mgas3.setGas((int)droneState.motorGas3,false);
			
			for(int i = 0; i < DroneState.Motors.count; i++)
			{
				mgas[i].setGas((int)droneState.motorGas[i],false);
			}
			
			mgasBase.setGas((int)droneState.baseGas, false);
			
			mcbMotorsEnabled.setSelected(droneState.motorsEnabled);
			mcbStabilizationEnabled.setSelected(droneState.stabilizationEnabled);
			mcbHoldPosEnabled.setSelected((droneState.pidFlags > 0.f)?true:false);
			mcbTrickMode.setSelected((droneState.trickMode != DroneState.TrickMode.DISABLED)?true:false);
		}
	}

	public MotorsGui()
	{
		super("Motors",230,540);
		
		this.setTitle(ResBox.text("MOTORS"));
		this.setIconImage(ResBox.icon("PROPELLER").getImage());
		this.createUI();
		
		DroneTelemetry.instance().addObserver(new OnTelemetryUpdate());
	}
	
	private void createUI()
	{
		JPanel pnlMotors = new JPanel(new MigLayout("","","[][][][][][grow]"));
		
		mcbMotorsEnabled = new JCheckBox(ResBox.text("MOTORS_ENABLED"));
		mcbMotorsEnabled.addActionListener(new OnMotorsEnabled());
		
		mcbReverse = new JCheckBox(ResBox.text("REVERSE"));
		mcbReverse.addActionListener(new OnMotorsReverse());
		
		mcbStabilizationEnabled = new JCheckBox(ResBox.text("STABILIZATION_ENABLED"));
		mcbStabilizationEnabled.addActionListener(new OnStabilizationEnabled());
		
		mcbHoldPosEnabled = new JCheckBox(ResBox.text("HOLD_POS_ENABLED"));
		mcbHoldPosEnabled.setEnabled(false);
		
		mcbTrickMode = new JCheckBox(ResBox.text("TRICK_MODE"));
		mcbTrickMode.setEnabled(false);

		//mgas0 = new MotorGasSlider("M1");
		//mgas1 = new MotorGasSlider("M2");
		//mgas2 = new MotorGasSlider("M3");
		//mgas3 = new MotorGasSlider("M4");
		mgas = new MotorGasSlider[DroneState.Motors.count];
		for(int i = 0; i < DroneState.Motors.count; i++)
		{
			mgas[i] = new MotorGasSlider("M"+i);
			mgas[i].addChangeListener(new MotorGasChanged(i));
		}
		mgasBase = new MotorGasSlider("BASE");
				
		//mgas0.addChangeListener(new MotorGasChanged(0));
		//mgas1.addChangeListener(new MotorGasChanged(1));
		//mgas2.addChangeListener(new MotorGasChanged(2));
		//mgas3.addChangeListener(new MotorGasChanged(3));
		mgasBase.addChangeListener(new SetAllMotorGas());
		
		pnlMotors.add(mcbMotorsEnabled,"span,wrap");
		pnlMotors.add(mcbStabilizationEnabled,"span,wrap");
		pnlMotors.add(mcbReverse,"span,wrap");
		pnlMotors.add(mcbHoldPosEnabled,"span,wrap");
		pnlMotors.add(mcbTrickMode,"span,wrap");
		for(int i = 0; i < DroneState.Motors.count; i++)
		{
			pnlMotors.add(mgas[i],"grow");
		}
		//pnlMotors.add(mgas0,"grow");
		//pnlMotors.add(mgas1,"grow");
		//pnlMotors.add(mgas2,"grow");
		//pnlMotors.add(mgas3,"grow");
		pnlMotors.add(mgasBase,"grow");
		
		this.add(pnlMotors);
	}

	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getMotorsWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setMotorsWnd(ws);
	}
}
