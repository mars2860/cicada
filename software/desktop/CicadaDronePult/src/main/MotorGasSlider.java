package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DecimalFormat;

import javax.swing.JLabel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import helper.NumericDocument;
import net.miginfocom.swing.MigLayout;
import pdl.DroneState;

public class MotorGasSlider extends javax.swing.JPanel
{
	private static final long serialVersionUID = -4207784842035696255L;
	
	public static final int DEFAULT_MIN_GAS = 0;
	public static final int DEFAULT_NULL_GAS = 0;
	public static final int DEFAULT_MAX_GAS = 1999;	// dshot switches motor off at value 2000!
	//public static final int MAX_GAS = 200;	// for PWM 25 kHz
	
	private JSlider mSlider;
	private JTextField mlbGas;
	private JLabel mlbGasPercent;
	private ChangeListener mcl;
	private boolean mValueAdjusting;
	private int mMinGas = DEFAULT_MIN_GAS;
	private int mNullGas = DEFAULT_NULL_GAS;
	private int mMaxGas = DEFAULT_MAX_GAS;
	
	private class OnGasChanged implements ChangeListener
	{
		@Override
		public synchronized void stateChanged(ChangeEvent e)
		{
			if(mSlider.getValueIsAdjusting())
				mValueAdjusting = true;
			
			int value = mSlider.getValue();
			float percent = value;
			
			if(value >= mNullGas)
			{
				percent /= mSlider.getMaximum();
				percent *= 100.0;
			}
			else
			{
				percent /= mSlider.getMinimum();
				percent *= 100.0;
			}
				
			DecimalFormat fmt = new DecimalFormat();
			fmt.setMaximumFractionDigits(0);
			fmt.setMinimumFractionDigits(0);
			
			mlbGas.setText(Integer.toString(value));
			mlbGasPercent.setText(fmt.format(percent) + "%");
			
			/*if(mSlider.getValueIsAdjusting() == false && mValueAdjusting == true)
			{
				if(mcl != null)
					mcl.stateChanged(e);
				
				mValueAdjusting = false;
			}*/
			if(mcl != null && mValueAdjusting)
				mcl.stateChanged(e);
			
			if(mSlider.getValueIsAdjusting() == false && mValueAdjusting == true)
				mValueAdjusting = false;
		}
	}
	
	private class OnGasSubmit implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			int gas = Integer.parseInt(mlbGas.getText());
			setGas(gas,true);
		}
	}
	
	public MotorGasSlider(String name)
	{
		mMinGas = DroneState.Motors.minGas;
		mNullGas = DroneState.Motors.nullGas;
		mMaxGas = DroneState.Motors.maxGas;
		
		this.createVertical(name);
	}
	
	public MotorGasSlider(String name, boolean horizontal)
	{
		mMinGas = DroneState.Motors.minGas;
		mNullGas = DroneState.Motors.nullGas;
		mMaxGas = DroneState.Motors.maxGas;
		
		if(horizontal)
			this.createHorizontal(name);
		else
			this.createVertical(name);
	}
	
	private void createVertical(String name)
	{
		this.setLayout(new MigLayout("insets 0 0 0 0","[35!, center]","[][grow][][]"));
		
		mSlider = new JSlider(JSlider.VERTICAL, mMinGas, mMaxGas, mNullGas);
		mlbGas = new JTextField();
		mlbGasPercent = new JLabel();
		
		mlbGas.setHorizontalAlignment(JTextField.CENTER);
		mlbGas.setDocument(new NumericDocument(0,false));
		mlbGas.addActionListener(new OnGasSubmit());
		
		mSlider.setMinorTickSpacing(1);
		mSlider.addChangeListener(new OnGasChanged());
		// To update labels set value
		mSlider.setValue(1);
		mSlider.setValue(0);
		
		this.add(new JLabel(name),"wrap");
		this.add(mSlider,"grow,wrap");
		this.add(mlbGas,"grow,wrap");
		this.add(mlbGasPercent);
	}
	
	private void createHorizontal(String name)
	{
		this.setLayout(new MigLayout("insets 0 0 0 0","[][grow][][]","[center]"));
		
		mSlider = new JSlider(JSlider.HORIZONTAL, mMinGas, mMaxGas, mNullGas);
		mlbGas = new JTextField();
		mlbGasPercent = new JLabel();
		
		mlbGas.setHorizontalAlignment(JLabel.CENTER);
		mlbGasPercent.setHorizontalAlignment(JLabel.CENTER);
		
		mSlider.setMinorTickSpacing(1);
		mSlider.addChangeListener(new OnGasChanged());
		// To update labels set value
		mSlider.setValue(1);
		mSlider.setValue(0);
		
		this.add(new JLabel(name));
		this.add(mSlider,"growx");
		this.add(mlbGas,"w 35!");
		this.add(mlbGasPercent,"w 30!");
	}
	
	public void addChangeListener(ChangeListener listener)
	{
		mcl = listener;
	}
	
	public void addGas(int gas)
	{
		gas += mSlider.getValue();
		this.setGas(gas, true);
	}
	
	public int getGas()
	{
		return mSlider.getValue();
	}
	
	public synchronized void setGas(int gas, boolean invokeListener)
	{
		if(!mValueAdjusting)
		{
			if(invokeListener)
				mSlider.setValueIsAdjusting(true);
			
			mSlider.setValue(gas);
			
			if(invokeListener)
				mSlider.setValueIsAdjusting(false);
		}
	}
	
	public void setMinMax(int min, int max)
	{
		mSlider.setMinimum(min);
		mSlider.setMaximum(max);
	}
}
