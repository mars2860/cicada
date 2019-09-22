package main;

import java.text.DecimalFormat;

import javax.swing.JLabel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import net.miginfocom.swing.MigLayout;

public class MotorGasSlider extends javax.swing.JPanel
{
	private static final long serialVersionUID = -4207784842035696255L;
	
	private JSlider mSlider;
	private JLabel mlbGas;
	private JLabel mlbGasPercent;
	
	private class OnGasChanged implements ChangeListener
	{
		@Override
		public void stateChanged(ChangeEvent e)
		{
			//if(!mSlider.getValueIsAdjusting())
			{
				int value = mSlider.getValue();
				float percent = value;
				percent /= mSlider.getMaximum();
				percent *= 100.0;
				
				DecimalFormat fmt = new DecimalFormat();
				fmt.setMaximumFractionDigits(0);
				fmt.setMinimumFractionDigits(0);
				
				mlbGas.setText(Integer.toString(value));
				mlbGasPercent.setText(fmt.format(percent) + "%");
	        }
		}
	}
	
	public MotorGasSlider(String name)
	{
		this.createVertical(name);
	}
	
	public MotorGasSlider(String name, boolean horizontal)
	{
		if(horizontal)
			this.createHorizontal(name);
		else
			this.createVertical(name);
	}
	
	private void createVertical(String name)
	{
		this.setLayout(new MigLayout("insets 0 0 0 0","[30!, center]","[][grow]"));
		
		mSlider = new JSlider(JSlider.VERTICAL, 0, 255, 0);
		mlbGas = new JLabel();
		mlbGasPercent = new JLabel();
		
		mSlider.setMinorTickSpacing(1);
		mSlider.addChangeListener(new OnGasChanged());
		// To update labels set value
		mSlider.setValue(1);
		mSlider.setValue(0);
		
		this.add(new JLabel(name),"wrap");
		this.add(mSlider,"growy,wrap");
		this.add(mlbGas,"wrap");
		this.add(mlbGasPercent);
	}
	
	private void createHorizontal(String name)
	{
		this.setLayout(new MigLayout("insets 0 0 0 0","[][grow]","[center]"));
		
		mSlider = new JSlider(JSlider.HORIZONTAL, 0, 255, 0);
		mlbGas = new JLabel();
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
		this.add(mlbGas,"w 30!");
		this.add(mlbGasPercent,"w 30!");
	}
	
	public void addChangeListener(ChangeListener listener)
	{
		mSlider.addChangeListener(listener);
	}
	
	public int getGas()
	{
		return mSlider.getValue();
	}
	
	/** valueAdjust - see JSlider.setValueIsAdjusting */
	public void setGas(int gas, boolean valueAdjust)
	{
		mSlider.setValueIsAdjusting(valueAdjust);
		mSlider.setValue(gas);
	}
}
