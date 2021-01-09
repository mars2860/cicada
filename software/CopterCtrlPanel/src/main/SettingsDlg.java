package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DecimalFormat;
import java.util.Locale;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.TitledBorder;

import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.commands.CmdSetPeriods;
import helper.NumericDocument;
import net.miginfocom.swing.MigLayout;

public class SettingsDlg extends JDialog
{
	private static final long serialVersionUID = 5506867286826277615L;
	
	private JIpTextField mtfCopterIp;
	private JTextField mtfCopterCmdPort;
	private JTextField mtfCopterTelemetryPort;
	private JTextField mtfCopterVideoPort;
	private JTextField mtfTelemetryPeriod;
	private JTextField mtfPidPeriod;
	private JComboBox<Locale> mcbLocale;
	private CopterCtrlPanel mMainFrame;
	
	private class OnBtnOk implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			Settings.instance().setCopterIp(mtfCopterIp.getIpAddressString());
			Settings.instance().setCopterCmdPort(Integer.parseInt(mtfCopterCmdPort.getText()));
			Settings.instance().setCopterTelemetryPort(Integer.parseInt(mtfCopterTelemetryPort.getText()));
			Settings.instance().setCopterVideoPort(Integer.parseInt(mtfCopterVideoPort.getText()));
			
			Locale locale = (Locale)mcbLocale.getSelectedItem();
			Locale.setDefault(locale);
			
			Settings.instance().setTelemetryPeriod(Integer.parseInt(mtfTelemetryPeriod.getText()));
			Settings.instance().setPidPeriod(Integer.parseInt(mtfPidPeriod.getText()));
			
			CmdSetPeriods cmd = new CmdSetPeriods(	Settings.instance().getTelemetryPeriod(),
													Settings.instance().getPidPeriod());
			
			CopterCommander.instance().addCmd(cmd);
			CopterCommander.instance().addCmd(cmd);
			CopterCommander.instance().addCmd(cmd);
			
			SettingsDlg.this.setVisible(false);
			
			mMainFrame.stop();
			mMainFrame.start();
		}
	}
	
	private class OnBtnCancel implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			SettingsDlg.this.setVisible(false);
		}
	}
	
	public SettingsDlg(CopterCtrlPanel owner)
	{
		super(owner.getMainFrame(), true);
		mMainFrame = owner;
		this.createUI();
	}
	
	private void createUI()
	{
		DecimalFormat fmt = new DecimalFormat();
		fmt.setMaximumFractionDigits(2);
		fmt.setGroupingUsed(false);
		
		JPanel pnlSettings = new JPanel(new MigLayout());

		mtfCopterIp = new JIpTextField();
		mtfCopterCmdPort = new JTextField();
		mtfCopterTelemetryPort = new JTextField();
		mtfCopterVideoPort = new JTextField();
		mtfTelemetryPeriod = new JTextField();
		mtfPidPeriod = new JTextField();
			
		mtfCopterCmdPort.setDocument(new NumericDocument(0, false));
		mtfCopterTelemetryPort.setDocument(new NumericDocument(0, false));
		mtfCopterVideoPort.setDocument(new NumericDocument(0, false));
		mtfTelemetryPeriod.setDocument(new NumericDocument(0, false));
		mtfPidPeriod.setDocument(new NumericDocument(0, false));
			
		mtfCopterCmdPort.setHorizontalAlignment(JTextField.RIGHT);
		mtfCopterTelemetryPort.setHorizontalAlignment(JTextField.RIGHT);
		mtfCopterVideoPort.setHorizontalAlignment(JTextField.RIGHT);
		mtfTelemetryPeriod.setHorizontalAlignment(JTextField.RIGHT);
		mtfPidPeriod.setHorizontalAlignment(JTextField.RIGHT);
			
		mtfCopterIp.setText(Settings.instance().getCopterIp());
		mtfCopterCmdPort.setText(Integer.toString(Settings.instance().getCopterCmdPort()));
		mtfCopterTelemetryPort.setText(Integer.toString(Settings.instance().getCopterTelemetryPort()));
		mtfCopterVideoPort.setText(Integer.toString(Settings.instance().getCopterVideoPort()));
		mtfTelemetryPeriod.setText(Integer.toString(Settings.instance().getTelemetryPeriod()));
		mtfPidPeriod.setText(Integer.toString(Settings.instance().getPidPeriod()));
		
		mcbLocale = new JComboBox<Locale>();
		
		mcbLocale.addItem(new Locale("en"));
		mcbLocale.addItem(new Locale("ru"));
		
		mcbLocale.setSelectedItem(Locale.getDefault());
		
		JPanel pnlNet = new JPanel(new MigLayout("","[40!][40!][40!]"));
		pnlNet.setBorder(new TitledBorder(ResBox.text("NET")));
		
		pnlNet.add(new JLabel(ResBox.text("IP_ADDRESS")),"span,grow,wrap");
		pnlNet.add(mtfCopterIp, "span,grow,wrap");
		
		pnlNet.add(new JLabel(ResBox.text("CMD_PORT")));
		pnlNet.add(new JLabel(ResBox.text("TELEMETRY_PORT")));
		pnlNet.add(new JLabel(ResBox.text("VIDEO_PORT")),"wrap");
		
		pnlNet.add(mtfCopterCmdPort, "grow");
		pnlNet.add(mtfCopterTelemetryPort, "grow");
		pnlNet.add(mtfCopterVideoPort, "grow, wrap");
		
		JPanel pnlCalibration = new JPanel(new MigLayout());
		pnlCalibration.setBorder(new TitledBorder(ResBox.text("CALIBRATION")));

		pnlCalibration.add(new JLabel("AccelXOffset"));
		String text = ": " + Integer.toString(Settings.instance().accel.offsetX);
		pnlCalibration.add(new JLabel(text));
		pnlCalibration.add(new JLabel("GyroXOffset"));
		text = ": " + Integer.toString(Settings.instance().getGyroXOffset());
		pnlCalibration.add(new JLabel(text),"wrap");
		
		pnlCalibration.add(new JLabel("AccelYOffset"));
		text = ": " + Integer.toString(Settings.instance().accel.offsetY);
		pnlCalibration.add(new JLabel(text));
		pnlCalibration.add(new JLabel("GyroYOffset"));
		text = ": " + Integer.toString(Settings.instance().getGyroYOffset());
		pnlCalibration.add(new JLabel(text),"wrap");
		
		pnlCalibration.add(new JLabel("AccelZOffset"));
		text = ": " + Integer.toString(Settings.instance().accel.offsetZ);
		pnlCalibration.add(new JLabel(text));
		pnlCalibration.add(new JLabel("GyroZOffset"));
		text = ": " + Integer.toString(Settings.instance().getGyroZOffset());
		pnlCalibration.add(new JLabel(text),"wrap");
		
		pnlCalibration.add(new JLabel("MagnetXOffset"));
		text = ": " + Integer.toString(Settings.instance().getMagnetXOffset());
		pnlCalibration.add(new JLabel(text));
		pnlCalibration.add(new JLabel("MagnetXScale"));
		text = ": " + fmt.format(Settings.instance().getMagnetXScale());
		pnlCalibration.add(new JLabel(text),"wrap");
		
		pnlCalibration.add(new JLabel("MagnetYOffset"));
		text = ": " + Integer.toString(Settings.instance().getMagnetYOffset());
		pnlCalibration.add(new JLabel(text));
		pnlCalibration.add(new JLabel("MagnetYScale"));
		text = ": " + fmt.format(Settings.instance().getMagnetYScale());
		pnlCalibration.add(new JLabel(text),"wrap");
		
		pnlCalibration.add(new JLabel("MagnetZOffset"));
		text = ": " + Integer.toString(Settings.instance().getMagnetZOffset());
		pnlCalibration.add(new JLabel(text));
		pnlCalibration.add(new JLabel("MagnetZScale"));
		text = ": " + fmt.format(Settings.instance().getMagnetZScale());
		pnlCalibration.add(new JLabel(text),"wrap");
		
		JPanel pnlPeriods = new JPanel(new MigLayout("","[][60!]"));
		pnlPeriods.setBorder(new TitledBorder(ResBox.text("UPDATE_PERIODS")));
		
		pnlPeriods.add(new JLabel(ResBox.text("TELEMETRY")));
		pnlPeriods.add(mtfTelemetryPeriod,"grow");
		pnlPeriods.add(new JLabel("Cur: " + CopterTelemetry.instance().getTelemetryPeriod()),"wrap");
		pnlPeriods.add(new JLabel(ResBox.text("PID")));
		pnlPeriods.add(mtfPidPeriod,"grow");
		pnlPeriods.add(new JLabel("Cur: " + "5"));// CopterTelemetry.instance().getPidPeriod()));

		JPanel pnlOkCancel = new JPanel(new MigLayout("insets 0 0 0 0","[80!][80!]"));
		JButton btnOk = new JButton(ResBox.text("OK"));
		JButton btnCancel = new JButton(ResBox.text("CANCEL"));
		
		btnOk.addActionListener(new OnBtnOk());
		btnCancel.addActionListener(new OnBtnCancel());
		
		pnlOkCancel.add(btnOk,"grow");
		pnlOkCancel.add(btnCancel,"grow");
		
		JPanel pnlYawPid = createPidPanel(
				ResBox.text("YAW_PID"),
				Settings.instance().getYawPidEnabled(),
				Settings.instance().getYawPidKp(),
				Settings.instance().getYawPidKi(),
				Settings.instance().getYawPidKd());
		
		JPanel pnlPitchPid = createPidPanel(
				ResBox.text("PITCH_PID"),
				Settings.instance().getPitchPidEnabled(),
				Settings.instance().getPitchPidKp(),
				Settings.instance().getPitchPidKi(),
				Settings.instance().getPitchPidKd());
		
		JPanel pnlRollPid = createPidPanel(
				ResBox.text("ROLL_PID"),
				Settings.instance().getRollPidEnabled(),
				Settings.instance().getRollPidKp(),
				Settings.instance().getRollPidKi(),
				Settings.instance().getRollPidKd());
		
		JPanel pnlAltPid = createPidPanel(
				ResBox.text("ALT_PID"),
				Settings.instance().getAltPidEnabled(),
				Settings.instance().getAltPidKp(),
				Settings.instance().getAltPidKi(),
				Settings.instance().getAltPidKd());
		
		pnlSettings.add(pnlCalibration,"spanx 2, spany 2,grow");
		
		pnlSettings.add(pnlYawPid,"grow");
		pnlSettings.add(pnlPitchPid,"grow,wrap");
		
		pnlSettings.add(pnlRollPid,"grow");
		pnlSettings.add(pnlAltPid,"grow,wrap");
		
		pnlSettings.add(pnlNet,"spanx 2, grow");
		pnlSettings.add(pnlPeriods,"spanx 2,grow,wrap");
		pnlSettings.add(new JLabel(ResBox.text("LANGUAGE")));
		pnlSettings.add(mcbLocale, "span,w 80!,wrap");
		
		pnlSettings.add(new JPanel(),"h 10!,span,wrap");
		pnlSettings.add(pnlOkCancel, "span, align right");

		this.setLayout(new MigLayout("","[grow]","[grow]"));
		this.setTitle(ResBox.text("SETTINGS"));
		this.setResizable(true);
		this.setSize(585, 500);
		this.setLocationRelativeTo(null);
		this.add(pnlSettings, "grow");
	}
	
	private JPanel createPidPanel(String title, boolean enabled, float kp, float ki, float kd)
	{
		JPanel pnl = new JPanel(new MigLayout());
		pnl.setBorder(new TitledBorder(title));
		
		DecimalFormat fmt = new DecimalFormat();
		fmt.setMaximumFractionDigits(3);
		fmt.setGroupingUsed(false);
		
		pnl.add(new JLabel(ResBox.text("PID_ENABLED")));
		pnl.add(new JLabel(": " + Boolean.toString(enabled)),"wrap");
		pnl.add(new JLabel("Kp"));
		pnl.add(new JLabel(": " + fmt.format(kp)),"wrap");
		pnl.add(new JLabel("Ki"));
		pnl.add(new JLabel(": " + fmt.format(ki)),"wrap");
		pnl.add(new JLabel("Kd"));
		pnl.add(new JLabel(": " + fmt.format(kd)),"wrap");
		
		return pnl;
	}
}