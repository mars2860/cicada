package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.io.File;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;

import main.AppSettings.WndState;
import treetable.AbstractTreeTableModel;
import treetable.JTreeTable;
import treetable.TreeTableModel;
import net.miginfocom.swing.MigLayout;
import pdl.DroneCommander;
import pdl.DroneTelemetry;
import pdl.res.Profile;
import pdl.res.SettingsNode;
import pdl.DroneState;

public class SettingsGui extends JSavedFrame
{
	// TODO Изменение текущего языка
	// TODO Перезапуск сервера при изменении сетевых настроек
	
	private static final long serialVersionUID = 1818813549822697828L;

	public class SettingsTreeTableModel extends AbstractTreeTableModel
	{
		private String mColName[] = {	ResBox.text("PARAM"),
										ResBox.text("CURRENT_VALUE")};
				
		public SettingsTreeTableModel(SettingsNode root)
		{
			super(root);
			
			// exclude RemoteCtrl because we cant edit enums inside treetable
			// RemoteCtrl we can customize in RemoteCtrlGui
			for(SettingsNode group : root.childs)
			{
				if(group.value instanceof DroneState.RemoteCtrl)
				{
					root.childs.remove(group);
					return;
				}
			}
		}

		@Override
		public int getColumnCount()
		{
			return mColName.length;
		}

		@Override
		public String getColumnName(int column)
		{
			return mColName[column];
		}

		@Override
		public Class<?> getColumnClass(int column)
		{
			if(column == 0)
				return TreeTableModel.class;                    
			
			return String.class;
		}

		@Override
		public Object getValueAt(Object node, int column)
		{
			SettingsNode n = (SettingsNode)node;
			if(column == 0)
				return n.name;
			if(column == 1 && n.childs.size() == 0)
				return n.value;
			
			return "";
		}

		@Override
		public Object getChild(Object parent, int index)
		{
			SettingsNode node = (SettingsNode)parent;
			
			if(index < node.childs.size())
				return node.childs.get(index);
			
			return null;
		}

		@Override
		public int getChildCount(Object parent)
		{
			SettingsNode node = (SettingsNode)parent;
			return node.childs.size();
		}
		
		@Override
		public boolean isCellEditable(Object node, int column)
		{
			if(column == 1 && isLeaf(node))
				return true;
			
			return super.isCellEditable(node, column);
		}
		
		@Override
		public boolean isLeaf(Object node)
		{
			SettingsNode n = (SettingsNode)node;
			if(n.childs.size() > 0)
				return false;
			return true;
		}
		
		@Override
		public void setValueAt(Object aValue, Object node, int column)
		{
			SettingsNode n = (SettingsNode)node;
			n.value = aValue;
		}
	}
	
	/*
	private class OnBtnReceive implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(DroneTelemetry.instance().isDroneConnected() == false)
			{
				JOptionPane.showMessageDialog(
						SettingsGui.this,
						ResBox.text("COPTER_NOT_FOUND"),
						"",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			DroneState ds = DroneTelemetry.instance().getDroneState();
			// Restore net settings
			// It needs because net settings are not presented inside telemetry packet
			//ds.net = Settings.instance().getDroneSettings().net;
			Settings.instance().setDroneSettings(ds);
			Node root = buildSettingsTree(ds);
			mtt.setTreeTableModel(new SettingsTreeTableModel(root));
			
			JOptionPane.showMessageDialog(
					SettingsGui.this,
					ResBox.text("SETTINGS_RECEIVED"),
					"",
					JOptionPane.INFORMATION_MESSAGE);
		}
	}
	*/
	
	private class OnBtnSend implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(DroneTelemetry.instance().isDroneConnected() == false)
				return;
			
			DroneState ds = grabNewSettings();
			
			if(ds == null)
			{
				JOptionPane.showMessageDialog(
						SettingsGui.this,
						ResBox.text("CANT_GRAB_SETTINGS"),
						ResBox.text("ERROR"),
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			if(DroneTelemetry.instance().isDroneConnected() == false)
			{
				JOptionPane.showMessageDialog(
						SettingsGui.this,
						ResBox.text("COPTER_NOT_FOUND"),
						ResBox.text("ERROR"),
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			DroneCommander.instance().sendSettingsToDrone(ds);
			
			JOptionPane.showMessageDialog(
					SettingsGui.this,
					ResBox.text("SETTINGS_SENT"),
					"",
					JOptionPane.INFORMATION_MESSAGE);
			
			// wait 1s to receive actual settings from drone
			try
			{
				Thread.sleep(1000);
			}
			catch(Exception ex)
			{
				ex.printStackTrace();
			}
			
			ds = DroneTelemetry.instance().getDroneState();
			// Fill settings tree with received settings to be sure in settings we have really on drone
			Profile.instance().setDroneSettings(ds);
			
			updateSettingsTree();
		}	
	}
	
	private class OnProfileItem implements ItemListener
	{
		@Override
		public void itemStateChanged(ItemEvent e)
		{
			if(e.getStateChange() == ItemEvent.DESELECTED)
				return;
			
			String profileName = e.getItem().toString();
			
			if(Profile.instance().getName().compareToIgnoreCase(profileName) == 0)
				return;
			
			File file = AppSettings.instance().getProfileFile(profileName);
			if(Profile.load(file) == false)
			{
				JOptionPane.showMessageDialog(	null,
												ResBox.text("CANT_LOAD_PROFILE") + ":" + file.getAbsolutePath(),
												ResBox.text("ERROR"),
												JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			AppSettings.instance().setCurProfileName(profileName);
			
			updateSettingsTree();

			JOptionPane.showMessageDialog(
					SettingsGui.this,
					ResBox.text("SETTINGS_LOADED"),
					"",
					JOptionPane.INFORMATION_MESSAGE);			
		}
	}
	
	private class OnBtnSave implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			DroneState ds = grabNewSettings();
			
			if(ds == null)
			{
				JOptionPane.showMessageDialog(
						SettingsGui.this,
						ResBox.text("CANT_GRAB_SETTINGS"),
						ResBox.text("ERROR"),
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			String profileName = JOptionPane.showInputDialog(	SettingsGui.this,
																ResBox.text("TYPE_PROFILE_NAME"),
																Profile.instance().getName());
			if(profileName == null || profileName.isEmpty())
			{
				JOptionPane.showMessageDialog(
						SettingsGui.this,
						ResBox.text("TYPE_PROFILE_NAME"),
						ResBox.text("ERROR"),
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			boolean addNewItemToCombo = false;
			
			if(profileName.compareToIgnoreCase(Profile.instance().getName()) != 0)
				addNewItemToCombo = true;
			
			Profile.instance().setDroneSettings(ds);
			
			File file = AppSettings.instance().getProfileFile(profileName);
 			
			if(Profile.instance().save(file) == true)
			{
				// Select new profile in combo box
				if(addNewItemToCombo)
				{
					cbProfiles.addItem(profileName);
				}
				
				ItemListener listeners[] = cbProfiles.getItemListeners();
				// don't fire item changed event
				cbProfiles.removeItemListener(listeners[0]);
				cbProfiles.setSelectedItem(profileName);
				cbProfiles.addItemListener(listeners[0]);
				
				JOptionPane.showMessageDialog(
						SettingsGui.this,
						ResBox.text("SETTINGS_SAVED"),
						"",
						JOptionPane.INFORMATION_MESSAGE);	
			}
			else
			{
				JOptionPane.showMessageDialog(
						SettingsGui.this,
						ResBox.text("SETTINGS_SAVED"),
						"",
						JOptionPane.ERROR_MESSAGE);
			}
		}
	}
	
	private class OnBtnSaveDefault implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(DroneTelemetry.instance().isDroneConnected() == false)
				return;
			
			DroneCommander.instance().saveDefaultCfg();
			
			JOptionPane.showMessageDialog(
					SettingsGui.this,
					ResBox.text("SETTINGS_SAVED_DEFAULT"),
					"",
					JOptionPane.INFORMATION_MESSAGE);	
		}
	}
	
	private class OnBtnLoadDefault implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(DroneTelemetry.instance().isDroneConnected() == false)
				return;
			
			DroneCommander.instance().loadDefaultCfg();
			
			JOptionPane.showMessageDialog(
					SettingsGui.this,
					ResBox.text("SETTINGS_LOADED_DEFAULT"),
					"",
					JOptionPane.INFORMATION_MESSAGE);
			
			DroneState ds = DroneTelemetry.instance().getDroneState();
			// Fill settings tree with received settings to be sure in settings we have really on drone
			Profile.instance().setDroneSettings(ds);
			
			updateSettingsTree();
		}
	}
	
	private JTreeTable mtt;
	private JComboBox<String> cbProfiles;
	
	public SettingsGui()
	{
		super("Settings",640,480);
		this.setTitle(ResBox.text("SETTINGS"));
		this.setIconImage(ResBox.icon("SETTINGS").getImage());
		this.createUI();
	}
	
	private void updateSettingsTree()
	{
		SettingsNode root = Profile.instance().buildSettingsTree();

		if(mtt == null)
		{
			mtt = new JTreeTable(new SettingsTreeTableModel(root));
			mtt.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		}
		else
		{
			mtt.setTreeTableModel(new SettingsTreeTableModel(root));
		}
	}
	
	private void createUI()
	{
		JPanel pnlBtns = new JPanel(new MigLayout("","[][][][grow][][]"));
		
		cbProfiles = new JComboBox<String>();
		for(String item : AppSettings.instance().listProfiles())
		{
			cbProfiles.addItem(item);
		}
		cbProfiles.setSelectedItem(AppSettings.instance().getCurProfileName());
		cbProfiles.addItemListener(new OnProfileItem());
		
		updateSettingsTree();
		
		//JButton btnReceive = new JButton(ResBox.text("RECEIVE"));
		JButton btnSend = new JButton(ResBox.text("SEND"));
		JButton btnSave = new JButton(ResBox.text("SAVE"));
		JButton btnSaveDefault = new JButton(ResBox.text("SAVE_DEFAULT_CFG"));
		JButton btnLoadDefault = new JButton(ResBox.text("LOAD_DEFAULT_CFG"));
		
		//btnReceive.addActionListener(new OnBtnReceive());
		btnSend.addActionListener(new OnBtnSend());
		btnSave.addActionListener(new OnBtnSave());
		btnSaveDefault.addActionListener(new OnBtnSaveDefault());
		btnLoadDefault.addActionListener(new OnBtnLoadDefault());
		
		pnlBtns.add(cbProfiles,"w 150!");
		//pnlBtns.add(btnReceive,"w 80!");
		pnlBtns.add(btnSend,"w 80!");
		pnlBtns.add(btnSave,"w 80!");
		pnlBtns.add(new JPanel(),"grow");
		pnlBtns.add(btnLoadDefault);
		pnlBtns.add(btnSaveDefault);
		
		JPanel pnlSettings = new JPanel(new MigLayout("","[grow]","[][grow]"));
		
		pnlSettings.add(pnlBtns,"grow,wrap");
		pnlSettings.add(new JScrollPane(mtt),"grow");
		
		this.add(pnlSettings);
	}

	private DroneState grabNewSettings()
	{
		DroneState ds = new DroneState();
		SettingsNode root = (SettingsNode)mtt.getTreeTableModel().getRoot();
		
		for(SettingsNode group : root.childs)
		{
			for(SettingsNode param : group.childs)
			{
				try
				{
					Object settingGroupObj = param.settingGroup.get(ds);
					Object value = param.value;
					String type = param.settingField.getGenericType().toString();

					if(type.startsWith("int"))
					{
						int i = Integer.parseInt(value.toString());
						param.settingField.setInt(settingGroupObj, i);
					}
					else if(type.startsWith("double"))
					{
						double d = Double.parseDouble(value.toString());
						param.settingField.setDouble(settingGroupObj, d);
					}
					else if(type.startsWith("float"))
					{
						float f = Float.parseFloat(value.toString());
						param.settingField.setFloat(settingGroupObj, f);
					}
					else if(type.startsWith("bool"))
					{
						boolean b = Boolean.parseBoolean(value.toString());
						param.settingField.setBoolean(settingGroupObj, b);
					}
					else if(type.contains("String"))
					{
						param.settingField.set(settingGroupObj, value.toString());
					}
				}
				catch(NumberFormatException e)
				{
					String text = ResBox.text("INVALID_VALUE") + " " + param.name; 
					JOptionPane.showMessageDialog(
							SettingsGui.this,
							text,
							ResBox.text("ERROR"),
							JOptionPane.ERROR_MESSAGE);
					return null;
				}
				catch (IllegalArgumentException e)
				{
					e.printStackTrace();
					return null;
				} catch (IllegalAccessException e)
				{
					e.printStackTrace();
					return null;
				}
			}
		}
		
		return ds;
	}

	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getSettingsWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setSettingsWnd(ws);
	}
}
