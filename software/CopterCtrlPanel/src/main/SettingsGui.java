package main;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;

import copter.CopterCommander;
import copter.CopterTelemetry;
import copter.DroneState;
import copter.DroneState.SettingGroup;
import main.Settings.WndState;
import treetable.AbstractTreeTableModel;
import treetable.JTreeTable;
import treetable.TreeTableModel;
import net.miginfocom.swing.MigLayout;

public class SettingsGui extends JSavedFrame
{
	// TODO Изменение текущего языка
	// TODO Перезапуск сервера при изменении сетевых настроек
	
	private static final long serialVersionUID = 1818813549822697828L;
	
	public class Node
	{
		public String name = new String();
		public String value = new String();
		public String nValue = new String();
		// Relationship between Node and DroneState
		public java.lang.reflect.Field settingField;
		public java.lang.reflect.Field settingGroup;
		public List<Node> childs = new ArrayList<Node>();
		
		public Node() {}

		public Node(String name)
		{
			Node.this.name = name;
		}
		
		@Override
		public String toString()
		{
			return name;
		}
	}

	public class SettingsTreeTableModel extends AbstractTreeTableModel
	{
		private String mColName[] = {	ResBox.text("PARAM"),
										ResBox.text("NEW_VALUE"),
										ResBox.text("CURRENT_VALUE")};
				
		public SettingsTreeTableModel(Node root)
		{
			super(root);
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
			Node n = (Node)node;
			if(column == 0)
				return n.name;
			if(column == 1)
				return n.nValue;
			if(column == 2)
				return n.value;
			
			return "";
		}

		@Override
		public Object getChild(Object parent, int index)
		{
			Node node = (Node)parent;
			
			if(index < node.childs.size())
				return node.childs.get(index);
			
			return null;
		}

		@Override
		public int getChildCount(Object parent)
		{
			Node node = (Node)parent;
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
			Node n = (Node)node;
			if(n.childs.size() > 0)
				return false;
			return true;
		}
		
		@Override
		public void setValueAt(Object aValue, Object node, int column)
		{
			Node n = (Node)node;
			n.nValue = aValue.toString();
		}
	}
	
	private class OnBtnReceive implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(CopterTelemetry.instance().isCopterConnected() == false)
			{
				JOptionPane.showMessageDialog(
						SettingsGui.this,
						ResBox.text("COPTER_NOT_FOUND"),
						"",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			DroneState ds = CopterTelemetry.instance().getDroneState();
			// Restore net settings
			// It needs because net settings are not presented inside telemetry packet
			ds.net = Settings.instance().getDroneSettings().net;
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
	
	private class OnBtnSend implements ActionListener
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
			
			if(CopterTelemetry.instance().isCopterConnected() == false)
			{
				JOptionPane.showMessageDialog(
						SettingsGui.this,
						ResBox.text("COPTER_NOT_FOUND"),
						ResBox.text("ERROR"),
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			CopterCommander.instance().sendSettingsToDrone(ds);
			
			JOptionPane.showMessageDialog(
					SettingsGui.this,
					ResBox.text("SETTINGS_SENT"),
					"",
					JOptionPane.INFORMATION_MESSAGE);
			
			ds = CopterTelemetry.instance().getDroneState();
			// Restore net settings
			// It needs because net settings are not presented inside telemetry packet
			ds.net = Settings.instance().getDroneSettings().net;
			Settings.instance().setDroneSettings(ds);
			
			Node root = buildSettingsTree(ds);
			mtt.setTreeTableModel(new SettingsTreeTableModel(root));
		}	
	}
	
	private class OnBtnLoad implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			Settings.instance().load();
			DroneState ds = Settings.instance().getDroneSettings();
			Node root = buildSettingsTree(ds);
			mtt.setTreeTableModel(new SettingsTreeTableModel(root));

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
			Settings.instance().save();
			
			JOptionPane.showMessageDialog(
					SettingsGui.this,
					ResBox.text("SETTINGS_SAVED"),
					"",
					JOptionPane.INFORMATION_MESSAGE);
		}
	}
	
	private JTreeTable mtt;
	
	public SettingsGui()
	{
		super("Settings",640,480);
		this.setTitle(ResBox.text("SETTINGS"));
		this.setIconImage(ResBox.icon("SETTINGS").getImage());
		this.createUI();
	}
	
	private void createUI()
	{
		DroneState ds = Settings.instance().getDroneSettings();
		Node root = buildSettingsTree(ds);

		mtt = new JTreeTable(new SettingsTreeTableModel(root));
		mtt.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		
		JPanel pnlBtns = new JPanel(new MigLayout("","[][][][]"));
		
		JButton btnReceive = new JButton(ResBox.text("RECEIVE"));
		JButton btnSend = new JButton(ResBox.text("SEND"));
		JButton btnLoad = new JButton(ResBox.text("LOAD"));
		JButton btnSave = new JButton(ResBox.text("SAVE"));
		
		btnReceive.addActionListener(new OnBtnReceive());
		btnSend.addActionListener(new OnBtnSend());
		btnLoad.addActionListener(new OnBtnLoad());
		btnSave.addActionListener(new OnBtnSave());
		
		pnlBtns.add(btnReceive,"w 80!");
		pnlBtns.add(btnSend,"w 80!");
		pnlBtns.add(btnLoad,"w 80!");
		pnlBtns.add(btnSave,"w 80!");
		
		JPanel pnlSettings = new JPanel(new MigLayout("","[grow]","[][grow]"));
		
		pnlSettings.add(pnlBtns,"wrap");
		pnlSettings.add(new JScrollPane(mtt),"grow");
		
		this.add(pnlSettings);
	}
	
	private Node buildSettingsTree(DroneState ds)
	{
		Node root = new Node(ResBox.text("SETTINGS"));
		
		// Build settings tree by settings class
		// Each nested class of Settings is Group of Params
		// Each member of that class is Param
		
		for(java.lang.reflect.Field settingGroupField : DroneState.class.getFields())
		{
			SettingGroup settingGroupAnnotation = settingGroupField.getAnnotation(DroneState.SettingGroup.class);
			
			if(settingGroupAnnotation != null)
			{
				Node groupNode = new Node(ResBox.text(settingGroupAnnotation.name()));
				
				for(java.lang.reflect.Field settingField : settingGroupField.getType().getFields())
				{
					if(settingField.isAnnotationPresent(DroneState.Setting.class) == false)
						continue;
					
					Node paramNode = new Node(settingField.getName());
					
					try
					{
						paramNode.value = settingField.get(settingGroupField.get(ds)).toString();
						paramNode.settingField = settingField;
						paramNode.settingGroup = settingGroupField;
					}
					catch(Exception e)
					{
						e.printStackTrace();
					}
					
					groupNode.childs.add(paramNode);
				}
				
				root.childs.add(groupNode);
			}
		}
		
		return root;
	}
	
	private DroneState grabNewSettings()
	{
		DroneState ds = Settings.instance().getDroneSettings();
		Node root = (Node)mtt.getTreeTableModel().getRoot();
		
		for(Node group : root.childs)
		{
			for(Node param : group.childs)
			{
				if(param.nValue.isEmpty())
					continue;
				
				try
				{
					Object settingGroupObj = param.settingGroup.get(ds);
					String type = param.settingField.getGenericType().toString();

					if(type.startsWith("int"))
					{
						int i = Integer.parseInt(param.nValue);
						param.settingField.setInt(settingGroupObj, i);
					}
					else if(type.startsWith("double"))
					{
						double d = Double.parseDouble(param.nValue);
						param.settingField.setDouble(settingGroupObj, d);
					}
					else if(type.startsWith("float"))
					{
						float f = Float.parseFloat(param.nValue);
						param.settingField.setFloat(settingGroupObj, f);
					}
					else if(type.startsWith("bool"))
					{
						boolean b = Boolean.parseBoolean(param.nValue);
						param.settingField.setBoolean(settingGroupObj, b);
					}
					else if(type.contains("String"))
					{
						param.settingField.set(settingGroupObj, param.nValue);
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
		return Settings.instance().getSettingsWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		Settings.instance().setSettingsWnd(ws);
	}
}
