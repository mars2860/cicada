package main;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;

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
	private static final long serialVersionUID = 1818813549822697828L;
	
	public class Node
	{
		public String name;
		public String value;
		public List<Node> childs = new ArrayList<Node>();
		
		public Node()
		{
			
		}

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
			if (column == 0)
			{
				return TreeTableModel.class;                    
			}
			
			return String.class;
		}

		@Override
		public Object getValueAt(Object node, int column)
		{
			Node n = (Node)node;
			if(column == 0)
				return n.name;
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

		JPanel pnlSettings = new JPanel(new MigLayout("","[grow]","[grow]"));
		
		mtt = new JTreeTable(new SettingsTreeTableModel(root));
		mtt.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		
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
