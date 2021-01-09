package main;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;

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
		
		/*private String mRootNode[] = {	ResBox.text("NET"),
										ResBox.text("ACCEL"),
										ResBox.text("GYRO"),
										ResBox.text("MAGNETO"),
										ResBox.text("YAW_RATE_PID"),
										ResBox.text("PITCH_PID"),
										ResBox.text("ROLL_PID"),
										ResBox.text("ALT_PID"),
										ResBox.text("OTHER") };*/
		
		//private Node mRootNode;
		
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
			if(column == 0)
				return node.toString();
			
			return node.toString() + ".col" + Integer.toString(column);
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
	
	public SettingsGui()
	{
		super("Settings",640,480);
		this.setTitle(ResBox.text("SETTINGS"));
		this.setIconImage(ResBox.icon("SETTINGS").getImage());
		this.createUI();
	}
	
	private void createUI()
	{
		
		Node root = new Node(ResBox.text("SETTINGS"));
		
		// Build settings tree by settings class
		// Each nested class of Settings is Group of Params
		// Each member of that class is Param
		
		for(java.lang.reflect.Field field : Settings.class.getFields())
		{
			Class<?> paramGroup = field.getType();
			
			if(paramGroup.isMemberClass())
			{
				Node groupNode = new Node(paramGroup.getSimpleName());
				
				for(java.lang.reflect.Field param : paramGroup.getFields())
				{
					Node paramNode = new Node(param.getName());
					groupNode.childs.add(paramNode);
				}
				
				root.childs.add(groupNode);
			}
		}
		
		JPanel pnlSettings = new JPanel(new MigLayout("","[grow]","[grow]"));
		
		JTreeTable jtt = new JTreeTable(new SettingsTreeTableModel(root));
		
		jtt.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		
		pnlSettings.add(new JScrollPane(jtt),"grow");
		
		this.add(pnlSettings);
	}
}
