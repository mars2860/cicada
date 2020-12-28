package main;

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

	private class SettingsTreeTableModel extends AbstractTreeTableModel
	{
		private String mColName[] = {	ResBox.text("PARAM"),
										ResBox.text("NEW_VALUE"),
										ResBox.text("CURRENT_VALUE")};
		
		private String mRootNode[] = {	ResBox.text("NET"),
										ResBox.text("ACCEL"),
										ResBox.text("GYRO"),
										ResBox.text("MAGNETO"),
										ResBox.text("YAW_RATE_PID"),
										ResBox.text("PITCH_PID"),
										ResBox.text("ROLL_PID"),
										ResBox.text("ALT_PID"),
										ResBox.text("OTHER") };
		
		public SettingsTreeTableModel()
		{
			super(ResBox.text("SETTINGS"));
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
			if(parent.toString().startsWith(ResBox.text("SETTINGS")))
				return mRootNode[index];
			
			return null;
		}

		@Override
		public int getChildCount(Object parent)
		{
			if(parent.toString().startsWith(ResBox.text("SETTINGS")))
				return mRootNode.length;
			
			return 0;
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
		JPanel pnlSettings = new JPanel(new MigLayout("","[grow]","[grow]"));
		
		JTreeTable jtt = new JTreeTable(new SettingsTreeTableModel());
		
		jtt.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		
		pnlSettings.add(new JScrollPane(jtt),"grow");
		
		this.add(pnlSettings);
	}
}
