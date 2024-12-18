package main;

import java.awt.Component;
import java.awt.Cursor;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.FileOutputStream;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFileChooser;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.ListCellRenderer;
import javax.swing.ListSelectionModel;
import javax.swing.border.TitledBorder;
import javax.swing.filechooser.FileNameExtensionFilter;

import ChartDirector.Chart;
import ChartDirector.ChartViewer;
import ChartDirector.DrawArea;
import ChartDirector.Layer;
import ChartDirector.PlotArea;
import ChartDirector.TTFText;
import ChartDirector.TrackCursorAdapter;
import ChartDirector.ViewPortAdapter;
import ChartDirector.ViewPortChangedEvent;
import ChartDirector.XYChart;
import main.AppSettings.WndState;
import net.miginfocom.swing.MigLayout;
import pdl.DroneTelemetry;
import pdl.res.Profile;
import pdl.DroneState;

public class ChartsGui extends JSavedFrame
{
	private static final long serialVersionUID = 1426094359626415327L;
	/// microseconds
	private static final int REALTIME_CHART_RANGE = 30000000;
	/// millis
	private static final int REALTIME_CHART_UPDATE_PERIOD = 250;
	// name of a special list item to render a track of drone
	// when it renders a track of drone, no other charts draw
	private static final String TRACK_LIST_ITEM_NAME = "track";
	
	private Set<String> mSelSources = new HashSet<String>();
	private DroneState[] mData;
	private double mSelLeftTimestamp;
	private double mSelRightTimestamp;
	
	private JRadioButton rbRealtime;
	private JRadioButton rbSnapshot;
	private JButton btnSaveImage;
	private JButton btnExportSelRange;
	private JButton btnExportFullRange;
	private JButton btnClearBlackBox;
	private ChartViewer mDataChartViewer;
	private ChartMouseListener mChartMouseListener;
	private Timer mRealtimeChartTimer;
	private Object mTimerSyncObj = new Object();
	
	private boolean mDrawTrack = false;
	
	private class CheckListItem
	{
		private String label;
		private boolean isSelected = false;

		public CheckListItem(String label)
		{
			this.label = label;
		}

		public boolean isSelected()
		{
			return isSelected;
		}

		public void setSelected(boolean isSelected)
		{
			this.isSelected = isSelected;
			
			ChartsGui.this.plotDataChart(label, !isSelected);
		}

		@Override
		public String toString()
		{
			return label;
		}
	}

	private class CheckListRenderer extends JCheckBox implements ListCellRenderer<CheckListItem>
	{
		private static final long serialVersionUID = 1L;

		@Override
		public Component getListCellRendererComponent(	JList<? extends CheckListItem> list,
														CheckListItem value,
														int index,
														boolean isSelected,
														boolean cellHasFocus)
		{
			setEnabled(list.isEnabled());
			setSelected(value.isSelected());
		    setFont(list.getFont());
		    setBackground(list.getBackground());
		    setForeground(list.getForeground());
		    setText(value.toString());
		    return this;
		}
	}
	
	private class OnCheckListMouseClick extends MouseAdapter
	{
		@Override
		public void mouseClicked(MouseEvent event)
		{
			@SuppressWarnings("unchecked")
			JList<CheckListItem> list = (JList<CheckListItem>)event.getSource();
			int index = list.locationToIndex(event.getPoint());
	        CheckListItem item = list.getModel().getElementAt(index);
	        if(item.label.compareToIgnoreCase(TRACK_LIST_ITEM_NAME) == 0)
	        {
	        	mDrawTrack = !item.isSelected();
	        }
	        item.setSelected(!item.isSelected());
	        list.repaint(list.getCellBounds(index, index));
		}
	}
	
	private class OnBtnSubmit implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			if(mRealtimeChartTimer != null)
			{
				mRealtimeChartTimer.cancel();

				synchronized(mTimerSyncObj)
				{
					try
					{
						mTimerSyncObj.wait(500);
					}
					catch (InterruptedException e1)
					{
						e1.printStackTrace();
					}
				}
			}
			
			if(mChartMouseListener != null)
			{
				mDataChartViewer.removeMouseListener(mChartMouseListener);
				mDataChartViewer.removeTrackCursorListener(mChartMouseListener);
			}
			
			mDataChartViewer.setViewPortWidth(1.0);
			mDataChartViewer.setViewPortLeft(0.0);
			mDataChartViewer.setViewPortTop(0.0);
			mDataChartViewer.setViewPortHeight(1.0);
			
			if(rbSnapshot.isSelected())
			{
				mData = DroneTelemetry.instance().getBlackBox(0);
				
				if(mData.length > 0)
				{
					mDataChartViewer.setFullRange("x", mData[0].timestamp, mData[mData.length - 1].timestamp);
					mSelLeftTimestamp = mData[0].timestamp;
					mSelRightTimestamp = mSelLeftTimestamp;
					
					mDataChartViewer.setZoomInWidthLimit(100.0/mData.length);

					btnSaveImage.setEnabled(true);
					btnExportSelRange.setEnabled(true);
					btnExportFullRange.setEnabled(true);
					
					mDataChartViewer.addMouseListener(mChartMouseListener);
					mDataChartViewer.addTrackCursorListener(mChartMouseListener);
				}
				
				drawAllCharts();
			}
			else if(rbRealtime.isSelected())
			{
				btnSaveImage.setEnabled(false);
				btnExportSelRange.setEnabled(false);
				btnExportFullRange.setEnabled(false);
				
				mRealtimeChartTimer = new Timer("RealtimeChartTimer");
				mRealtimeChartTimer.scheduleAtFixedRate(
						new OnUpdateChartByTimer(), 0, REALTIME_CHART_UPDATE_PERIOD);
			}
		}
	}
	
	private class OnBtnSaveImage implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			fc.setFileFilter(new FileNameExtensionFilter("Image files",
					"png","jpg","pdf","bmp","svg"));
			 
			int userSelection = fc.showSaveDialog(ChartsGui.this);
			 
			if(userSelection == JFileChooser.APPROVE_OPTION)
			{
				File fileToSave = fc.getSelectedFile();
				if(fileToSave.getName().contains(".") == false)
				{
					fileToSave = new File(fileToSave.getPath() + ".png");
				}
			    mDataChartViewer.getChart().makeChart(fileToSave.getAbsolutePath());
			}
		}
	}
	
	private class OnBtnExportSelRange implements ActionListener
	{
		protected double getStartTimestamp()
		{
			return mSelLeftTimestamp;
		}
		
		protected double getEndTimestamp()
		{
			return mSelRightTimestamp;
		}
		
		@Override
		public void actionPerformed(ActionEvent e)
		{
			fc.setFileFilter(new FileNameExtensionFilter("CSV files","csv"));
			 
			int userSelection = fc.showSaveDialog(ChartsGui.this);
			 
			if(userSelection == JFileChooser.APPROVE_OPTION)
			{
				File fileToSave = fc.getSelectedFile();
				ChartsGui.this.exportToCsv(
						fileToSave.getAbsolutePath(), getStartTimestamp(), getEndTimestamp());
			}	
		}		
	}
	
	private class OnBtnExportFullRange extends OnBtnExportSelRange
	{
		protected double getStartTimestamp()
		{
			return mData[0].timestamp;
		}
		
		protected double getEndTimestamp()
		{
			return mData[mData.length - 1].timestamp;
		}
	}
	
	private class OnBtnClearBlackBox implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			DroneTelemetry.instance().clearBlackBox();
		}
	}
	
	private class OnFrameResize extends ComponentAdapter
	{
		@Override
		public void componentResized(ComponentEvent e)
		{
			drawAllCharts();
			super.componentResized(e);
		}
	}
	
	private class OnUpdateChartByTimer extends TimerTask
	{
		@Override
		public void run()
		{
			mData = DroneTelemetry.instance().getBlackBox(REALTIME_CHART_RANGE);
			
			if(mData.length > 0)
			{
				double start = mData[0].timestamp;
				double end = mData[mData.length - 1].timestamp;
				
				if(end - REALTIME_CHART_RANGE < start)
					end = start + REALTIME_CHART_RANGE;

				mDataChartViewer.setFullRange("x", start, end);
				mDataChartViewer.setZoomInWidthLimit(100.0/mData.length);
				
				mSelLeftTimestamp = mData[0].timestamp;
				mSelRightTimestamp = mSelLeftTimestamp;
			}
			
			ChartsGui.this.drawAllCharts();
			
			synchronized(mTimerSyncObj)
			{
				mTimerSyncObj.notify();
			}
		}
	}
	
	private class OnWindowListener extends WindowAdapter
	{
		@Override
		public void windowClosed(WindowEvent e)
		{
			if(mRealtimeChartTimer != null)
				mRealtimeChartTimer.cancel();
			
			super.windowClosed(e);
		}
	}
	
	private class OnChartViewportChanged extends ViewPortAdapter
	{
		@Override
		public void viewPortChanged(ViewPortChangedEvent e)
		{
			super.viewPortChanged(e);
			
			if(e.needUpdateChart())
				drawAllCharts();
	        
			//if(e.needUpdateImageMap())
	        //    updateImageMap(chartViewer1); 
		}
	}
	
	private class ChartMouseListener extends TrackCursorAdapter implements MouseListener
	{
		@Override
		public void mouseMovedPlotArea(MouseEvent arg0)
		{
			trackLineLabel((XYChart)mDataChartViewer.getChart(), mDataChartViewer.getPlotAreaMouseX());
			mDataChartViewer.updateDisplay();
		    // Hide the track cursor when the mouse leaves the plot area
		    mDataChartViewer.removeDynamicLayer("MouseExitedPlotArea");
		}
		
	    // Draw track line with data labels
		// copy&paste from chartdirector example
	    private void trackLineLabel(XYChart c, int mouseX)
	    {
	    	if(mData.length == 0 || c == null || mSelSources.isEmpty() || mDrawTrack)
	    		return;
	    	
	        // Clear the current dynamic layer and get the DrawArea object to draw on it.
	        DrawArea d = c.initDynamicLayer();

	        // The plot area object
	        PlotArea plotArea = c.getPlotArea();

	        // Get the data x-value that is nearest to the mouse, and find its pixel coordinate.
	        double xValue = c.getNearestXValue(mouseX);
	        int xCoor = c.getXCoor(xValue);

	        // Draw a vertical track line at the x-position
	        d.vline(plotArea.getTopY(), plotArea.getBottomY(), xCoor, d.dashLineColor(0x000000, 0x0101));

	        DecimalFormat fmt = new DecimalFormat();
	        fmt.setMaximumFractionDigits(3);
	        int minutes = (int)(xValue / 60000000.0);
	        int seconds = (int)((xValue - minutes*60000000.0)/1000000.0);
	        double mseconds = (xValue - minutes*60000000.0 - seconds*1000000.0)/1000.0;
	        // Draw a label on the x-axis to show the track line position.
	        String xlabel = "";

	        xlabel = "<*font,bgColor=000000*> " + Integer.toString(minutes) + "m" + 
	        			Integer.toString(seconds) + "s" + fmt.format(mseconds) + "ms"+ " <*/font*>";
	        
	        TTFText t = d.text(xlabel, "Arial Bold", 8);

	        // Restrict the x-pixel position of the label to make sure it stays inside the chart image.
	        int xLabelPos = Math.max(0, Math.min(xCoor - t.getWidth() / 2, c.getWidth() - t.getWidth()));
	        t.draw(xLabelPos, plotArea.getBottomY() + 6, 0xffffff);

	        // Iterate through all layers to draw the data labels
	        for(int i = 0; i < c.getLayerCount(); ++i)
	        {
	            Layer layer = c.getLayerByZ(i);

	            // The data array index of the x-value
	            int xIndex = layer.getXIndexOf(xValue);

	            // Iterate through all the data sets in the layer
	            for(int j = 0; j < layer.getDataSetCount(); ++j)
	            {
	                ChartDirector.DataSet dataSet = layer.getDataSetByZ(j);

	                // Get the color and position of the data label
	                int color = dataSet.getDataColor();
	                int yCoor = c.getYCoor(dataSet.getPosition(xIndex), dataSet.getUseYAxis());

	                // Draw a track dot with a label next to it for visible data points in the plot area
	                if( (yCoor >= plotArea.getTopY()) && 
	                	(yCoor <= plotArea.getBottomY()) &&
	                	(color != Chart.Transparent))
	                {

	                    d.circle(xCoor, yCoor, 4, 4, color, color);

	                    String label = "<*font,bgColor=" + Integer.toHexString(color) + "*> " +
	                    //c.formatValue(dataSet.getValue(xIndex), "{value|P4}") + " <*/font*>";
	                    fmt.format(dataSet.getValue(xIndex)) + " <*/font*>";
	                    t = d.text(label, "Arial Bold", 8);

	                    // Draw the label on the right side of the dot if the mouse is on the left side the chart,
	                    // and vice versa. This ensures the label will not go outside the chart image.
	                    if (xCoor <= (plotArea.getLeftX() + plotArea.getRightX()) / 2)
	                    {
	                        t.draw(xCoor + 5, yCoor, 0xffffff, Chart.Left);
	                    }
	                    else
	                    {
	                        t.draw(xCoor - 5, yCoor, 0xffffff, Chart.Right);
	                    }
	                }
	            }
	        }
	    }

		@Override
		public void mouseClicked(MouseEvent e)
		{
			int mouseX = mDataChartViewer.getPlotAreaMouseX();
			XYChart chart = (XYChart)mDataChartViewer.getChart();
			double timestamp = chart.getNearestXValue(mouseX);
			
			if(Math.abs(mSelLeftTimestamp - timestamp) < Math.abs(mSelRightTimestamp - timestamp))
			{
				mSelLeftTimestamp = timestamp;
			}
			else
			{
				mSelRightTimestamp = timestamp;
			}
			
			if(mSelLeftTimestamp > mSelRightTimestamp)
				mSelLeftTimestamp = mSelRightTimestamp;
			if(mSelRightTimestamp < mSelLeftTimestamp)
				mSelRightTimestamp = mSelLeftTimestamp;
			
			drawAllCharts();
		}

		@Override
		public void mousePressed(MouseEvent e) {}

		@Override
		public void mouseReleased(MouseEvent e) {}

		@Override
		public void mouseEntered(MouseEvent e) {}

		@Override
		public void mouseExited(MouseEvent e) {}
	}
	
	final JFileChooser fc = new JFileChooser()
	{
        private static final long serialVersionUID = 7919427933588163126L;

        public void approveSelection()
        {
            File f = getSelectedFile();
            if (f.exists() && getDialogType() == SAVE_DIALOG)
            {
                int result = JOptionPane.showConfirmDialog(this,
                        "The file exists, overwrite?", "Existing file",
                        JOptionPane.YES_NO_CANCEL_OPTION);
                switch (result)
                {
                case JOptionPane.YES_OPTION:
                    super.approveSelection();
                    return;
                case JOptionPane.CANCEL_OPTION:
                    cancelSelection();
                    return;
                default:
                    return;
                }
            }
            super.approveSelection();
        }
    };

	public ChartsGui()
	{
		super("Charts",960,540);
		this.setTitle(ResBox.text("CHARTS"));
		this.setIconImage(ResBox.icon("CHARTS").getImage());
		createUI();
	}
	
	private void createUI()
	{
		this.setLayout(new MigLayout("","[grow][200px]","[grow]"));
		this.addComponentListener(new OnFrameResize());
		this.addWindowListener(new OnWindowListener());
		
		//---------------------------------------------------------------------
		// Period panel
		
		rbSnapshot = new JRadioButton(ResBox.text("SNAPSHOT"));
		rbRealtime = new JRadioButton(ResBox.text("REALTIME"));
		
		ButtonGroup bgPeriod = new ButtonGroup();
		bgPeriod.add(rbSnapshot);
		bgPeriod.add(rbRealtime);
		
		rbSnapshot.setSelected(true);
		
		JButton btnSubmit = new JButton(ResBox.text("SUBMIT"));
		btnSubmit.addActionListener(new OnBtnSubmit());
		
		JPanel pnlPeriod = new JPanel(new MigLayout("","[grow]",""));
		pnlPeriod.setBorder(new TitledBorder(ResBox.text("PERIOD")));
		
		pnlPeriod.add(rbRealtime,"grow,wrap");
		pnlPeriod.add(rbSnapshot,"grow,wrap");
		pnlPeriod.add(btnSubmit,"grow");
		
		//---------------------------------------------------------------------
		// Source panel
		
		//List<CheckListItem> items = buildCheckListItems(DroneState.class, "");
		List<CheckListItem> items = new ArrayList<CheckListItem>();
		try
		{
			items = buildCheckListItems(Profile.instance().getDroneSettings(), "");
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
		// Add special item for rendering track of drone
		items.add(new CheckListItem(TRACK_LIST_ITEM_NAME));
		JList<CheckListItem> lstSource = new JList<CheckListItem>(items.toArray(new CheckListItem[0]));
		
		lstSource.setCellRenderer(new CheckListRenderer());
	    lstSource.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
	    lstSource.addMouseListener(new OnCheckListMouseClick());
	    
	    btnSaveImage = new JButton(ResBox.text("SAVE_IMAGE"));
	    btnSaveImage.addActionListener(new OnBtnSaveImage());
	    btnExportSelRange = new JButton(ResBox.text("EXPORT_SEL_RANGE"));
	    btnExportSelRange.addActionListener(new OnBtnExportSelRange());
	    btnExportFullRange = new JButton(ResBox.text("EXPORT_FULL_RANGE"));
	    btnExportFullRange.addActionListener(new OnBtnExportFullRange());
	    btnClearBlackBox = new JButton(ResBox.text("CLEAR_BLACK_BOX"));
	    btnClearBlackBox.addActionListener(new OnBtnClearBlackBox());
	    
	    btnSaveImage.setEnabled(false);
		btnExportSelRange.setEnabled(false);
		btnExportFullRange.setEnabled(false);
	    
	    JPanel pnlData = new JPanel(new MigLayout("","[grow]","[grow][][]"));
	    pnlData.setBorder(new TitledBorder(ResBox.text("DATA")));
	    pnlData.add(new JScrollPane(lstSource),"grow,wrap");
	    pnlData.add(btnSaveImage,"grow,wrap");
	    pnlData.add(btnExportSelRange,"grow,wrap");
	    pnlData.add(btnExportFullRange,"grow,wrap");
	    pnlData.add(btnClearBlackBox,"grow");

	    JPanel pnlSource = new JPanel(new MigLayout("insets 0 0 0 0","[grow]","[][grow]"));
	    
	    pnlSource.add(pnlPeriod,"grow,wrap");
	    pnlSource.add(pnlData,"grow");
	    
	    mDataChartViewer = new ChartViewer();
	    // layout gets broken if i set borders
	    //mDataChartViewer.setBorder(new TitledBorder(Text.get("DATA")));
	    mDataChartViewer.setMouseUsage(Chart.MouseUsageScrollOnDrag);
	    mDataChartViewer.setHotSpotCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
	    mDataChartViewer.setScrollDirection(Chart.DirectionHorizontalVertical);
	    mDataChartViewer.setZoomDirection(Chart.DirectionHorizontalVertical);
	    mDataChartViewer.setMouseWheelZoomRatio(1.1);
	    mDataChartViewer.addViewPortListener(new OnChartViewportChanged());
	    mDataChartViewer.setMinimumSize(new java.awt.Dimension(200,200));
	    mChartMouseListener = new ChartMouseListener();
	    
	    mData = new DroneState[0];
	    
		this.add(mDataChartViewer,"grow");
		this.add(pnlSource,"grow");
	}

	/*
	private List<CheckListItem> buildCheckListItems(Class<?> klass, String baseName)
	{
		List<CheckListItem> items = new ArrayList<CheckListItem>();
		
		Field fields[] = klass.getFields();
		for(int i = 0; i < fields.length; i++)
		{
			if(fields[i].getType().getFields().length > 0)
			{
				items.addAll(buildCheckListItems(fields[i].getType(), fields[i].getName() + "."));
			}
			else if( fields[i].getAnnotation(DroneState.NoChart.class) == null && 
					 java.lang.reflect.Modifier.isStatic(fields[i].getModifiers()) == false)
			{
				
				CheckListItem j = new CheckListItem(baseName + fields[i].getName());
				items.add(j);
			}
		}

		return items;
	}
	*/
	
	private List<CheckListItem> buildCheckListItems(Object obj, String baseName) throws IllegalArgumentException, IllegalAccessException
	{
		Class<?> klass = obj.getClass();
		List<CheckListItem> items = new ArrayList<CheckListItem>();
		
		Field fields[] = klass.getFields();
		for(int i = 0; i < fields.length; i++)
		{
			if( fields[i].getAnnotation(DroneState.NoChart.class) != null || 
				java.lang.reflect.Modifier.isStatic(fields[i].getModifiers()) == true)
			{
				continue;
			}
				
			if(fields[i].getType().getFields().length > 0)
			{
				items.addAll(buildCheckListItems(fields[i].get(obj), fields[i].getName() + "."));
			}
			else
			{
				if(fields[i].getType().isArray())
				{
					Object array = fields[i].get(obj);
					int length = Array.getLength(array);
					for(int idx = 0; idx < length; idx++)
					{
						CheckListItem j = new CheckListItem(baseName + fields[i].getName() + "[" + idx + "]");
						items.add(j);	
					}
				}
				else
				{
					CheckListItem j = new CheckListItem(baseName + fields[i].getName());
					items.add(j);
				}
			}
		}

		return items;
	}
	
	private synchronized void drawAllCharts()
	{
		if(mData == null)
			return;
		
		double xMinValue = 0;
		double xMaxValue = 0;
		double yMinValue = 0;
		double yMaxValue = 0;
		
		int w = mDataChartViewer.getWidth();
		int h = mDataChartViewer.getHeight();
		
		if(w < 200)
			w = 200;
		if(h < 200)
			h = 200;
		
		XYChart c = new XYChart(w, h);
		c.setPlotArea(60, 45, w - 90, h - 90, -1, -1, 0xc0c0c0, 0xc0c0c0, -1);
		c.recycle(mDataChartViewer.getChart());
        c.setClipping();
		//c.addTitle(Text.get("DATA"));
		c.addLegend(60, 5, false);
	    c.xAxis().setTitle(ResBox.text("TIME_AXIS"));
	    c.xAxis().setMargin(0,0);    
	    c.yAxis().setTitle("");
	    c.yAxis().setMargin(0,0);
	    
	    if(mData.length == 0)
	    {
	    	mDataChartViewer.setChart(c);
	    	return;
	    }
	    
	    double timestamp[] = new double[mData.length];
	    
	    double drawTrackMinEast = 0;
	    double drawTrackMaxEast = 0;

	    for(int i = 0; i < mData.length; i++)
	    {
	    	if(mDrawTrack)
	    	{
	    		timestamp[i] = mData[i].posEast;
	    		
	    		if(mData[i].posEast < drawTrackMinEast)
	    			drawTrackMinEast = mData[i].posEast;
	    		if(mData[i].posEast > drawTrackMaxEast)
	    			drawTrackMaxEast = mData[i].posEast;
	    	}
	    	else
	    	{
	    		timestamp[i] = mData[i].timestamp;
	    	}
	    }
	    
	    if(mDrawTrack)
	    {
	    	xMinValue = drawTrackMinEast - 0.5;
	    	xMaxValue = drawTrackMaxEast + 0.5;
	    	mDataChartViewer.setFullRange("x", xMinValue, xMaxValue);
	    }
	    else if(rbSnapshot.isSelected())
	    {
	    	xMinValue = mData[0].timestamp;
	    	xMaxValue = mData[mData.length - 1].timestamp;
	    	mDataChartViewer.setFullRange("x", xMinValue, xMaxValue);
	    }

		double timestampStart = mDataChartViewer.getValueAtViewPort("x", mDataChartViewer.getViewPortLeft());
		double timestampEnd = mDataChartViewer.getValueAtViewPort("x", mDataChartViewer.getViewPortLeft() +
	            mDataChartViewer.getViewPortWidth());

		int startIndex = (int)Math.floor(Chart.bSearch(timestamp, timestampStart));
        int endIndex = (int)Math.ceil(Chart.bSearch(timestamp, timestampEnd));
        int noOfPoints = endIndex - startIndex + 1;
        double[] viewportTimestamp = (double[])Chart.arraySlice(timestamp, startIndex, noOfPoints);
        if(viewportTimestamp.length < 2)
        	return;
        timestampStart = viewportTimestamp[0];
        timestampEnd = viewportTimestamp[viewportTimestamp.length - 1];
        double minDataValue = 0;
        double maxDataValue = 0;
        boolean initMaxMinDataValue = true;
		
		for(String srcName : mSelSources)
		{
			if(mDrawTrack == true && srcName.compareToIgnoreCase(TRACK_LIST_ITEM_NAME) != 0)
				continue;
			
			if(mDrawTrack == false && srcName.compareToIgnoreCase(TRACK_LIST_ITEM_NAME) == 0)
				continue;
			
			try
			{
				String fieldName[] = srcName.split("[.]");
				double data[] = new double[noOfPoints];
				double trackTargetX[] = null;
				double trackTargetY[] = null;
				if(mDrawTrack)
				{
					trackTargetX = new double[noOfPoints];
					trackTargetY = new double[noOfPoints];
				}
			
				for(int i = startIndex, k = 0; i <= endIndex; i++, k++)
				{
					Object value = mData[i];
					
					if(mDrawTrack)
					{
						value = mData[i].posNorth;
						trackTargetX[k] = mData[i].posNorthPid.target;
						trackTargetY[k] = mData[i].posEastPid.target;
					}
					else
					{
						for(int j = 0; j < fieldName.length; j++)
						{
							if(fieldName[j].contains("["))
							{
								String arrayName[] = fieldName[j].split("[\\[\\]]");
								value = value.getClass().getField(arrayName[0]).get(value);
								value = Array.get(value,Integer.parseInt(arrayName[1]));
							}
							else
							{
								value = value.getClass().getField(fieldName[j]).get(value);
							}
						}
					}
					
					if(value instanceof Double)
						data[k] = (Double)value;
					
					// Заплатка. Иногда в массив данных попадают бесконечности. Откуда - не понятно
					// Nan тоже на всякий случай фильтруем
					if(Double.isInfinite(data[k]) ||
					   Double.isNaN(data[k]) )
					   data[k] = 0.0;
					
					if(initMaxMinDataValue)
					{
						minDataValue = data[0];
						maxDataValue = data[0];
						initMaxMinDataValue = false;
					}
					
					if(data[k] < minDataValue)
						minDataValue = data[k];
					
					if(data[k] > maxDataValue)
						maxDataValue = data[k];
				}
			
				c.addLineLayer(data, -1, srcName).setXData(viewportTimestamp);
				
				if(mDrawTrack)
				{
					c.addLineLayer(trackTargetX, -1, "target").setXData(trackTargetY);
					break;
				}
			}
			catch(Exception e)
			{
				e.printStackTrace();
			}
		}
		
		if(mDrawTrack)
		{
			yMinValue = minDataValue;
			yMaxValue = maxDataValue;
			
			// make scale of x and y to be equal
			double xFullRange = xMaxValue - xMinValue;
			double yFullRange = yMaxValue - yMinValue;
			double xLength = c.getPlotArea().getWidth();
			double yLength = c.getPlotArea().getHeight();
			
			double xScale = xFullRange / xLength;
			double yScale = yFullRange / yLength;
			
			double minAxisValue = 0;
			double maxAxisValue = 0;
			
			double tick = 0;

			if(xScale > yScale)
			{
				yFullRange = xScale * yLength;
				minAxisValue = yMinValue + (yMaxValue - yMinValue - yFullRange)/2.0;
				maxAxisValue = minAxisValue + yFullRange;
				tick = (maxAxisValue - minAxisValue)/10.0;
				c.yAxis().setLinearScale(minAxisValue,maxAxisValue,tick);
				c.xAxis().setLinearScale(xMinValue,xMaxValue,tick);
			}
			else
			{
				xFullRange = yScale * xLength;
				minAxisValue = xMinValue + (xMaxValue - xMinValue - xFullRange)/2.0;
				maxAxisValue = minAxisValue + xFullRange;
				tick = (maxAxisValue - minAxisValue)/10.0;
				c.xAxis().setLinearScale(minAxisValue,maxAxisValue,tick);
				c.yAxis().setLinearScale(yMinValue,yMaxValue,tick);
			}
		}
		else
		{
			if(mDataChartViewer.getViewPortWidth() == 1.0)
				mDataChartViewer.setFullRange("y", minDataValue, maxDataValue);
		
			double bottom = mDataChartViewer.getViewPortTop();
			double top = bottom + mDataChartViewer.getViewPortHeight();
			maxDataValue = mDataChartViewer.getValueAtViewPort("y", top);
			minDataValue = mDataChartViewer.getValueAtViewPort("y", bottom);
			double d = (maxDataValue - minDataValue)*0.05;
			maxDataValue += d;
			minDataValue -= d;
			c.yAxis().setLinearScale(minDataValue, maxDataValue, (maxDataValue - minDataValue)/10.0);
			
			if(rbRealtime.isSelected())
			{
				 timestampEnd = mDataChartViewer.getValueAtViewPort("x",
						 mDataChartViewer.getViewPortLeft() + mDataChartViewer.getViewPortWidth());
				 c.xAxis().setLinearScale(timestampStart, timestampEnd, (timestampEnd - timestampStart)/20.0);
			}
			else
			{
				// c.xAxis().setLinearScale(timestampStart, timestampEnd, 500000);
				mDataChartViewer.syncLinearAxisWithViewPort("x", c.xAxis());
			}
			
			//c.xAxis().setTickDensity(20);
			//c.yAxis().setTickDensity(20);
		}
		
		if(mSelLeftTimestamp != mSelRightTimestamp)
		{
			c.xAxis().addZone(mSelLeftTimestamp, mSelRightTimestamp, 0x800080);
		}
		
		// replace yellow color in chart palette because it looks ugly
		c.setColor(11, 0x5f0000);
		
       	mDataChartViewer.setChart(c);   
	}
	
	private void plotDataChart(String name, boolean remove)
	{
		if(remove)
			mSelSources.remove(name);
		else
			mSelSources.add(name);
		
		drawAllCharts();
	}
	
	private void exportToCsv(String filename, double timestampStart, double timestampEnd)
	{
	    double timestamp[] = new double[mData.length];
	    
	    for(int i = 0; i < mData.length; i++)
	    	timestamp[i] = mData[i].timestamp;
	    
		try(FileOutputStream fos = new FileOutputStream(filename))
		{
			String field = "timestamp";
			String comma = ",";
			String lineSeparator = System.getProperty("line.separator");
			
			int startIndex = (int)Math.floor(Chart.bSearch(timestamp, timestampStart));
	        int endIndex = (int)Math.ceil(Chart.bSearch(timestamp, timestampEnd));
			
			fos.write(field.getBytes());
			
			for(String srcName : mSelSources)
			{
				if(srcName.compareToIgnoreCase(TRACK_LIST_ITEM_NAME) == 0)
					continue;
				fos.write(comma.getBytes());
				fos.write(srcName.getBytes());
			}
			
			fos.write(lineSeparator.getBytes());
			
			for(int i = startIndex; i <= endIndex; i++)
			{
				fos.write(Double.toString(mData[i].timestamp).getBytes());
				
				for(String srcName : mSelSources)
				{
					if(srcName.compareToIgnoreCase(TRACK_LIST_ITEM_NAME) == 0)
						continue;
					
					String fieldName[] = srcName.split("[.]");
					Object value = mData[i];

					for(int j = 0; j < fieldName.length; j++)
						value = value.getClass().getField(fieldName[j]).get(value);
					
					fos.write(comma.getBytes());
					fos.write(value.toString().getBytes());
				}
				
				fos.write(lineSeparator.getBytes());
			}
        }
        catch(Exception ex)
		{
              
            System.out.println(ex.getMessage());
        }
	}

	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getChartsWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setChartsWnd(ws);
	}
}
