package main;

import java.awt.Dimension;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.util.Observable;
import java.util.Observer;

import javax.swing.JEditorPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.text.Document;

import main.AppSettings.WndState;
import net.miginfocom.swing.MigLayout;
import pdl.DroneLog;

public class LogGui extends JSavedFrame
{
	private static final long serialVersionUID = -356406639277013737L;
	private JEditorPane mViewer;

	private class OnLogUpdate implements Observer
	{
		@Override
		public void update(Observable o, Object arg)
		{
			try
			{
				Document doc = mViewer.getDocument();
				doc.insertString(doc.getLength(),arg.toString(),null);
			}
			catch(Exception exc)
			{
				exc.printStackTrace();
			}
		}
	}
	
	public LogGui()
	{
		super("Log",330,330);
		this.setTitle(ResBox.text("LOG"));
		this.setIconImage(ResBox.icon("LOG").getImage());
		this.createUI();
		DroneLog.instance().addObserver(new OnLogUpdate());
	}
	
	private void createUI()
	{
		JPanel pnlLog = new JPanel(new MigLayout("","[grow]","[grow]"));

        mViewer = new JEditorPane();
        mViewer.setContentType("text/plain");
        mViewer.setEditable(false);
        mViewer.setText(DroneLog.instance().getLog());
        
        final JScrollPane scroller = new JScrollPane(mViewer);      
        //NOTE: this is the magic that is kind of a workaround
        // you can also implement your own type of JScrollPane
        // using the JScrollBar and a JViewport which is the 
        // preferred method of doing something like this the 
        // other option is to create a JEditorPane subclass that
        // implements the Scrollable interface.
        scroller.addComponentListener(new ComponentAdapter()
        {
        	@Override
        	public void componentResized(ComponentEvent e) {
        		mViewer.setSize(new Dimension(
                scroller.getWidth()-20, 
                scroller.getHeight()-20));
        	}
        });
        
        pnlLog.add(new JScrollPane(mViewer),"grow");
		
		this.add(pnlLog);
	}
	
	public void clearLog()
	{
		mViewer.setText("");
	}

	@Override
	protected WndState loadWndState()
	{
		return AppSettings.instance().getLogWnd();
	}

	@Override
	protected void saveWndState(WndState ws)
	{
		AppSettings.instance().setLogWnd(ws);	
	}
}
