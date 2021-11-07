package helper;

import javax.swing.text.*;
import java.awt.*;
import java.text.DecimalFormatSymbols;
import java.text.NumberFormat;
import java.text.ParseException;

/** Numeric document model */
public class NumericDocument extends PlainDocument 
{
	private static final long serialVersionUID = 1077774416136985362L;
	
	protected int decimalPrecision = 0;
    protected boolean allowNegative = false;
    
    private static String comma = String.valueOf(DecimalFormatSymbols.getInstance().getDecimalSeparator());
    private static String minus = String.valueOf(DecimalFormatSymbols.getInstance().getMinusSign());

     public NumericDocument(int decimalPrecision, boolean allowNegative) 
     {
          super();
          this.decimalPrecision = decimalPrecision;
          this.allowNegative = allowNegative;
     }
     
     private boolean isNumeric(String str)
     {
    	 if(str.length() == 1 && (str.contains(comma) || str.contains(minus)))
    		 return false;
    	 
    	 try
    	 { 
    		 NumberFormat.getInstance().parse(str);
    		 //Double.parseDouble(str);
    	 }
    	 /*catch(NumberFormatException e)
    	 {
    		 e.printStackTrace();
    		 return false;
    	 }*/
    	 catch(ParseException e) 
    	 {
			//e.printStackTrace();
    		Toolkit.getDefaultToolkit().beep();
			return false;
		 }
    	 return true;
     }
   
     public void insertString(int offset, String str, AttributeSet attr) throws BadLocationException 
     {
    	 if(str == null)
    		 return;
    	 //First, is it a valid character?
    	 if( isNumeric(str) == false && 
    		 str.equals(comma) == false && 
    		 str.equals(minus) == false )
    	 {
    		 Toolkit.getDefaultToolkit().beep();
    		 return;
    	 }
    	 // Next, can we place a decimal here?
    	 String text = super.getText(0,super.getLength()); 
    	 if(str.contains(comma) == true)
    	 {
    		 if( text.contains(comma) == true ||
    			 decimalPrecision == 0 )
    		 {
    		    Toolkit.getDefaultToolkit().beep();
    		    return;
    		 }
    	 }
    	 // Next, do we get past the decimal precision limit?
    	 int commaPos = text.indexOf(comma); 
    	 if( commaPos != -1 &&
    		 offset > commaPos &&
    		 super.getLength() - commaPos > decimalPrecision)
    	 {
    		 Toolkit.getDefaultToolkit().beep();
    		 return;
    	 }
    	 // Next, can we put a negative sign?
    	 if(str.equals(minus) == true)
    	 if( text.contains(minus) == true ||
    		 offset != 0 ||
    		 allowNegative == false )
    	 {
    		 Toolkit.getDefaultToolkit().beep();
    		 return;
    	 }
    	 // All is fine, so add the character to the text box
    	 super.insertString(offset, str, attr);
     }
}
