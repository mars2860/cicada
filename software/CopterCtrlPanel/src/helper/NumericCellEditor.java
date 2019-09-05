package helper;

import java.awt.Component;

import javax.swing.AbstractCellEditor;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.border.EmptyBorder;
import javax.swing.table.TableCellEditor;

public class NumericCellEditor extends AbstractCellEditor implements TableCellEditor
{
	private static final long serialVersionUID = -7465863293481741384L;
	
	private JTextField mEditor;
	
	/// Конструктор
	/**	@param[in] dec Количество знаков после запятой
	 * 	@param[in] allowNegative Разрешение на ввод отрицательных чисел
	 * */
	public NumericCellEditor(int dec, boolean allowNegative)
	{
		mEditor = new JTextField();
		mEditor.setHorizontalAlignment(JTextField.RIGHT);
		mEditor.setBorder(new EmptyBorder(0, 0, 0, 0));
		mEditor.setDocument(new NumericDocument(dec, allowNegative));
	}
	
	@Override
	public Object getCellEditorValue() 
	{
		return mEditor.getText();
	}

	@Override
	public Component getTableCellEditorComponent(	JTable table,
													Object value,
													boolean isSelected,
													int row,
													int col)
	{
		if(value != null)
			mEditor.setText(value.toString());
		return mEditor;
	}
}
