package helper;

import java.util.ArrayList;
import java.util.List;

public class ArrayHelper
{
	public static List<Double> asList(double[] a, int length)
	{
	    List<Double> list = new ArrayList<Double>();
	    for(int i = 0; i < length && list.add(a[i]); i++);
	    return list;
	}
}
