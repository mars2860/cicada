package pdl.res;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.annotations.Expose;

import pdl.DroneState;

public class Profile
{
	public static final String EXT = ".json";
	
	private static Profile mSingleton;
	
	public static Profile instance()
	{
		return mSingleton;
	}

	@Expose
	private DroneState droneSettings;
	
	private File mFile;

	private Profile()
	{
		setDefault();
	}
	
	public void setDroneSettings(DroneState ds)
	{
		droneSettings = ds;
	}
	
	public DroneState getDroneSettings()
	{
		DroneState ds = new DroneState();
		
		try
		{
			ds = (DroneState)droneSettings.clone(); 
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
		
		return ds; 
	}
	
	public String getName()
	{
		if(mFile == null)
			return "";
		
		return mFile.getName().split("[.]")[0];
	}
	
	public boolean save()
	{
		if(mFile == null)
			return false;
		
		return this.save(mFile);
	}

	public boolean save(File file)
	{
        boolean result = true;
		Gson gson = new GsonBuilder()
				 .excludeFieldsWithModifiers(java.lang.reflect.Modifier.TRANSIENT)
				 .excludeFieldsWithoutExposeAnnotation().create();
		
		String json = gson.toJson(this);

		FileOutputStream fos = null;

		try
		{
			fos = new FileOutputStream(file);
			fos.write(json.getBytes());
			mFile = file;
		}
		catch(Exception e)
		{
			e.printStackTrace();
			result = false;
		}
		finally
		{
			try
			{
				if (fos != null)
					fos.close();
			}
			catch(Exception e)
			{
				e.printStackTrace();
				result = false;
			}
		}

		return result;
	}
	
	public void setDefault()
	{
		droneSettings = new DroneState();
	}
	
	public static boolean load(File file)
	{
		StringBuffer fileContent = new StringBuffer("");
		int n;
        boolean result = true;
		FileInputStream fis = null;

		try
		{
			fis = new FileInputStream(file);

			byte[] buffer = new byte[1024];

			while ((n = fis.read(buffer)) != -1)
			{
				fileContent.append(new String(buffer, 0, n));
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
			result = false;
		}
		finally
		{
			try
			{
				if(fis != null)
					fis.close();
			}
			catch(Exception e)
			{
				e.printStackTrace();
				result = false;
			}
		}

		try
		{
			Gson gson = new GsonBuilder()
					 .excludeFieldsWithModifiers(java.lang.reflect.Modifier.TRANSIENT)
					 .excludeFieldsWithoutExposeAnnotation().create();

			mSingleton = gson.fromJson(fileContent.toString(), Profile.class);
			
			// заплатка: после загрузки количество моторов не меняется и остаётся по умолчанию
			// так как объект DroneState создаётся раньше, чем парсер запишет в motors.count кол-во моторов
			// поэтому после загрузки пересоздаём массив моторов
			mSingleton.droneSettings.motorGas = new double[DroneState.Motors.count];
			
			mSingleton.mFile = file;
		}
		catch(Exception e)
		{
			e.printStackTrace();
			result = false;
		}

		return result;
	}

    /** Creates settings tree data structure for gui widgets
     *  If you want to add new settings parameter to GUI
     *  just add it to DroneState class and it will be
     *  added automatically to settings gui by this method
     */
	public SettingsNode buildSettingsTree()
	{
        DroneState ds = droneSettings;

		SettingsNode root = new SettingsNode(TextBox.get("SETTINGS"));

		// Build settings tree by settings class
		// Each nested class of Settings is Group of Params
		// Each member of that class is Param

		for(java.lang.reflect.Field settingGroupField : DroneState.class.getFields())
		{
			DroneState.SettingGroup settingGroupAnnotation = settingGroupField.getAnnotation(DroneState.SettingGroup.class);

			if(settingGroupAnnotation != null)
			{
				SettingsNode groupNode = new SettingsNode(TextBox.get(settingGroupAnnotation.name()));
				
				try
				{
					groupNode.value = settingGroupField.get(ds);
				}
				catch(Exception e)
				{
					e.printStackTrace();
				}

				for(java.lang.reflect.Field settingField : settingGroupField.getType().getFields())
				{
					if(settingField.isAnnotationPresent(DroneState.Setting.class) == false)
						continue;

					SettingsNode paramNode = new SettingsNode(settingField.getName());

					try
					{
						paramNode.value = settingField.get(settingGroupField.get(ds));
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
}
