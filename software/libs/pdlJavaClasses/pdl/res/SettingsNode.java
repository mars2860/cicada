package pdl.res;

import java.util.ArrayList;
import java.util.List;

public class SettingsNode {
    public String name = new String();
    public Object value = new String();
    public Object userData;

    // Relationship between Node and DroneState
    public java.lang.reflect.Field settingField;
    public java.lang.reflect.Field settingGroup;
    public List<SettingsNode> childs = new ArrayList<SettingsNode>();

    public SettingsNode() {}

    public SettingsNode(String name)
    {
        SettingsNode.this.name = name;
    }

    @Override
    public String toString()
    {
        return name;
    }
}
