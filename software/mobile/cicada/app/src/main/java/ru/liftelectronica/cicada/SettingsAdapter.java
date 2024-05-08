package ru.liftelectronica.cicada;

import android.app.Activity;
import android.content.Context;
import android.text.InputType;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.BaseExpandableListAdapter;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import pdl.res.TextBox;
import pdl.res.SettingsNode;
import pdl.DroneState;

public class SettingsAdapter extends BaseExpandableListAdapter {

    private SettingsNode data;
    private Activity activity;

    public SettingsAdapter(SettingsNode data, Activity activity) {
        this.data = data;
        this.activity = activity;
    }

    @Override
    public int getGroupCount() {
        return data.childs.size();
    }

    @Override
    public int getChildrenCount(int groupPosition) {
        return data.childs.get(groupPosition).childs.size();
    }

    @Override
    public Object getGroup(int groupPosition) {
        return data.childs.get(groupPosition);
    }

    @Override
    public Object getChild(int groupPosition, int childPosition) {
        return data.childs.get(groupPosition).childs.get(childPosition);
    }

    @Override
    public long getGroupId(int groupPosition) {
        return groupPosition;
    }

    @Override
    public long getChildId(int groupPosition, int childPosition) {
        return childPosition;
    }

    @Override
    public boolean hasStableIds() {
        return false;
    }

    @Override
    public View getGroupView(int groupPosition, boolean isExpanded, View convertView, ViewGroup parent) {
        SettingsNode group = (SettingsNode) getGroup(groupPosition);

        if (convertView == null) {
            LayoutInflater layoutInflater = (LayoutInflater)activity.getApplicationContext().
                    getSystemService(Context.LAYOUT_INFLATER_SERVICE);
            convertView = layoutInflater.inflate(R.layout.settings_group, null);
        }
        TextView tvSettingsGroup = (TextView)convertView.findViewById(R.id.tvSettingsGroup);
        tvSettingsGroup.setText(group.name);
        return convertView;
    }

    @Override
    public View getChildView(int groupPosition, int childPosition, boolean isLastChild, View convertView, ViewGroup parent) {
        SettingsNode param = (SettingsNode)getChild(groupPosition,childPosition);

        int type = 0;
        int choicePos = 0;
        List<?> choices = new ArrayList<Object>();  // to prevent warn - "could be not init"
        ArrayAdapter<?> spAdapter = new ArrayAdapter<Object>( // to prevent warn - "could be not init"
                activity,
                android.R.layout.simple_spinner_item,
                (List<Object>)choices);

        int res = R.layout.settings_item_num;

        if(param.value instanceof Boolean) {
            type = 1;
            res = R.layout.settings_item_bool;
        } else if(param.value instanceof DroneState.RemoteCtrl.RotateBy) {
            type = 2;
            res = R.layout.settings_item_choice;

            choices = new ArrayList<DroneState.RemoteCtrl.RotateBy>(Arrays.asList(DroneState.RemoteCtrl.RotateBy.values()));

            // don't know how to implement this control type in mobile app
            choices.remove(DroneState.RemoteCtrl.RotateBy.ROTATE_BY_HEADING);

            spAdapter = new ArrayAdapter<DroneState.RemoteCtrl.RotateBy>(
                    activity,
                    android.R.layout.simple_spinner_item,
                    (List<DroneState.RemoteCtrl.RotateBy>)choices);

        } else if(param.value instanceof DroneState.RemoteCtrl.MoveBy) {
            type = 2;
            res = R.layout.settings_item_choice;

            choices = new ArrayList<DroneState.RemoteCtrl.MoveBy>(Arrays.asList(DroneState.RemoteCtrl.MoveBy.values()));

            spAdapter = new ArrayAdapter<DroneState.RemoteCtrl.MoveBy>(
                    activity,
                    android.R.layout.simple_spinner_item,
                    (List<DroneState.RemoteCtrl.MoveBy>)choices);
        } else if(param.value instanceof DroneState.RemoteCtrl.LiftBy) {
            type = 2;
            res = R.layout.settings_item_choice;

            choices = new ArrayList<DroneState.RemoteCtrl.LiftBy>(Arrays.asList(DroneState.RemoteCtrl.LiftBy.values()));

            // don't know how to implement this control type in mobile app
            choices.remove(DroneState.RemoteCtrl.LiftBy.LIFT_BY_ALT);
            choices.remove(DroneState.RemoteCtrl.LiftBy.LIFT_BY_GAS_DELTA);
            choices.remove(DroneState.RemoteCtrl.LiftBy.LIFT_BY_GAS_FORSAGE);

            spAdapter = new ArrayAdapter<DroneState.RemoteCtrl.LiftBy>(
                    activity,
                    android.R.layout.simple_spinner_item,
                    (List<DroneState.RemoteCtrl.LiftBy>)choices);
        }

        convertView = (View)param.userData;

        if (convertView == null) {
            LayoutInflater layoutInflater = (LayoutInflater)activity.getApplicationContext()
                    .getSystemService(Context.LAYOUT_INFLATER_SERVICE);
            convertView = layoutInflater.inflate(res, null);
            param.userData = convertView;

            TextView tvSettingName = (TextView)convertView.findViewById(R.id.tvSettingName);
            tvSettingName.setText(param.name);

            try {
                switch (type) {
                    case 0:
                        EditText vv = (EditText) convertView.findViewById(R.id.paramValue);
                        vv.setText(param.value.toString());

                        if(param.value instanceof String) {
                            vv.setInputType(InputType.TYPE_TEXT_FLAG_NO_SUGGESTIONS);
                        }

                        /*
                        if(param.name.compareToIgnoreCase("ip") == 0) {
                            vv.setInputType(InputType.TYPE_TEXT_VARIATION_URI);
                        } else if(param.name.compareToIgnoreCase("gateway") == 0) {
                            vv.setInputType(InputType.TYPE_TEXT_VARIATION_URI);
                        } else if(param.name.compareToIgnoreCase("subnet") == 0) {
                            vv.setInputType(InputType.TYPE_TEXT_VARIATION_URI);
                        } else if(param.name.compareToIgnoreCase("ssid") == 0) {
                            vv.setInputType(InputType.TYPE_TEXT_FLAG_NO_SUGGESTIONS);
                        } else if(param.name.compareToIgnoreCase("psk") == 0) {
                            vv.setInputType(InputType.TYPE_TEXT_FLAG_NO_SUGGESTIONS);
                        }*/
                        break;
                    case 1:
                        CheckBox cc = (CheckBox) convertView.findViewById(R.id.paramValue);
                        cc.setChecked((Boolean) param.value);
                        break;
                    case 2:
                        for (choicePos = 0; choicePos < choices.size(); choicePos++) {
                            if (param.value.equals(choices.get(choicePos)))
                                break;
                        }

                        spAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);

                        Spinner ss = (Spinner) convertView.findViewById(R.id.paramValue);
                        ss.setAdapter(spAdapter);
                        ss.setSelection(choicePos);
                        break;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        return convertView;
    }

    @Override
    public boolean isChildSelectable(int groupPosition, int childPosition) {
        return true;
    }

    public DroneState grabNewSettings() {
        DroneState ds = new DroneState();

        for(SettingsNode group : data.childs)
        {
            for(SettingsNode param : group.childs)
            {
                try
                {
                    Object settingGroupObj = param.settingGroup.get(ds);
                    String type = param.settingField.getGenericType().toString();

                    Object value = param.value;

                    View v = (View)param.userData;

                    if(v != null) {
                        View valueView = v.findViewById(R.id.paramValue);

                        if (valueView instanceof EditText) {
                            EditText tt = (EditText) valueView;
                            value = tt.getText().toString();
                        } else if (valueView instanceof CheckBox) {
                            CheckBox cc = (CheckBox) valueView;
                            value = cc.isChecked();
                        } else if (valueView instanceof Spinner) {
                            Spinner ss = (Spinner) valueView;
                            value = ss.getSelectedItem();
                        }
                    }

                    if(type.startsWith("int"))
                    {
                        int i = Integer.parseInt(value.toString());
                        param.settingField.setInt(settingGroupObj, i);
                    }
                    else if(type.startsWith("double"))
                    {
                        double d = Double.parseDouble(value.toString());
                        param.settingField.setDouble(settingGroupObj, d);
                    }
                    else if(type.startsWith("float"))
                    {
                        float f = Float.parseFloat(value.toString());
                        param.settingField.setFloat(settingGroupObj, f);
                    }
                    else if(type.startsWith("bool"))
                    {
                        boolean b = Boolean.parseBoolean(value.toString());
                        param.settingField.setBoolean(settingGroupObj, b);
                    }
                    else if(type.contains("String"))
                    {
                        param.settingField.set(settingGroupObj, value.toString());
                    }
                    else
                    {
                        param.settingField.set(settingGroupObj, value);
                    }
                }
                catch(Exception e)
                {
                    String text = TextBox.get("INVALID_VALUE") + " " + group.name + "." + param.name;
                    Toast.makeText(activity,text,Toast.LENGTH_LONG).show();
                    return null;
                }
            }
        }

        return ds;
    }
}
