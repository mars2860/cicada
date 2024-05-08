package ru.liftelectronica.cicada;

import android.content.Context;
import android.content.DialogInterface;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ExpandableListView;
import android.widget.Spinner;
import android.widget.Toast;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.ArrayList;

import pdl.Alarm;
import pdl.DroneAlarmCenter;
import pdl.res.Profile;
import pdl.res.SettingsNode;
import pdl.DroneCommander;
import pdl.DroneState;
import pdl.DroneTelemetry;
import pdl.commands.CmdResetAltitude;
import pdl.res.TextBox;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.app.AlertDialog;
public class SettingsActivity extends AppCompatActivity {

    private static final String CUR_PROFILE_FILENAME = "profile.txt";
    private static final String PROFILE_EXT = ".json";

    private SettingsAdapter settingsAdapter;
    Spinner spProfile;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        int profileSelPos = 0;
        String curProfile = SettingsActivity.loadCurProfileName(this.getApplicationContext());

        String profile[] = this.getApplicationContext().getFilesDir().list(new FilenameFilter() {
            @Override
            public boolean accept(File dir, String filename) {
                return filename.toLowerCase().endsWith(PROFILE_EXT);
            }
        });

        ArrayList<String> profilesList = new ArrayList<String>();

        for(int i = 0; i < profile.length; i++) { // remove .json from profile name
            profile[i] = profile[i].split("[.]")[0];
            if(profile[i].compareTo(curProfile) == 0)
                profileSelPos = i;
            profilesList.add(profile[i]);
        }

        final ArrayAdapter<String> profileAdapter = new ArrayAdapter<String>(
                this,android.R.layout.simple_spinner_item,profilesList);

        // Specify the layout to use when the list of choices appears
        profileAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);

        spProfile = (Spinner)findViewById(R.id.spProfile);
        // Apply the adapter to the spinner
        spProfile.setAdapter(profileAdapter);
        spProfile.setSelection(profileSelPos);

        spProfile.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                String profile = profileAdapter.getItem(position);
                File file = new File(SettingsActivity.this.getFilesDir(), profile + PROFILE_EXT);
                if (Profile.load(file) == true) {
                    updateSettingsView();
                    SettingsActivity.saveCurProfileName(SettingsActivity.this, profile);
                    Toast.makeText(SettingsActivity.this, R.string.loaded_successfully, Toast.LENGTH_LONG).show();
                } else {
                    Toast.makeText(SettingsActivity.this, R.string.loading_failed, Toast.LENGTH_LONG).show();
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        // Create settings
        updateSettingsView();
    }

    @Override
    protected void onStart() {
        super.onStart();
        try
        {
            // Restart drone services because it can be closed when we close MainActivity
            DroneCommander.instance().start(DroneState.net.ip, DroneState.net.cmdPort);
            DroneTelemetry.instance().start(DroneState.net.ip, DroneState.net.telemetryPort);
        }
        catch(UnknownHostException e)
        {
            Toast.makeText(this,TextBox.get("INVALID_HOST"),Toast.LENGTH_LONG).show();
        }
        catch(SocketException e) {
            Toast.makeText(this,TextBox.get("SOCKET_NOT_OPEN"),Toast.LENGTH_LONG).show();
        }
    }

    public void updateSettingsView() {
        SettingsNode settings = Profile.instance().buildSettingsTree();
        settingsAdapter = new SettingsAdapter(settings,this);
        ExpandableListView lvSettings = (ExpandableListView)findViewById(R.id.lvSettings);
        lvSettings.setAdapter(settingsAdapter);
    }

    public void onBtnSend(View v) {
        if(settingsAdapter == null)
            return;

        if(DroneTelemetry.instance().isDroneConnected() == false) {
            Toast.makeText(this, TextBox.get("ALARM_DRONE_NOT_FOUND"), Toast.LENGTH_SHORT).show();
            return;
        }

        DroneState ds = settingsAdapter.grabNewSettings();

        if(ds != null) {
            Profile.instance().setDroneSettings(ds);

            DroneTelemetry.instance().resetFlyTime();
            DroneCommander.instance().sendSettingsToDrone(ds);
            CmdResetAltitude cmd = new CmdResetAltitude();
            // we can lose udp packet so send it three times
            DroneCommander.instance().addCmd(cmd);
            DroneCommander.instance().addCmd(cmd);
            DroneCommander.instance().addCmd(cmd);

            if(DroneAlarmCenter.instance().getAlarm(Alarm.ALARM_SEND_ERROR)) {
                Toast.makeText(this,TextBox.get("ALARM_SEND_ERROR"), Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(this, R.string.settings_sent, Toast.LENGTH_SHORT).show();
            }
        }
    }

    public void onBtnSaveAs(View v) {
        LayoutInflater inflater = LayoutInflater.from(SettingsActivity.this);
        View view = inflater.inflate(R.layout.edit_profile_name, null);
        AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(SettingsActivity.this);
        alertDialogBuilder.setView(view);

        final EditText etName = (EditText) view.findViewById(R.id.etProfileName);
        etName.setText(spProfile.getSelectedItem().toString());

        alertDialogBuilder.setCancelable(false)
                .setPositiveButton(R.string.ok,
                        new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int id) {
                                if (settingsAdapter == null)
                                    return;

                                String profile = etName.getText().toString();

                                if (profile.length() == 0) {
                                    Toast.makeText(SettingsActivity.this, R.string.invalid_name, Toast.LENGTH_LONG).show();
                                    return;
                                }

                                DroneState ds = settingsAdapter.grabNewSettings();

                                if (ds != null) {
                                    Profile.instance().setDroneSettings(ds);
                                    File file = new File(SettingsActivity.this.getFilesDir(), profile + PROFILE_EXT);
                                    if (Profile.instance().save(file) == true) {

                                        ArrayAdapter<String> profileAdapter =
                                                (ArrayAdapter<String>) spProfile.getAdapter();
                                        int pos = profileAdapter.getPosition(profile);
                                        if (pos < 0) {
                                            profileAdapter.add(profile);
                                            pos = spProfile.getCount() - 1;
                                        }
                                        spProfile.setSelection(pos);
                                        Toast.makeText(SettingsActivity.this, R.string.saved_successfully, Toast.LENGTH_LONG).show();
                                    } else {
                                        Toast.makeText(SettingsActivity.this, R.string.saving_failed, Toast.LENGTH_LONG).show();
                                    }
                                }
                            }
                        })
                .setNegativeButton(R.string.cancel,
                        new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int id) {
                                dialog.cancel();
                            }
                        });
        AlertDialog alertDialog = alertDialogBuilder.create();
        alertDialog.show();
    }

    public void onBtnRemove(View v) {
        if(spProfile.getCount() <= 1)
            return;

        new AlertDialog.Builder(this)
                .setIcon(android.R.drawable.ic_dialog_alert)
                .setTitle(R.string.confirm)
                .setMessage(R.string.are_you_sure)
                .setPositiveButton(R.string.yes, new DialogInterface.OnClickListener()
                {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        String profile = spProfile.getSelectedItem().toString();

                        ArrayAdapter<String> profileAdapter =
                            (ArrayAdapter<String>) spProfile.getAdapter();

                        SettingsActivity.this.getApplicationContext().deleteFile(profile + PROFILE_EXT);
                        profileAdapter.remove(profile);

                        spProfile.setSelection(0);
                    }
                })
                .setNegativeButton(R.string.no, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        dialog.cancel();
                    }
                })
                .show();
    }

    public void onBtnLoadDefault(View v) {
        if(DroneTelemetry.instance().isDroneConnected() == false)
            return;

        DroneCommander.instance().loadDefaultCfg();

        v.postDelayed(new Runnable() {
            @Override
            public void run() {
                DroneState ds = DroneTelemetry.instance().getDroneState();
                Profile.instance().setDroneSettings(ds);
                updateSettingsView();
                Toast.makeText(SettingsActivity.this, R.string.loaded_successfully, Toast.LENGTH_LONG).show();
            }
        }, 500);
    }

    public static String loadCurProfileName(Context context) {
        String profile = "";
        File file = new File(context.getFilesDir(),CUR_PROFILE_FILENAME);
        FileInputStream fis = null;
        int n;
        try {
            byte buffer[] = new byte[256];
            fis = new FileInputStream(file);
            n = fis.read(buffer);
            if(n > 0) {
                profile = new String(buffer, 0, n);
            }
        }
        catch(Exception e) {
            Log.e("", "Can't read " + CUR_PROFILE_FILENAME);
        }
        finally {
            try {
                if(fis != null)
                    fis.close();
            }
            catch(Exception e) {
                Log.e("", "Can't close " + CUR_PROFILE_FILENAME);
            }
        }
        return profile;
    }

    public static void saveCurProfileName(Context context, String profile) {
        File file = new File(context.getFilesDir(),CUR_PROFILE_FILENAME);
        FileOutputStream fos = null;
        try {
            fos = new FileOutputStream(file);
            fos.write(profile.getBytes());
        } catch(Exception e) {
            Log.e("", "Can't save " + CUR_PROFILE_FILENAME);
        }
        finally {
            try {
                if(fos != null)
                    fos.close();
            } catch (Exception e) {
                Log.e("", "Can't close " + CUR_PROFILE_FILENAME);
            }
        }
    }
}
