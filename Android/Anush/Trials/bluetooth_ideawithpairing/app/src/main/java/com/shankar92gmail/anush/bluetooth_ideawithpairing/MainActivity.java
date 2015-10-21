package com.shankar92gmail.anush.bluetooth_ideawithpairing;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.Toast;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends Activity implements AdapterView.OnItemClickListener {

    ArrayAdapter<String> listAdapter;
    ArrayList<String> pairedDevices;
    public static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    ListView listview;
    BluetoothAdapter btAdapter;
    Set<BluetoothDevice> devicesArray;
    IntentFilter filter;
    BroadcastReceiver receiver;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        init();
        if(btAdapter == null) {
            Toast.makeText(getApplicationContext(),"NO BLUETOOTH SUPPORT ON DEVICE",Toast.LENGTH_LONG).show();
            finish();
        }
        else {
            if (!btAdapter.isEnabled()) {
                turnonBT();
            }
            getPairedDevices();
            startDisc();
        }
    }

    private void startDisc() {
        btAdapter.cancelDiscovery();
        btAdapter.startDiscovery();
    }

    private void turnonBT() {
        Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
        startActivityForResult(intent, 1);
    }

    private void getPairedDevices() {
        devicesArray = btAdapter.getBondedDevices();
        if(devicesArray.size()>0)
        {
            for(BluetoothDevice device:devicesArray)
            {
                pairedDevices.add(device.getName());
            }
        }
    }

    private void init() {
        listview = (ListView)findViewById(R.id.listView12);
        listview.setOnItemClickListener(this);
        listAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1,0);
        listview.setAdapter(listAdapter);
        btAdapter = BluetoothAdapter.getDefaultAdapter();
        pairedDevices = new ArrayList<String>();
        filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
        receiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                String action = intent.getAction();

                if(BluetoothDevice.ACTION_FOUND.equals(action))
                {
                    BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);

                    String s = " ";
                            for(int a=0;a<pairedDevices.size();a++)
                            {
                                if(device.getName().equals(pairedDevices.get(a)))
                                {
                                    //append on and break out of for loop

                                    s = "(Paired)";
                                    break;
                                }
                            }
                    listAdapter.add(device.getName()+"  "+s+"  "+"\n"+device.getAddress());
                }
                else if(BluetoothAdapter.ACTION_DISCOVERY_STARTED.equals(action))
                {

                }
                else if(BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action))
                {

                }

                else if(BluetoothAdapter.ACTION_STATE_CHANGED.equals(action))
                {
                    if(btAdapter.getState() == btAdapter.STATE_TURNING_OFF)
                    {
                        turnonBT();
                    }
                }
            }
        };
        registerReceiver(receiver,filter);
        filter = new IntentFilter(BluetoothAdapter.ACTION_DISCOVERY_STARTED);
        registerReceiver(receiver,filter);
        filter = new IntentFilter(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        registerReceiver(receiver,filter);
        filter = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
        registerReceiver(receiver,filter);
    }

    @Override
    protected void onPause() {
        super.onPause();
        unregisterReceiver(receiver);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == 1) {
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(getApplicationContext(), "BLUETOOTH CANNOT BE TURNED ON", Toast.LENGTH_LONG).show();
            }
            if (resultCode == RESULT_OK) {
                Toast.makeText(getApplicationContext(), "BLUETOOTH SUCCESSFULLY TURNED ON", Toast.LENGTH_LONG).show();
            }
        }
    }

    @Override
    public void onItemClick(AdapterView<?> parent, View view, int position, long id) {

        if(btAdapter.isDiscovering())
        {
            btAdapter.cancelDiscovery();
        }
        if(listAdapter.getItem(position).contains("Paired"))
        {
            Object[] o = devicesArray.toArray();
            BluetoothDevice selectedDevice = (BluetoothDevice)o[position];
            Toast.makeText(getApplicationContext(),"Device paired", Toast.LENGTH_LONG).show();
             ConnectThread connect = new ConnectThread(selectedDevice);
            connect.start();
        }
        else
        {
            Toast.makeText(getApplicationContext(),"Device not paired", Toast.LENGTH_LONG).show();
        }
    }

    private class ConnectThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final BluetoothDevice mmDevice;

        public ConnectThread(BluetoothDevice device) {
            // Use a temporary object that is later assigned to mmSocket,
            // because mmSocket is final
            BluetoothSocket tmp = null;
            mmDevice = device;

            // Get a BluetoothSocket to connect with the given BluetoothDevice
            try {
                // MY_UUID is the app's UUID string, also used by the server code
                tmp = device.createRfcommSocketToServiceRecord(MY_UUID);
            } catch (IOException e) { }
            mmSocket = tmp;
        }

        public void run() {
            // Cancel discovery because it will slow down the connection
            btAdapter.cancelDiscovery();

            try {
                // Connect the device through the socket. This will block
                // until it succeeds or throws an exception
                mmSocket.connect();
            } catch (IOException connectException) {
                // Unable to connect; close the socket and get out
                try {
                    mmSocket.close();
                } catch (IOException closeException) { }
                return;
            }

            // Do work to manage the connection (in a separate thread)
            manageConnectedSocket(mmSocket);
        }

        private void manageConnectedSocket(BluetoothSocket mmSocket) {
        }

        /** Will cancel an in-progress connection, and close the socket */
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) { }
        }
    }
}
