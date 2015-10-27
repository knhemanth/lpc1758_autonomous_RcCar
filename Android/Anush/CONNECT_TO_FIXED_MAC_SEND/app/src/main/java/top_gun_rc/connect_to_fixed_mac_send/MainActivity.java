package top_gun_rc.connect_to_fixed_mac_send;

import java.io.IOException;
import java.io.OutputStream;
import java.lang.reflect.Method;
import java.util.UUID;


import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.Toast;

public class MainActivity extends Activity {

    Button btnOn, btnOff,btOn,btConnect;
    int BT_CONNECT_CODE = 1;
    int connect = 0;
    private BluetoothAdapter btAdapter;
    private BluetoothSocket btSocket;
    private OutputStream outStream = null;

    // SPP UUID service
    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    // MAC-address of Bluetooth module
    private static String address = "20:15:03:03:09:75";

    private static String tx_data1 = "12345\n";

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        btnOn = (Button) findViewById(R.id.btConnect);
        btnOff = (Button) findViewById(R.id.btnOff);
        btOn = (Button) findViewById(R.id.btOn);
        btConnect = (Button) findViewById(R.id.btConnect);

        btAdapter = BluetoothAdapter.getDefaultAdapter();
        btOn.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                checkBTState();
            }
        });

        btnOn.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                if(connect == 1) {
                    sendData(tx_data1);
                    Toast.makeText(getBaseContext(), "Turn on LED", Toast.LENGTH_SHORT).show();
                }
            }
        });

        btnOff.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                if(connect == 1) {
                    sendData(tx_data1);
                    Toast.makeText(getBaseContext(), "Turn off LED", Toast.LENGTH_SHORT).show();
                }
            }
        });

        btConnect.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (connect == 0) {
                    connect();
                }
            }
        });

    }

    private BluetoothSocket createBluetoothSocket(BluetoothDevice device) throws IOException {
        if(Build.VERSION.SDK_INT >= 10){
            try {
                final Method  m = device.getClass().getMethod("createInsecureRfcommSocketToServiceRecord", new Class[] { UUID.class });
                return (BluetoothSocket) m.invoke(device, MY_UUID);
            } catch (Exception e) {
            }
        }
        return  device.createRfcommSocketToServiceRecord(MY_UUID);
    }


    private void checkBTState() {
        // Check for Bluetooth support and then check to make sure it is turned on
        if(btAdapter==null) {
            } else {
            if (btAdapter.isEnabled()) {
            } else {
                //Prompt user to turn on Bluetooth
                Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(enableBtIntent, BT_CONNECT_CODE);
            }
        }
    }

    private void sendData(String message) {
        byte[] msgBuffer = message.getBytes();


        try {
            outStream.write(msgBuffer);
            Toast.makeText(getApplicationContext(),"data sent",Toast.LENGTH_LONG).show();
        } catch (IOException e) {
            String msg = "exception occurred during write: " + e.getMessage();
            if (address.equals("00:00:00:00:00:00"))
                msg = msg + ".\n\nUpdate your server address from 00:00:00:00:00:00 to the correct address on line 35 in the java code";
            msg = msg +  ".\n\nCheck that the SPP UUID: " + MY_UUID.toString() + " exists on server.\n\n";

              }
    }

    private void connect() {
        // Set up a pointer to the remote node using it's address.
        BluetoothDevice device = btAdapter.getRemoteDevice(address);

        try {
            btSocket = createBluetoothSocket(device);
        } catch (IOException e1) {
             }

        btAdapter.cancelDiscovery();

        // Establish the connection.  This will block until it connects.
        try {
            btSocket.connect();
            connect = 1;
            Toast.makeText(getApplicationContext(), "Connected", Toast.LENGTH_SHORT).show();

        } catch (IOException e) {
            try {
                btSocket.close();
            } catch (IOException e2) {
             }
        }

        try {
            outStream = btSocket.getOutputStream();
        } catch (IOException e) {

        }
    }

    public void onActivityResult(int req_code, int res_code, Intent data)
    {
        if(req_code == BT_CONNECT_CODE)
        {
            if(res_code == RESULT_OK)
            {
                Toast.makeText(getBaseContext(),"BLUETOOTH SUCCESSFULLY ENABLED", Toast.LENGTH_LONG).show();

            }
            if(res_code == RESULT_CANCELED)
            {
                Toast.makeText(getBaseContext(),"BLUETOOTH TURN ON FAILED", Toast.LENGTH_LONG).show();
            }
        }
    }

}
