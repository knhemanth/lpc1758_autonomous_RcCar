package com.strangelyhuman.rccarnavigator;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.DialogFragment;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.Color;
import android.net.ConnectivityManager;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Build;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.Button;
import android.widget.Toast;
import android.support.v7.widget.Toolbar;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.UiSettings;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.PolylineOptions;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.lang.reflect.Method;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.UUID;

import javax.xml.transform.Source;


    public class MapsActivity extends FragmentActivity {

        private GoogleMap mMap; // Might be null if Google Play services APK is not available.
        ArrayList<LatLng> MarkerPoints;

        private static String TAG = "Maps Activity";
        String CarRoute = "";
        int Count_Ordinates = 0;
        double destinationLat;
        double destinationLon;

        Button btnOn, btnOff,btOn,btConnect,btdisconnect, sndrt;
        int BT_CONNECT_CODE = 1;
        int connect = 0;
        private BluetoothAdapter btAdapter;
        private BluetoothSocket btSocket;
        private OutputStream outStream = null;
        private InputStream instream = null;

        // SPP UUID service
        private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

        // MAC-address of Bluetooth module (you must edit this line)
        private static String address = "20:15:03:03:09:75";

        private static String tx_data1 = "10000000\n";
        private static String tx_data2 = "00000000\n";



        public boolean isConnected() {
            ConnectivityManager connectivityManager = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);

            return connectivityManager.getActiveNetworkInfo() != null && connectivityManager.getActiveNetworkInfo().isConnectedOrConnecting();
        }

//    public class makeRouteDialogFragment extends DialogFragment {
//
//        @Override
//        public Dialog onCreateDialog(Bundle savedInstanceState) {
//            Bundle args = getArguments();
//            String title = args.getString("Plot a Route");
//            String message = args.getString("Would you like to plot a route between the two points?");
//
//            return new AlertDialog.Builder(getActivity())
//                    .setTitle(title)
//                    .setMessage(message)
//
//                    .setPositiveButton(android.R.string.yes, new DialogInterface.OnClickListener() {
//                        @Override
//                        public  void onClick(DialogInterface dialog, int which) {
//                            getTargetFragment().onActivityResult(getTargetRequestCode(), Activity.RESULT_OK, null);
//                        }
//                    })
//
//                    .setNegativeButton(android.R.string.no, new DialogInterface.OnClickListener() {
//                        @Override
//                        public void onClick(DialogInterface dialog, int which) {
//                            getTargetFragment().onActivityResult(getTargetRequestCode(), Activity.RESULT_CANCELED, null);
//                        }
//                    })
//                    .create();
//        }
//    }

        @Override
        protected void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            setContentView(R.layout.activity_maps);
            //Toolbar myToolbar = (Toolbar) findViewById(R.id.my_toolbar);
            setUpMapIfNeeded();
            mMap.setMyLocationEnabled(true);

            btnOn = (Button) findViewById(R.id.btnOn);
            btnOff = (Button) findViewById(R.id.btnOff);
            btOn = (Button) findViewById(R.id.btOn);
            btConnect = (Button) findViewById(R.id.btConnect);
            btdisconnect = (Button)findViewById(R.id.btdisconnect);
            sndrt = (Button) findViewById(R.id.sndrt);
            btAdapter = BluetoothAdapter.getDefaultAdapter();

            btOn.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    checkBTState();
                }
            });

            btnOn.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    if(connect == 1) {
                        sendData(tx_data1);
                        Toast.makeText(getBaseContext(), "Turn on LED", Toast.LENGTH_SHORT).show();
                    }
                }
            });

            btnOff.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    if(connect == 1) {
                        sendData(tx_data2);
                        Toast.makeText(getBaseContext(), "Turn off LED", Toast.LENGTH_SHORT).show();
                    }
                }
            });

            btConnect.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    if (connect == 0) {
                        connect();
                    }
                }
            });

            btdisconnect.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    if(connect == 1) {
                        disconnect();
                    }
                }
            });

            sndrt.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    String btsend = Double.toString(destinationLat);
                    Log.d(TAG, btsend);
                    sendData(".\n");
                    Toast.makeText(getBaseContext(), "Sending Route...", Toast.LENGTH_SHORT).show();
                    Log.d(TAG, CarRoute);
                }
            });

            //check if your phone is connected to the internet
            if (!isConnected()) {
                Toast.makeText(MapsActivity.this, "Please ensure that you are connected to the internet", Toast.LENGTH_LONG).show();
            }

            //AlertDialog dialog = new makeRouteDialogFragment()

            //initialize your latlng array
            MarkerPoints = new ArrayList<LatLng>();
            mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
                @Override
                public void onMapClick(LatLng point) {

                    //check if the no of markers on map is > 2
                    if (MarkerPoints.size() >= 2) {
                        MarkerPoints.clear();
                        mMap.clear();
                        return;
                    }

                    //Adding a new point
                    MarkerPoints.add(point);

                    //create new marker options
                    MarkerOptions options = new MarkerOptions();

                    //set the position of the marker
                    options.position(point);

                    if (MarkerPoints.size() == 1) {
                        options.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_RED));
                        options.draggable(true);
                        options.title("Source");
                        Toast.makeText(MapsActivity.this, "Add a Origin Marker", Toast.LENGTH_SHORT).show();
                    }

                    if (MarkerPoints.size() == 2) {
                        options.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN));
                        options.title("Destination" + " " + point.latitude + " " + point.longitude);
                        options.draggable(true);
                        Toast.makeText(MapsActivity.this, "Add a Destination Marker", Toast.LENGTH_SHORT).show();
                    }

                 // DestinationMarker += MarkerPoints.get(1).latitude + " " + MarkerPoints.get(1).longitude;


                    //add marker to the map
                    mMap.addMarker(options);
                    if (MarkerPoints.size() == 2) {
                        LatLng origin = MarkerPoints.get(0);
                        LatLng destination = MarkerPoints.get(1);
                        //getting a string for the directions api
                        destinationLat = destination.latitude;
                        destinationLon = destination.longitude;
                        Log.d(TAG, Double.toString(destinationLat) + " " + Double.toString(destinationLon));
                        String url = getDirectionsUrl(origin, destination);
                        String elevationUrl = getElevationUrl(origin, destination);
                        //Toast.makeText(MapsActivity.this, elevationUrl, Toast.LENGTH_LONG).show();
                        DownloadTask downloadTask1 = new DownloadTask();
                        downloadTask1.execute(url);
                    }
                }
            });
            //add an event for button route
        }

        private BluetoothSocket createBluetoothSocket(BluetoothDevice device) throws IOException {
            if(Build.VERSION.SDK_INT >= 10){
                try {
                    final Method m = device.getClass().getMethod("createInsecureRfcommSocketToServiceRecord", new Class[] { UUID.class });
                    return (BluetoothSocket) m.invoke(device, MY_UUID);
                } catch (Exception e) {
                    Log.e(TAG, "Could not create Insecure RFComm Connection",e);
                }
            }
            return  device.createRfcommSocketToServiceRecord(MY_UUID);
        }


        private void checkBTState() {
            // Check for Bluetooth support and then check to make sure it is turned on
            // Emulator doesn't support Bluetooth and will return null
            if(btAdapter==null) {
                errorExit("Fatal Error", "Bluetooth not support");
            } else {
                if (btAdapter.isEnabled()) {
                    Log.d(TAG, "...Bluetooth ON...");
                } else {
                    //Prompt user to turn on Bluetooth
                    Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                    startActivityForResult(enableBtIntent, BT_CONNECT_CODE);
                }
            }
        }

        private void errorExit(String title, String message){
            Toast.makeText(getBaseContext(), title + " - " + message, Toast.LENGTH_LONG).show();
            finish();
        }

        private void sendData(String message) {
            byte[] msgBuffer = message.getBytes();

            Log.d(TAG, "...Send data: " + message + "...");

            try {
                outStream.write(msgBuffer);
                Toast.makeText(getApplicationContext(),"data sent",Toast.LENGTH_LONG).show();
            } catch (IOException e) {
                String msg = "In onResume() and an exception occurred during write: " + e.getMessage();
                if (address.equals("00:00:00:00:00:00"))
                    msg = msg + ".\n\nUpdate your server address from 00:00:00:00:00:00 to the correct address on line 35 in the java code";
                msg = msg +  ".\n\nCheck that the SPP UUID: " + MY_UUID.toString() + " exists on server.\n\n";

                errorExit("Fatal Error", msg);
            }
        }

        private void disconnect() {
            try{
                instream = btSocket.getInputStream();
            }catch (IOException e){
                errorExit("Fatal Error", "In onResume() and input stream creation failed:" + e.getMessage() + ".");
            }

            if (instream != null) {
                try {instream.close();} catch (Exception e) {}
                instream = null;
            }

            if (outStream != null) {
                try {outStream.close();} catch (Exception e) {}
                outStream = null;
            }

            if (btSocket != null) {
                try {btSocket.close();
                    Toast.makeText(getApplicationContext(), "Disconnected", Toast.LENGTH_SHORT).show();
                    connect = 0;
                } catch (Exception e) {}
                btSocket = null;
            }

        }

        private void connect() {
            // Set up a pointer to the remote node using it's address.
            BluetoothDevice device = btAdapter.getRemoteDevice(address);

            // Two things are needed to make a connection:
            //   A MAC address, which we got above.
            //   A Service ID or UUID.  In this case we are using the
            //     UUID for SPP.
            try {
                btSocket = createBluetoothSocket(device);
            } catch (IOException e1) {
                errorExit("Fatal Error", "In onResume() and socket create failed: " + e1.getMessage() + ".");
            }

            // Discovery is resource intensive.  Make sure it isn't going on
            // when you attempt to connect and pass your message.
            btAdapter.cancelDiscovery();

            // Establish the connection.  This will block until it connects.
            Log.d(TAG, "...Connecting...");
            try {
                btSocket.connect();
                Log.d(TAG, "...Connection ok...");
                connect = 1;
                Toast.makeText(getApplicationContext(), "Connected", Toast.LENGTH_SHORT).show();

            } catch (IOException e) {
                try {
                    btSocket.close();
                } catch (IOException e2) {
                    errorExit("Fatal Error", "In onResume() and unable to close socket during connection failure" + e2.getMessage() + ".");
                }
            }

            // Create a data stream so we can talk to server.
            Log.d(TAG, "...Create Socket...");

            try {
                outStream = btSocket.getOutputStream();
            } catch (IOException e) {
                errorExit("Fatal Error", "In onResume() and output stream creation failed:" + e.getMessage() + ".");

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

        private String getElevationUrl(LatLng origin, LatLng destination) {
            //https://maps.googleapis.com/maps/api/elevation/json?path=36.578581,-118.291994|36.23998,-116.83171&samples=3&key=YOUR_API_KEY
            String origin_string = origin.latitude + "," + origin.longitude;
            String destination_string = destination.latitude + "," + destination.longitude;
            String output = "json?";
            String path = origin_string + "|" + destination_string;
            String samples = "3";
            String elevationUrl = "https://maps.googleapis.com/maps/api/elevation/" + output + "path=" + path + "&samples=" + samples + "&key=AIzaSyCwfUYxgA3FXrxX6RqlOJVbf16lHGa7uSs";
            return elevationUrl;
        }

        private String getDirectionsUrl(LatLng origin, LatLng destination) {
            //Origin of the route
            String originStr = "origin=" + origin.latitude + "," + origin.longitude;
            String destinationStr = "destination=" + destination.latitude + "," + destination.longitude;
            String output = "json";
            String parameters = originStr + "&" + destinationStr + "&mode=walking&key=AIzaSyCwfUYxgA3FXrxX6RqlOJVbf16lHGa7uSs";
            String path = origin.latitude + "," + origin.longitude + "|" + destination.latitude + "," + destination.longitude;
            //building the string for the web service
            String url = "https://maps.googleapis.com/maps/api/directions/" + output + "?" + parameters;
            //adding a road maps api
            //String url = "https://roads.googleapis.com/v1/snapToRoads?"+ path + "&key=AIzaSyCwfUYxgA3FXrxX6RqlOJVbf16lHGa7uSs";
            return url;
        }

        private String downloadUrl(String strUrl) throws IOException {
            String data = "";
            InputStream iStream = null;
            HttpURLConnection urlConnection = null;
            try {
                URL url = new URL(strUrl);

                //create a http connection
                urlConnection = (HttpURLConnection) url.openConnection();

                //Connecting to url
                urlConnection.connect();

                //reading data from Url
                iStream = urlConnection.getInputStream();

                BufferedReader br = new BufferedReader(new InputStreamReader(iStream));
                StringBuilder sb = new StringBuilder();

                String line = "";
                while ((line = br.readLine()) != null) {
                    sb.append(line);
                }

                data = sb.toString();
                br.close();
            } catch (Exception e) {
                Log.d("Exception download url", e.toString());
            } finally {
                iStream.close();
                urlConnection.disconnect();
            }
            Log.d(TAG, data);
            return data;
        }

        //fetches data from url passed - DownloadTask method
        private class DownloadTask extends AsyncTask<String, Void, String> {

            //download data in non-ui thread
            @Override
            protected String doInBackground(String... url) {

                //store data from the web service
                String data = "";
                try {
                    //fetching data from the web service
                    data = downloadUrl(url[0]);
                } catch (Exception e) {
                    Log.d("Background Task", e.toString());
                }
                return data;
            }

            //executes in UI thread after doInBackground
            @Override
            protected void onPostExecute(String result) {
                super.onPostExecute(result);

                //invokes thread for parsing json data
                ParserTask parserTask = new ParserTask();
                parserTask.execute(result);
            }

        }

        //defining ParserTask to parse JSON Data
        private class ParserTask extends AsyncTask<String, Integer, List<List<HashMap<String, String>>>> {

            //parsing the data in non-ui thread
            @Override
            protected List<List<HashMap<String, String>>> doInBackground(String... jsonData) {

                JSONObject jObject;
                List<List<HashMap<String, String>>> routes = null;

                try {
                    jObject = new JSONObject(jsonData[0]);
                    DirectionsJSONParser parser = new DirectionsJSONParser();

                    //start your parsing here
                    routes = parser.parse(jObject);
                } catch (Exception e) {
                    e.printStackTrace();
                }
                return routes;
            }

            //executes in UI thread
            @Override
            protected void onPostExecute(List<List<HashMap<String, String>>> result) {
                ArrayList<LatLng> points = null;
                PolylineOptions lineOptions = null;
                MarkerOptions markerOptions = new MarkerOptions();

                //traversing through all the routes
                //lineOptions = new PolylineOptions();
                for (int i = 0; i < result.size(); i++) {
                    points = new ArrayList<LatLng>();
                    lineOptions = new PolylineOptions();

                    //fetching ith route
                    List<HashMap<String, String>> path = result.get(i);

                    CarRoute = "";
                    Count_Ordinates = 0;
                    //fetching points in the ith route
                    for (int j = 0; j < path.size(); j++) {
                        HashMap<String, String> point = path.get(j);

                        double lat = Double.parseDouble(point.get("lat"));
                        double lng = Double.parseDouble(point.get("lng"));

                        LatLng position = new LatLng(lat, lng);
                        //CarRoute +=  "" + lat + " " + lng + " ";
                        //Log.d(TAG, CarRoute);
                        //Log.d(TAG, position.toString());
                        if ((j % 2 == 0)) {
                            mMap.addMarker(markerOptions.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_BLUE))
                                    .position(position)
                                    .title("latitude: " + lat + ", " + "longitude: " + lng));
                            CarRoute += "" + lat + " " + lng + " ";
                            ++Count_Ordinates;
                        }

                        points.add(position);

                    }
                    CarRoute += destinationLat + " " + destinationLon;
                    Log.d(TAG, CarRoute);
                    Log.d(TAG, Integer.toString(Count_Ordinates));
                    //adding all points in the route to lineOptions
                    lineOptions.addAll(points);
                    lineOptions.width(5);
                    lineOptions.color(Color.BLUE);


                }
                //draw polyline on map
                mMap.addPolyline(lineOptions);
            }

        }


        @Override
        protected void onResume() {
            super.onResume();
            setUpMapIfNeeded();
        }

        /**
         * Sets up the map if it is possible to do so (i.e., the Google Play services APK is correctly
         * installed) and the map has not already been instantiated.. This will ensure that we only ever
         * call {@link #setUpMap()} once when {@link #mMap} is not null.
         * <p>
         * If it isn't installed {@link SupportMapFragment} (and
         * {@link com.google.android.gms.maps.MapView MapView}) will show a prompt for the user to
         * install/update the Google Play services APK on their device.
         * <p>
         * A user can return to this FragmentActivity after following the prompt and correctly
         * installing/updating/enabling the Google Play services. Since the FragmentActivity may not
         * have been completely destroyed during this process (it is likely that it would only be
         * stopped or paused), {@link #onCreate(Bundle)} may not be called again so we should call this
         * method in {@link #onResume()} to guarantee that it will be called.
         */
        private void setUpMapIfNeeded() {
            // Do a null check to confirm that we have not already instantiated the map.
            if (mMap == null) {
                // Try to obtain the map from the SupportMapFragment.
                mMap = ((SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.map))
                        .getMap();
                // Check if we were successful in obtaining the map.
                if (mMap != null) {
                    setUpMap();
                }
            }
        }

        /**
         * This is where we can add markers or lines, add listeners or move the camera. In this case, we
         * just add a marker near Africa.
         * <p>
         * This should only be called once and when we are sure that {@link #mMap} is not null.
         */
        private void setUpMap() {

            //app behavior when the app first starts up
            //centers the map at the SJSU campus
            LatLng SJSU = new LatLng(37.3351874, -121.8810715);
            mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(SJSU, 6));
            mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(SJSU, 18));

            //Click Listeners

            //Button button4 = (Button) findViewById(R.id.TxButton);
            Button button5 = (Button) findViewById(R.id.clearButton);
            button5.setOnClickListener(new View.OnClickListener() {
                public void onClick(View v) {
                    //Clear all markers on screen
                    mMap.clear();
                }
            });

            //need to work on getting a location value

            //UI settings are added here
            UiSettings uiValue = mMap.getUiSettings();
            uiValue.setZoomControlsEnabled(true);
            uiValue.setMyLocationButtonEnabled(true); //only works if mMap.setLocationEnabled(true)
            uiValue.setCompassEnabled(true); //also doesn't seem to work

        }

    }




