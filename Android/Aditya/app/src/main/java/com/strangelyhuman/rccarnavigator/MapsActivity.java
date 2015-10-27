package com.strangelyhuman.rccarnavigator;

import android.content.Intent;
import android.graphics.Color;
import android.net.Uri;
import android.os.AsyncTask;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.Button;
import android.widget.Toast;

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
import java.io.UnsupportedEncodingException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.xml.transform.Source;


public class MapsActivity extends FragmentActivity {

    private GoogleMap mMap; // Might be null if Google Play services APK is not available.
    ArrayList<LatLng> MarkerPoints;



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_maps);
        setUpMapIfNeeded();

        mMap.setMyLocationEnabled(true);

        //initialize your latlng array
        MarkerPoints =  new ArrayList<LatLng>();

        Button btnRoute = (Button) findViewById(R.id.routeButton);

        mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {

            @Override
            public void onMapClick(LatLng point) {

                //check if the no of markers on map is > 2
                if (MarkerPoints.size() > 2) {
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
                LatLng carOrigin  = MarkerPoints.get(0);
                if (MarkerPoints.size() == 2) {
                    options.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN));
                    options.title("Destination");
                    options.draggable(true);
                    Toast.makeText(MapsActivity.this, "Add a Destination Marker", Toast.LENGTH_SHORT).show();
                }


                //add marker to the map
                mMap.addMarker(options);
                if(MarkerPoints.size() >= 2) {
                    LatLng origin = MarkerPoints.get(0);
                    LatLng destination = MarkerPoints.get(1);
                    //getting a string for the directions api
                    String url = getDirectionsUrl(origin, destination);
                    DownloadTask downloadTask = new DownloadTask();
                    downloadTask.execute(url);
                }
                }
            });
        //add an event for button route
        }

    private String getDirectionsUrl(LatLng origin, LatLng destination){
        //Origin of the route
        String originStr ="origin=" + origin.latitude + "," + origin.longitude;
        //Toast.makeText(MapsActivity.this, "Creating Origin Parameters", Toast.LENGTH_SHORT).show();
        String destinationStr = "destination=" + destination.latitude + "," + destination.longitude;
        //Toast.makeText(MapsActivity.this, "Creating Destination Parameters", Toast.LENGTH_SHORT).show();
        String output = "json";
        String parameters = originStr+"&"+destinationStr+"&key=AIzaSyCZC6TjANIYInh6vqhJqSXS1qkedgnEwXA";
        //building the string for the web service
        String url = "https://maps.googleapis.com/maps/api/directions/"+output + "?" + parameters;
        return url;
    }

    private String downloadUrl(String strUrl) throws  IOException {
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
            while((line = br.readLine()) != null){
                sb.append(line);
            }

            data = sb.toString();
            br.close();
        } catch (Exception e){
            Log.d("Exception download url", e.toString());
        } finally {
            iStream.close();
            urlConnection.disconnect();
        }
        return data;
    }

    //fetches data from url passed - DownloadTask method
    private class DownloadTask extends AsyncTask<String, Void, String>{

        //download data in non-ui thread
        @Override
        protected String doInBackground(String...url){

            //store data from the web service
            String data ="";
            try{
                //fetching data from the web service
                data = downloadUrl(url[0]);
            }catch (Exception e){
                Log.d("Background Task", e.toString());
            }
            return data;
        }

        //executes in UI thread after doInBackground
        @Override
        protected void onPostExecute(String result){
            super.onPostExecute(result);

            //invokes thread for parsing json data
            ParserTask parserTask = new ParserTask();
            parserTask.execute(result);
        }

    }

    //defining ParserTask to parse JSON Data
    private class ParserTask extends AsyncTask<String, Integer, List<List<HashMap<String, String>>> >{

        //parsing the data in non-ui thread
        @Override
        protected List<List<HashMap<String,String>>> doInBackground(String... jsonData){

            JSONObject jObject;
            List<List<HashMap<String,String>>> routes = null;

            try{
               jObject = new JSONObject(jsonData[0]);
               DirectionsJSONParser parser = new DirectionsJSONParser();

               //start your parsing here
                routes = parser.parse(jObject);
            } catch (Exception e){
                e.printStackTrace();
            }
            return routes;
        }

        //executes in UI thread
        @Override
        protected void onPostExecute( List<List<HashMap<String,String>>> result){
            ArrayList<LatLng> points = null;
            PolylineOptions lineOptions = null;
            MarkerOptions markerOptions = new MarkerOptions();

            //traversing through all the routes
            //lineOptions = new PolylineOptions();
            for(int i = 0; i < result.size(); i++){
                points = new ArrayList<LatLng>();
                lineOptions = new PolylineOptions();

                //fetching ith route
                List<HashMap<String,String>> path = result.get(i);

                //fetching points in the ith route
                for(int j = 0; j < path.size(); j++){
                    HashMap<String,String> point = path.get(j);

                    double lat = Double.parseDouble(point.get("lat"));
                    double lng = Double.parseDouble(point.get("lng"));
                    LatLng position = new LatLng(lat, lng);

                    points.add(position);
                }
                //adding all points in the route to lineOptions
                lineOptions.addAll(points);
                lineOptions.width(2);
                lineOptions.color(Color.RED);
                //mMap.addPolyline(lineOptions);
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
     * <p/>
     * If it isn't installed {@link SupportMapFragment} (and
     * {@link com.google.android.gms.maps.MapView MapView}) will show a prompt for the user to
     * install/update the Google Play services APK on their device.
     * <p/>
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
     * <p/>
     * This should only be called once and when we are sure that {@link #mMap} is not null.
     */
    private void setUpMap() {

        //app behavior when the app first starts up
        //centers the map at the SJSU campus
        LatLng SJSU = new LatLng(37.3351874, -121.8810715);
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(SJSU, 6));
        mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(SJSU, 18));

        //Click Listeners

        /*Button Definitions*/
        Button button1 = (Button) findViewById(R.id.srcButton);
        Button button2 = (Button) findViewById(R.id.destButton);
        Button button3 = (Button) findViewById(R.id.routeButton);
        Button button4 = (Button) findViewById(R.id.TxButton);
        Button button5 = (Button) findViewById(R.id.clearButton);

        //what to do when you click on the source button
        button1.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                //Do something in response to button click
                mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
                    @Override
                    public void onMapClick(LatLng source) {
                        mMap.addMarker(new MarkerOptions()
                                .anchor(0.5f, 0.5f)
                                .position(source)
                                .title("Source")
                                .draggable(true));
                    }
                });
            }
        });



        //what to do when you click on the destination button
        button2.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Do something in response to button click
                mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
                    @Override
                    public void onMapClick(LatLng destination) {
                        mMap.addMarker(new MarkerOptions()
                                .anchor(0.5f, 0.5f)
                                .position(destination)
                                .title("Destination")
                                .draggable(true));
                    }
                });
            }
        });


        //what to do when you click on the route button
        button3.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Do something in response to button click
                //which in this case is create a route between our source and destination markers
//                mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener(){
//
//                });
                //https://maps.googleapis.com/maps/api/directions/json?origin=source&destination=destination&key=rccarnavigator;

            }
        });

        //what to do when you click on the send button
        button4.setOnClickListener(new View.OnClickListener() {
            public void  onClick(View v) {
                //Do something in response to button click
                //which in this case equates to sending co-ordinates to your RC Car
            }
        });

        button5.setOnClickListener(new View.OnClickListener(){
            public  void onClick(View v) {
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




