package com.example.boatcontrol

import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import fi.iki.elonen.NanoHTTPD
import org.osmdroid.config.Configuration
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.MapEventsOverlay
import org.osmdroid.events.MapEventsReceiver
import java.io.IOException
import java.net.NetworkInterface
import java.util.Collections
import org.osmdroid.views.overlay.Marker
import android.os.Handler
import android.os.Looper
import java.net.HttpURLConnection
import java.net.URL

import android.graphics.BitmapFactory
import android.graphics.drawable.BitmapDrawable



class MainActivity : AppCompatActivity() {
    private var latestLatitude: Double = 0.0
    private var latestLongitude: Double = 0.0

    private lateinit var map: MapView
    private lateinit var server: AndroidServer
    private lateinit var startServerButton: Button
    private lateinit var setDestinationButton: Button
    private lateinit var serverStatusText: TextView
    private lateinit var usvStatusText: TextView
    private var destinationMarker: Marker? = null
    private var usvMarker: Marker? = null
    private val handler = Handler(Looper.getMainLooper())
    private var targetLatitude: Double? = null
    private var targetLongitude: Double? = null
    private var usvLatitude: Double? = null
    private var usvLongitude: Double? = null

    private lateinit var yawValueText: TextView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Configuration.getInstance().load(applicationContext, getPreferences(MODE_PRIVATE))
        setContentView(R.layout.activity_main)

        yawValueText = findViewById(R.id.yawValueText)

        startServerButton = findViewById(R.id.startServerButton)
        setDestinationButton = findViewById(R.id.setDestinationButton)
        serverStatusText = findViewById(R.id.serverStatusText)
        usvStatusText = findViewById(R.id.usvStatusText)
        map = findViewById(R.id.map)

        map.setTileSource(TileSourceFactory.MAPNIK)
        map.setMultiTouchControls(true)

        startServerButton.setOnClickListener { startServer() }
        setDestinationButton.setOnClickListener { showMap() }
    }

    private fun startServer() {
        server = AndroidServer(8080)
        try {
            server.start()
            val ipAddress = getLocalIpAddress()
            Log.d("AndroidServer", "Server started at IP: $ipAddress")
            runOnUiThread {
                serverStatusText.text = "Server running at: $ipAddress"
                Toast.makeText(this, "Server started at: $ipAddress", Toast.LENGTH_SHORT).show()
            }
        } catch (e: IOException) {
            Log.e("AndroidServer", "Failed to start server", e)
            runOnUiThread {
                serverStatusText.text = "Server failed to start"
                Toast.makeText(this, "Server failed to start", Toast.LENGTH_SHORT).show()
            }
        }
    }

    private fun getLocalIpAddress(): String {
        try {
            val interfaces = Collections.list(NetworkInterface.getNetworkInterfaces())
            for (intf in interfaces) {
                val addrs = Collections.list(intf.inetAddresses)
                for (addr in addrs) {
                    if (!addr.isLoopbackAddress && addr.hostAddress.indexOf(':') < 0) {
                        return addr.hostAddress
                    }
                }
            }
        } catch (e: Exception) {
            Log.e("AndroidServer", "Error getting IP address", e)
        }
        return "0.0.0.0"
    }

    private fun showMap() {
        setDestinationButton.visibility = View.GONE
        map.visibility = View.VISIBLE

        val mapController = map.controller
        mapController.setZoom(10.0)
        mapController.setCenter(GeoPoint(37.7749, -122.4194))

        val mapEventsReceiver = object : MapEventsReceiver {
            override fun singleTapConfirmedHelper(p: GeoPoint?): Boolean {
                p?.let {
                    latestLatitude = it.latitude
                    latestLongitude = it.longitude
                    Log.d("MainActivity", "Tapped coordinates: $latestLatitude, $latestLongitude")

                    runOnUiThread {
                        usvStatusText.text = "Destination set: $latestLatitude, $latestLongitude"
                        destinationMarker?.let { map.overlays.remove(it) }
                        destinationMarker = Marker(map).apply {
                            position = it
                            title = "Destination"
                            setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
                        }
                        map.overlays.add(destinationMarker)
                        map.invalidate()
                    }
                    sendCoordinatesToESP(it.latitude, it.longitude)
                }
                return true
            }
            override fun longPressHelper(p: GeoPoint?): Boolean = false
        }
        map.overlays.add(MapEventsOverlay(mapEventsReceiver))
    }

    private fun sendCoordinatesToESP(latitude: Double, longitude: Double) {
        Log.d("MainActivity", "Sending coordinates: $latitude, $longitude")
        runOnUiThread { usvStatusText.text = "Coordinates sent: $latitude, $longitude" }

        Thread {
            try {
                val url = URL("http://192.168.51.20:8080/update?lat=$latitude&lon=$longitude")
                val connection = url.openConnection() as HttpURLConnection
                connection.requestMethod = "GET"
                connection.connectTimeout = 5000
                connection.readTimeout = 5000
                val responseCode = connection.responseCode

                if (responseCode == 200) {
                    Log.d("MainActivity", "Coordinates sent successfully")
                } else {
                    Log.e("MainActivity", "Failed to send coordinates, response code: $responseCode")
                }
            } catch (e: Exception) {
                Log.e("MainActivity", "Error sending coordinates", e)
            }
        }.start()
    }

    override fun onDestroy() {
        super.onDestroy()
        server.stop()
    }

    inner class AndroidServer(port: Int) : NanoHTTPD(port) {
        override fun serve(session: IHTTPSession): Response {
            Log.d("AndroidServer", "Request received: ${session.uri}")

            return when (session.uri) {
                "/connect" -> {
                    runOnUiThread {
                        serverStatusText.text = "USV Connected!"
                        setDestinationButton.visibility = View.VISIBLE
                    }
                    newFixedLengthResponse("Connected to Android Server")
                }
                "/update" -> {
                    val params = session.parameters
                    val lat = params["lat"]?.get(0)?.toDoubleOrNull()
                    val lon = params["lon"]?.get(0)?.toDoubleOrNull()

                    if (lat != null && lon != null) {
                        targetLatitude = lat
                        targetLongitude = lon
                        runOnUiThread { usvStatusText.text = "Heading to: $lat, $lon" }
                        Log.d("AndroidServer", "Received and stored new destination: $lat, $lon")
                        newFixedLengthResponse("OK")
                    } else if (targetLatitude != null && targetLongitude != null) {
                        newFixedLengthResponse("$targetLatitude,$targetLongitude")
                    } else {
                        Log.e("AndroidServer", "No target coordinates available")
                        newFixedLengthResponse("No coordinates set")
                    }
                }
                "/current_location" -> {
                    val params = session.parameters
                    val lat = params["lat"]?.get(0)?.toDoubleOrNull()
                    val lon = params["lon"]?.get(0)?.toDoubleOrNull()
                    val yaw = params["yaw"]?.get(0)?.toFloatOrNull() ?: 0f  // Default to 0 if missing

                    if (lat != null && lon != null) {
                        runOnUiThread { updateUSVLocation(lat, lon, yaw) }
                        newFixedLengthResponse("OK")
                    } else {
                        newFixedLengthResponse("Invalid location data")
                    }
                }
                else -> newFixedLengthResponse("Invalid request")
            }
        }
    }
    private fun updateUSVLocation(lat: Double, lon: Double, yaw: Float) {
        runOnUiThread {
            yawValueText.text = "Yaw: ${yaw}°"

            usvMarker?.let { map.overlays.remove(it) }
            val drawable = BitmapDrawable(resources,
                BitmapFactory.decodeResource(resources, R.drawable.usv_icon))

            usvMarker = Marker(map).apply {
                position = GeoPoint(lat, lon)
                title = "USV Location"
                setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
                setIcon(drawable)
                rotation = yaw  // Set marker rotation
            }

            map.overlays.add(usvMarker)
            map.invalidate()
        }
    }
}