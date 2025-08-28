import android.util.Log
import fi.iki.elonen.NanoHTTPD
import java.io.IOException

class AndroidServer(port: Int) : NanoHTTPD(port) {

    override fun serve(session: IHTTPSession): Response {
        if (session.method == Method.POST && session.uri == "/set-destination") {
            // Parse JSON data from the request
            val json = session.body
            Log.d("AndroidServer", "Received data: $json")

            // Send a response back to the ESP8266
            return newFixedLengthResponse("Coordinates received!")
        }
        return newFixedLengthResponse("Invalid request")
    }
}