package com.example.projectvelaris;

import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
import android.util.Log;
import android.widget.*;
import androidx.appcompat.app.AppCompatActivity;
import org.json.JSONObject;
import org.json.JSONException;
import java.net.*;

public class MainActivity extends AppCompatActivity {

    private TextView textView;
    private Button button;
    private final int SERVER_PORT = 8000;

    private boolean subscribed = false;
    private Thread listenerThread;
    private DatagramSocket socket;
    private InetAddress cachedServerAddress = null;
    private int packetCount = 0;
    private int lastAlert = 0; // Track last alert state

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        textView = findViewById(R.id.textViewResponse);
        button = findViewById(R.id.buttonSend);

        button.setOnClickListener(v -> {
            if (!subscribed) {
                subscribed = true;
                packetCount = 0;
                lastAlert = 0;
                startListening();
                button.setText("Unsubscribe");
            } else {
                subscribed = false;
                button.setText("Subscribe");
            }
        });
    }

    private void startListening() {
        listenerThread = new Thread(() -> {
            try {
                runOnUiThread(() -> textView.setText("Start listening again"));

                if (socket != null && !socket.isClosed()) {
                    socket.close();
                }

                socket = new DatagramSocket();
                cachedServerAddress = resolveServerAddress();

                JSONObject subscribeMessage = new JSONObject();
                subscribeMessage.put("command", "subscribe");
                byte[] buffer = subscribeMessage.toString().getBytes();
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length, cachedServerAddress, SERVER_PORT);
                socket.send(packet);

                byte[] response = new byte[2048];
                socket.setSoTimeout(100);  // check every 200 ms

                while (subscribed) {
                    try {
                    DatagramPacket responsePacket = new DatagramPacket(response, response.length);
                    socket.receive(responsePacket);

                    String responseText = new String(responsePacket.getData(), 0, responsePacket.getLength());
                    packetCount++;
                    runOnUiThread(() -> displayJson(responseText));
                    } catch (SocketTimeoutException e){}
                }

                JSONObject unsubscribeMessage = new JSONObject();
                unsubscribeMessage.put("command", "unsubscribe");
                buffer = unsubscribeMessage.toString().getBytes();
                packet = new DatagramPacket(buffer, buffer.length, cachedServerAddress, SERVER_PORT);
                socket.send(packet);

            } catch (Exception e) {
                e.printStackTrace();
                runOnUiThread(() -> textView.setText("❌ Error: " + e.getMessage()));
            }
            finally {
                if (socket != null && !socket.isClosed()) {
                    socket.close();
                }
                socket = null;
                runOnUiThread(() -> textView.setText("Unsubscribe successfully"));
            }
        });
        listenerThread.start();
    }


    private InetAddress resolveServerAddress() throws UnknownHostException {
        String fixedIP = "172.24.227.244";
        InetAddress address = InetAddress.getByName(fixedIP);
        runOnUiThread(() -> textView.setText("📡 Using hardcoded IP: " + fixedIP));
        return address;
    }


    private void displayJson(String jsonStr) {
        try {
            Log.d("UDP_JSON", "Received JSON: " + jsonStr);
            JSONObject json = new JSONObject(jsonStr.substring(jsonStr.indexOf("{")));

            int alert = json.optInt("alert", 0);
            if (alert != lastAlert) {
                ToneGenerator toneGen = new ToneGenerator(AudioManager.STREAM_NOTIFICATION, 100);
                toneGen.startTone(alert == 1 ? ToneGenerator.TONE_PROP_BEEP : ToneGenerator.TONE_PROP_ACK, 200);
                lastAlert = alert;
            }

            StringBuilder sb = new StringBuilder();
            sb.append("📦 Packet #").append(packetCount).append("\n\n");

            String type = json.optString("type", "unknown");

            if (type.equals("ble")) {
                sb.append("📡 Type: BLE\n");

                sb.append("🌡 Temperature: ").append(json.optString("temperature", "N/A")).append(" °C\n");
                sb.append("❤️ BPM: ").append(json.optString("bpm", "N/A")).append("\n");

                JSONObject acc = json.optJSONObject("acceleration");
                if (acc != null) {
                    sb.append("🧭 Acceleration\n");
                    sb.append("   x: ").append(acc.optString("x", "N/A")).append("\n");
                    sb.append("   y: ").append(acc.optString("y", "N/A")).append("\n");
                    sb.append("   z: ").append(acc.optString("z", "N/A")).append("\n");
                }

                sb.append("📍 Closest Beacon ID: ").append(json.optString("closest_beacon", "N/A")).append("\n");
                sb.append("📶 Avg RSSI: ").append(json.optString("avg_rssi", "N/A")).append(" dBm\n");

                double distance = json.optDouble("estimated_distance", -1);
                if (distance >= 0) {
                    sb.append("📏 Estimated Distance: ").append(String.format("%.2f", distance)).append(" m\n");
                } else {
                    sb.append("📏 Estimated Distance: N/A\n");
                }

            } else if (type.equals("lora")) {
                sb.append("📡 Type: LoRa\n");

                JSONObject gps = json.optJSONObject("gps");
                if (gps != null) {
                    sb.append("🛰 GPS\n");

                    int fix = gps.optInt("fix", 0);
                    double latRaw = gps.optDouble("lat", 0.0);
                    double lonRaw = gps.optDouble("lon", 0.0);
                    double alt = gps.optDouble("alt", 0.0);
                    int sats = gps.optInt("sats", 0);

                    String latDir = gps.optString("lat_dir", "N");
                    String lonDir = gps.optString("lon_dir", "E");

                    // Raw NMEA format
                    sb.append("   Fix: ").append(fix).append("\n");

                    // D° M.MMMM' format
                    int latDeg = (int)(latRaw / 100);
                    double latMin = latRaw - (latDeg * 100);
                    int lonDeg = (int)(lonRaw / 100);
                    double lonMin = lonRaw - (lonDeg * 100);

                    String latFormatted = String.format("%d° %.4f' %s", latDeg, latMin, latDir);
                    String lonFormatted = String.format("%d° %.4f' %s", lonDeg, lonMin, lonDir);
                    sb.append("   Latitude: ").append(latFormatted).append("\n");
                    sb.append("   Longitude: ").append(lonFormatted).append("\n");

                    // Decimal degrees (for map link)
                    double latDecimal = latDeg + (latMin / 60.0);
                    double lonDecimal = lonDeg + (lonMin / 60.0);
                    if (latDir.equals("S")) latDecimal = -latDecimal;
                    if (lonDir.equals("W")) lonDecimal = -lonDecimal;

                    sb.append("   Altitude: ").append(alt).append(" m\n");
                    sb.append("   Satellites: ").append(sats).append("\n");

                    if (fix == 1) {
                        String mapLink = "https://www.google.com/maps?q=" + latDecimal + "," + lonDecimal;
                        sb.append("🗺 <a href=\"").append(mapLink).append("\"><b>Open in Google Maps</b></a>\n");
                    }
                }
            }

            JSONObject wrist = json.optJSONObject("wrist_batt");
            if (wrist != null) {
                sb.append("🔋 Wrist Battery\n");
                sb.append("   V: ").append(wrist.optString("V", "N/A")).append("\n");
                sb.append("   P: ").append(wrist.optString("P", "N/A")).append("\n");
            }

            JSONObject beacon = json.optJSONObject("beacon_batt");
            if (beacon != null) {
                sb.append("📡 Beacon Battery\n");
                sb.append("   V: ").append(beacon.optString("V", "N/A")).append("\n");
                sb.append("   P: ").append(beacon.optString("P", "N/A")).append("\n");
            }

            sb.append("🚨 Alert: ").append(alert);

            String htmlText = sb.toString().replace("\n", "<br>");
            textView.setText(android.text.Html.fromHtml(htmlText, android.text.Html.FROM_HTML_MODE_LEGACY));
            textView.setMovementMethod(android.text.method.LinkMovementMethod.getInstance());


        } catch (JSONException e) {
            textView.setText("❌ Invalid JSON:\n" + jsonStr);
        }
    }
}
