package com.example.projectvelaris;

import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
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
                stopListening();
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
                while (subscribed) {
                    DatagramPacket responsePacket = new DatagramPacket(response, response.length);
                    socket.receive(responsePacket);

                    String responseText = new String(responsePacket.getData(), 0, responsePacket.getLength());
                    packetCount++;
                    runOnUiThread(() -> displayJson(responseText));
                }

            } catch (Exception e) {
                e.printStackTrace();
                runOnUiThread(() -> textView.setText("❌ Error: " + e.getMessage()));
            } finally {
                if (socket != null && !socket.isClosed()) {
                    socket.close();
                }
                socket = null;
                runOnUiThread(() -> textView.setText("Unsubscribe successfully"));
            }
        });
        listenerThread.start();
    }

    private void stopListening() {
        subscribed = false;

        // Send unsubscribe using the same socket BEFORE closing it
        try {
            if (socket != null && !socket.isClosed() && cachedServerAddress != null) {
                JSONObject unsubscribeMessage = new JSONObject();
                unsubscribeMessage.put("command", "unsubscribe");
                byte[] buffer = unsubscribeMessage.toString().getBytes();
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length, cachedServerAddress, SERVER_PORT);
                socket.send(packet);
                Thread.sleep(100); // allow message to go out before closing socket
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Now close the socket
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
        socket = null;

        // Stop listener thread
        if (listenerThread != null && listenerThread.isAlive()) {
            listenerThread.interrupt();
        }
        listenerThread = null;
    }

    private InetAddress resolveServerAddress() throws UnknownHostException {
        try {
            InetAddress address = InetAddress.getByName("iot");
            runOnUiThread(() -> textView.setText("📡 Trying host: iot"));
            return address;
        } catch (UnknownHostException e1) {
            try {
                InetAddress fallback = InetAddress.getByName("iot.local");
                runOnUiThread(() -> textView.setText("📡 Fallback to host: iot.local"));
                return fallback;
            } catch (UnknownHostException e2) {
                runOnUiThread(() -> textView.setText("❌ Host resolution failed for both 'iot' and 'iot.local'"));
                throw new UnknownHostException("Both 'iot' and 'iot.local' failed to resolve.");
            }
        }
    }

    private void displayJson(String jsonStr) {
        try {
            JSONObject json = new JSONObject(jsonStr.substring(jsonStr.indexOf("{")));

            int alert = json.optInt("alert", 0);

            if (alert != lastAlert) {
                ToneGenerator toneGen = new ToneGenerator(AudioManager.STREAM_NOTIFICATION, 100);
                if (alert == 1) {
                    toneGen.startTone(ToneGenerator.TONE_PROP_BEEP, 200);
                } else {
                    toneGen.startTone(ToneGenerator.TONE_PROP_ACK, 200);
                }
                lastAlert = alert;
            }

            StringBuilder sb = new StringBuilder();
            sb.append("📦 Packet #").append(packetCount).append("\n\n");
            sb.append("🌡 Temperature: ").append(json.optString("temperature", "N/A")).append(" °C\n");
            sb.append("❤️ BPM: ").append(json.optString("bpm", "N/A")).append("\n");

            JSONObject acc = json.optJSONObject("acceleration");
            if (acc != null) {
                sb.append("🧭 Acceleration\n");
                sb.append("   x: ").append(acc.optString("x", "N/A")).append("\n");
                sb.append("   y: ").append(acc.optString("y", "N/A")).append("\n");
                sb.append("   z: ").append(acc.optString("z", "N/A")).append("\n");
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

            sb.append("📍 Closest Beacon ID: ").append(json.optString("closest_beacon", "N/A")).append("\n");
            sb.append("📶 Avg RSSI: ").append(json.optString("avg_rssi", "N/A")).append(" dBm\n");

            double distance = json.optDouble("estimated_distance", -1);
            if (distance >= 0) {
                sb.append("📏 Estimated Distance: ").append(String.format("%.2f", distance)).append(" m\n");
            } else {
                sb.append("📏 Estimated Distance: N/A\n");
            }

            sb.append("🚨 Alert: ").append(alert);

            textView.setText(sb.toString());

        } catch (JSONException e) {
            textView.setText("Invalid JSON:\n" + jsonStr);
        }
    }
}
