package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Field;
import java.text.SimpleDateFormat;
import java.util.*;

public class Logger {
    private final String matchName;
    private final long startTime;
    private final File logFile;
    private final List<Map<String, Object>> dataPoints = new ArrayList<>();

    public Logger(String matchName) {
        this.matchName = matchName;
        this.startTime = System.currentTimeMillis();

        String timestamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US)
                .format(new Date());
        String filename = matchName + "_" + timestamp + ".json";

        File logDir = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logDir.exists()) logDir.mkdirs();

        logFile = new File(logDir, filename);
        RobotLog.ii("Logger", "Logging to: " + logFile.getAbsolutePath());
    }

    // Main method to log any object’s annotated fields
    public void put(String label, Object value) {
        Map<String, Object> snapshot = new HashMap<>();
        snapshot.put("timestamp", System.currentTimeMillis() - startTime);
        snapshot.put("label", label);
        snapshot.put("value", value);

        dataPoints.add(snapshot);
    }

    // Save all logged snapshots to file
    public void save() {
        try (FileWriter writer = new FileWriter(logFile)) {
            writer.write("{\n");
            writer.write("  \"matchName\": \"" + matchName + "\",\n");
            writer.write("  \"startTime\": " + startTime + ",\n");
            writer.write("  \"duration\": " + (System.currentTimeMillis() - startTime) + ",\n");
            writer.write("  \"dataPoints\": [\n");

            for (int i = 0; i < dataPoints.size(); i++) {
                Map<String, Object> dp = dataPoints.get(i);
                writer.write("    {\n");
                int j = 0;
                for (Map.Entry<String, Object> entry : dp.entrySet()) {
                    writer.write("      \"" + entry.getKey() + "\": \"" + entry.getValue() + "\"");
                    writer.write(j < dp.size() - 1 ? ",\n" : "\n");
                    j++;
                }
                writer.write("    }" + (i < dataPoints.size() - 1 ? "," : "") + "\n");
            }

            writer.write("  ]\n");
            writer.write("}\n");

            RobotLog.ii("Logger", "Saved " + dataPoints.size() + " data points");
        } catch (IOException e) {
            RobotLog.ee("Logger", "Failed to save log: " + e.getMessage());
        }
    }
}
