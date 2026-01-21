package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class Logger {
    private final List<DataPoint> dataPoints = new ArrayList<>();
    private final String matchName;
    private final long startTime;
    private final File logFile;

    public static class loopTime {
        public double drive;
        public double shooter;
        public double intake;
        public double led;
        public double limelight;
    }

    public static class DataPoint {
        public double timestamp;
        public double shooterVelocity;
        public double shooterTarget;
        public double gatePosition;
        public double targetDistance;
        public double posX;
        public double posY;
        public double heading;
        public loopTime loopTime;
        public String robotState;
        public double voltage;

        public DataPoint(double timestamp) {
            this.timestamp = timestamp;
        }
    }

    public Logger(String matchName) {
        this.matchName = matchName;
        this.startTime = System.currentTimeMillis();

        String timestamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US)
                .format(new Date());
        String filename = matchName + "_" + timestamp + ".json";

        File logDir = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logDir.exists()) {
            logDir.mkdirs();
        }

        logFile = new File(logDir, filename);
        RobotLog.ii("Logger", "Logging to: " + logFile.getAbsolutePath());
    }

    public void log(DataPoint point) {
        dataPoints.add(point);
    }

    public void save() {
        try (FileWriter writer = new FileWriter(logFile)) {
            writer.write("{\n");
            writer.write("  \"matchName\": \"" + matchName + "\",\n");
            writer.write("  \"startTime\": " + startTime + ",\n");
            writer.write("  \"duration\": " + (System.currentTimeMillis() - startTime) + ",\n");
            writer.write("  \"dataPoints\": [\n");

            for (int i = 0; i < dataPoints.size(); i++) {
                DataPoint dp = dataPoints.get(i);
                writer.write("    {\n");
                writer.write("      \"timestamp\": " + dp.timestamp + ",\n");
                writer.write("      \"shooterVelocity\": " + dp.shooterVelocity + ",\n");
                writer.write("      \"shooterTarget\": " + dp.shooterTarget + ",\n");
                writer.write("      \"posX\": " + dp.posX + ",\n");
                writer.write("      \"posY\": " + dp.posY + ",\n");
                writer.write("      \"heading\": " + dp.heading + ",\n");
                writer.write("      \"loopTimeDrive\": " + dp.loopTime.drive + ",\n");
                writer.write("      \"loopTimeIntake\": " + dp.loopTime.intake + ",\n");
                writer.write("      \"loopTimeShooter\": " + dp.loopTime.shooter + ",\n");
                writer.write("      \"loopTimeLED\": " + dp.loopTime.led + ",\n");
                writer.write("      \"loopTimeLimelight\": " + dp.loopTime.limelight + ",\n");
                writer.write("      \"voltage\": " + dp.voltage + ",\n");
                writer.write("      \"robotState\": \"" + dp.robotState + "\"\n");
                writer.write("    }" + (i < dataPoints.size() - 1 ? "," : "") + "\n");
            }

            writer.write("  ]\n");
            writer.write("}\n");

            RobotLog.ii("DataLogger", "Saved " + dataPoints.size() + " data points");
        } catch (IOException e) {
            RobotLog.ee("DataLogger", "Failed to save log: " + e.getMessage());
        }
    }
}