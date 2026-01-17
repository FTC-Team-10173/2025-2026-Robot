package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

public class Limelight implements Subsystem {

    private final Limelight3A limelight;
    private final IMU imu; // Currently unused but may incorporate for more accurate localization
    private double yawOffset;

    private CameraState currentState;
    private int desiredPipeline;

    private LLStatus llStatus;

    public final Results results;

    public enum CameraState {
        IDLE,
        SEARCHING,
        TARGETING,
        STOPPED
    }

    /** Snapshot of vision outputs */
    public static class Results {
        public boolean hasTarget;
        public double distanceMeters;
        public double tx;
        public double ty;
        public double ta;
        public int motifID;
        public LLResult result;
    }

    /** Bot pose based on vision */
    public static class Botpose {
        public double x;
        public double y;
        public LLResult result;
    }

    // Constructor
    public Limelight(HardwareMap hardwareMap, IMU imu) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.imu = imu;

        limelight.setPollRateHz(100); // update at 100 Hz (100 times per second)
        limelight.start(); // start the limelight processing

        desiredPipeline = 0;
        limelight.pipelineSwitch(desiredPipeline);

        currentState = CameraState.IDLE;
        results = new Results();
    }

    /** Fluent pipeline switch */
    public Limelight setPipeline(int pipelineIndex) {
        desiredPipeline = pipelineIndex;
        limelight.pipelineSwitch(pipelineIndex);
        return this;
    }

    @Override
    public void periodic() {
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);

        LLResult result = limelight.getLatestResult();
        llStatus = limelight.getStatus();

        if (result == null || !result.isValid()) {
            currentState = CameraState.SEARCHING;
            results.hasTarget = false;
            return;
        }

        if (result.getPipelineIndex() != desiredPipeline) {
            currentState = CameraState.IDLE;
            return;
        }

        updateResults(result);
    }

    public Botpose getBotpose() {
        Botpose botpose = new Botpose();

        LLResult result = results.result;

        if (result == null || !result.isValid()) {
            return botpose;
        }

        Pose3D pose = result.getBotpose();
        if (pose == null) {
            return botpose;
        }

        botpose.x = pose.getPosition().x;
        botpose.y = pose.getPosition().y;
        botpose.result = result;

        return botpose;
    }


    public void updateResults(LLResult result) {
        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            currentState = CameraState.SEARCHING;
            results.hasTarget = false;
            return;
        }

        FiducialResult validFiducial = null;

        // Use the best / closest fiducial
        for (FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                validFiducial = fiducial;
                break;
            }
        }

        if (validFiducial == null) {
            return;
        }

        results.motifID = validFiducial.getFiducialId();
        Pose3D robotToTag = validFiducial.getRobotPoseTargetSpace();

        Translation2d poseToTag = new Translation2d(
                robotToTag.getPosition().x,
                robotToTag.getPosition().y
        );

        // distance to tag
        results.distanceMeters = poseToTag.getNorm();

        // update auxiliary data
        results.result = result;
        results.hasTarget = true;
        currentState = CameraState.TARGETING;

        // yaw error to tag
        results.tx = validFiducial.getTargetXDegrees();
        results.ty = validFiducial.getTargetYDegrees();
        results.ta = validFiducial.getTargetArea();
    }

    public int getMotifID() {
        return results.hasTarget ? results.motifID : -1;
    }

    @Override
    public void updateTelemetry(Telemetry telemetry, TelemetryPacket packet) {
        telemetry.addLine(); // Separator from other data
        telemetry.addData(getName() + " State", currentState);

        if (llStatus != null) {
            telemetry.addData(getName() + " Pipeline", "%d", llStatus.getPipelineIndex());
            telemetry.addData(getName() + " FPS", "%.0f", llStatus.getFps());
            telemetry.addData(getName() + " RAM", "%.1f", llStatus.getRam());
            telemetry.addData(getName() + " CPU", "%.1f", llStatus.getCpu());
            telemetry.addData(getName() + " TEMP", "%.1f", llStatus.getTemp());
        }

        Botpose botpose = getBotpose();

        if (botpose.result != null) {
            telemetry.addData(getName() + "Pose X", "%.1f", botpose.x);
            telemetry.addData(getName() + "Pose Y", "%.1f", botpose.y);
            long staleness = botpose.result.getStaleness();
            if (staleness < 30) {
                telemetry.addData(getName() + " Pose Staleness", "FRESH " + staleness + "ms");
            } else {
                telemetry.addData(getName() + " Pose Staleness", "STALE " + staleness + "ms");
            }
        }

        if (results.hasTarget) {
            telemetry.addData(getName() + " Tag Distance (m)", "%.2f", results.distanceMeters);
            telemetry.addData(getName() + " Tag Distance (in)", "%.1f", results.distanceMeters * 39.37);
            telemetry.addData(getName() + " tx", "%.2f", results.tx);
            telemetry.addData(getName() + " ty", "%.2f", results.ty);
            telemetry.addData(getName() + " ta", "%.2f", results.ta);
            long staleness = botpose.result.getStaleness();
            if (staleness < 30) {
                telemetry.addData(getName() + " Tag Staleness", "FRESH " + staleness + "ms");
            } else {
                telemetry.addData(getName() + " Tag Staleness", "STALE " + staleness + "ms");
            }
            telemetry.addData(getName() + " Healthy", isHealthy());
        } else {
            telemetry.addLine("No AprilTags detected");
        }
    }

    @Override
    public boolean isHealthy() {
        return limelight.isConnected() && limelight.isRunning();
    }

    @Override
    public void stop() {
        limelight.shutdown();
        currentState = CameraState.STOPPED;
    }
}
