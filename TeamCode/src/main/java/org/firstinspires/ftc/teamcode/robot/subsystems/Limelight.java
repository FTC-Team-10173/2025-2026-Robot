package org.firstinspires.ftc.teamcode.robot.subsystems;

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
    public final Botpose botpose;

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
    public static class Botpose { // TODO: use botpose for localization verification
        public double x;
        public double y;
        public LLResult result; // TODO: use getStaleness() method in localization verification
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
        botpose = new Botpose();
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

        updateBotpose(result);
        updateResults(result);
    }

    public void updateBotpose(LLResult result) {
        Pose3D botpose_mt2 = result.getBotpose_MT2();
        if (botpose_mt2 != null) {
            botpose.x = botpose_mt2.getPosition().x;
            botpose.y = botpose_mt2.getPosition().y;

            botpose.result = result;
        }
    }

    public void updateResults(LLResult result) {
        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            currentState = CameraState.SEARCHING;
            results.hasTarget = false;
            return;
        }

        for (FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            if (id == 21 || id == 22 || id == 23) {
                results.motifID = id;
                break;
            }
        }

        // Use the best / closest fiducial
        FiducialResult fiducial = fiducials.get(0);
        Pose3D robotToTag = fiducial.getRobotPoseTargetSpace();

        double x = robotToTag.getPosition().x;
        double y = robotToTag.getPosition().y;
        double z = robotToTag.getPosition().z;

        // distance to tag
        results.distanceMeters = Math.sqrt(x * x + y * y + z * z);

        // update auxiliary data
        results.result = result;
        results.hasTarget = true;
        currentState = CameraState.TARGETING;

        // yaw error to tag
        results.tx = fiducial.getTargetXDegrees();
        results.ty = fiducial.getTargetYDegrees();
        results.ta = fiducial.getTargetArea();
    }

    public int getMotifID() {
        return results.hasTarget ? results.motifID : -1;
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine(); // Separator from other data
        telemetry.addData(getName() + " State", currentState);

        if (llStatus != null) {
            telemetry.addData(getName() + " Pipeline", "%d", llStatus.getPipelineIndex());
            telemetry.addData(getName() + " FPS", "%.0f", llStatus.getFps());
            telemetry.addData(getName() + " RAM", "%.1f", llStatus.getRam());
            telemetry.addData(getName() + " CPU", "%.1f", llStatus.getCpu());
            telemetry.addData(getName() + " TEMP", "%.1f", llStatus.getTemp());
        }

        if (botpose.result != null) {
            telemetry.addData(getName() + "Pose X", "%.1f", botpose.x);
            telemetry.addData(getName() + "Pose Y", "%.1f", botpose.y);
            long staleness = botpose.result.getStaleness();
            if (staleness < 100) {
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
            if (staleness < 100) {
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
