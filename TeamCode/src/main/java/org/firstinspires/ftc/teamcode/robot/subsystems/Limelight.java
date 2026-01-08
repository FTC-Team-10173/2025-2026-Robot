package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight implements Subsystem {

    private final Limelight3A limelight;
    private final IMU imu; // Currently unused but may incorporate for more accurate localization

    private CameraState currentState;
    private int desiredPipeline;

    private LLResult result;

    public final Results results;

    public enum CameraState {
        IDLE,
        STOPPED
    }

    /** Snapshot of vision outputs */
    public static class Results {
        public boolean hasTarget;
        public double distanceMeters;
        public double tx;
        public double ty;
        public double ta;
        public long stalenessMillis;
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
        result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            results.hasTarget = false;
            return;
        }

        if (result.getPipelineIndex() != desiredPipeline) return;

        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            results.hasTarget = false;
            return;
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
        results.stalenessMillis = result.getStaleness();
        results.hasTarget = true;

        // yaw error to tag
        results.tx = fiducial.getTargetXDegrees();
        results.ty = fiducial.getTargetYDegrees();
        results.ta = fiducial.getTargetArea();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine(); // Separator from other data
        telemetry.addData(getName() + " State", currentState);

        if (results.hasTarget) {
            telemetry.addData(getName() + " Tag Distance (m)", "%.2f", results.distanceMeters);
            telemetry.addData(getName() + " Tag Distance (in)", "%.1f", results.distanceMeters * 39.37);
            telemetry.addData(getName() + " tx", "%.2f", results.tx);
            telemetry.addData(getName() + " ty", "%.2f", results.ty);
            telemetry.addData(getName() + " ta", "%.2f", results.ta);
            if (results.stalenessMillis < 100) {
                telemetry.addData(getName() + " Staleness", "FRESH");
            } else {
                telemetry.addData(getName() + " Staleness", "STALE");
            }
            telemetry.addData(getName() + " Healthy", isHealthy());
        } else {
            telemetry.addLine("No AprilTags detected");
        }
    }

    @Override
    public boolean isHealthy() {
        return limelight != null && result != null && result.isValid();
    }

    @Override
    public void stop() {
        limelight.shutdown();
        currentState = CameraState.STOPPED;
    }
}
