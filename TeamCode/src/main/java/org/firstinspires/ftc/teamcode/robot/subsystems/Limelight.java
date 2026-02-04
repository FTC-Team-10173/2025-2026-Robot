package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Logger;

import java.util.List;

public class Limelight extends SubsystemBase {
    private final Limelight3A limelight;
    private final IMU imu;

    private int pipelineIndex = 0;
    private final Results results = new Results();

    public Limelight(HardwareMap hardwareMap) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.imu = hardwareMap.get(IMU.class, "imu");

        limelight.setPollRateHz(100);
        limelight.start();
        setPipeline(pipelineIndex);
    }

    @Override
    public void periodic() {
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            results.hasTarget = false;
            return;
        }

        if (result.getPipelineIndex() != pipelineIndex) {
            return;
        }

        updateResults(result);
    }

    public void setPipeline(int index) {
        this.pipelineIndex = index;
        limelight.pipelineSwitch(index);
    }

    public int getPipeline() {
        return pipelineIndex;
    }

    private void updateResults(LLResult result) {
        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            results.hasTarget = false;
            return;
        }

        FiducialResult bestFiducial = null;
        for (FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                bestFiducial = fiducial;
                break;
            }
        }

        if (bestFiducial == null) {
            results.hasTarget = false;
            return;
        }

        results.motifID = bestFiducial.getFiducialId();
        Pose3D robotToTag = bestFiducial.getRobotPoseTargetSpace();

        Translation2d poseToTag = new Translation2d(
                robotToTag.getPosition().x,
                robotToTag.getPosition().y
        );

        results.distanceMeters = poseToTag.getNorm();
        results.result = result;
        results.hasTarget = true;
        results.tx = bestFiducial.getTargetXDegrees();
        results.ty = bestFiducial.getTargetYDegrees();
        results.ta = bestFiducial.getTargetArea();
    }

    public Results getResults() {
        return results;
    }

    public int getMotifID() {
        return results.hasTarget ? results.motifID : -1;
    }

    public double getTargetDistance() {
        return results.distanceMeters * 39.37;
    }

    public boolean isHealthy() {
        return limelight.isConnected() && limelight.isRunning();
    }

    public void stop() {
        limelight.shutdown();
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());
        telemetry.addData(getName() + " Has Target", results.hasTarget);
        telemetry.addData(getName() + " Distance", results.distanceMeters);
        telemetry.addData(getName() + " Distance (Inch)", results.distanceMeters * 39.37);
        telemetry.addData(getName() + " TX", results.tx);

        LLResult result = results.result;

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            telemetry.addData(getName() + " X", botpose.getPosition().x * 39.37);
            telemetry.addData(getName() + " Y", botpose.getPosition().y * 39.37);
        }

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
            logger.put(getName() + " Has Target", results.hasTarget);
            logger.put(getName() + " Distance", results.distanceMeters);
            logger.put(getName() + " Distance (Inch)", results.distanceMeters * 39.37);
            logger.put(getName() + " TX", results.tx);
        }
    }

    public static class Results {
        public boolean hasTarget = false;
        public double distanceMeters = 0;
        public double tx = 0;
        public double ty = 0;
        public double ta = 0;
        public int motifID = -1;
        public LLResult result = null;
    }
}