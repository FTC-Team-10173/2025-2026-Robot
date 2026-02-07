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
    private LLResult result;

    public Limelight(HardwareMap hardwareMap) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.imu = hardwareMap.get(IMU.class, "imu");

        limelight.setPollRateHz(100);
        limelight.start();
        setPipeline(pipelineIndex);
    }

    @Override
    public void periodic() {
        // update robotYaw for MT2
        // TODO: Meta Tag 2 is not being used currently because yaw is not configured correctly
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        if (result.getPipelineIndex() != pipelineIndex) {
            return;
        }

        this.result = result;
    }

    public void setPipeline(int index) {
        this.pipelineIndex = index;
        limelight.pipelineSwitch(index);
    }

    public int getPipeline() {
        return pipelineIndex;
    }

    public LLResult getResults() {
        return result;
    }

    public boolean isHealthy() {
        return limelight.isConnected() && limelight.isRunning();
    }

    public void stop() {
        limelight.shutdown();
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            telemetry.addData(getName() + " X", botpose.getPosition().x * 39.37);
            telemetry.addData(getName() + " Y", botpose.getPosition().y * 39.37);
        }

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
        }
    }
}