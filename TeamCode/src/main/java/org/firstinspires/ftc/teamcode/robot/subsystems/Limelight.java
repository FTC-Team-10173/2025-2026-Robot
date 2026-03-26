package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Logger;

import java.util.List;

public class Limelight extends SubsystemBase {
    private final Limelight3A limelight;
    private double heading = 0;

    private int pipelineIndex = 0;
    private LLResult result;

    public Limelight(HardwareMap hardwareMap) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.start();
        setPipeline(pipelineIndex);
    }

    @Override
    public void periodic() {
        // update robotYaw for MT2
        // TODO: Meta Tag 2 is not being used currently because yaw is not configured correctly
        limelight.updateRobotOrientation(heading);

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        if (result.getPipelineIndex() != pipelineIndex) {
            return;
        }

        this.result = result;
    }

    public void updateHeading(double heading, double turretSupplier) {
        this.heading = Math.toDegrees(heading) + turretSupplier;
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
        telemetry.addData(getName() + " Heading", heading);

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();

            telemetry.addData(getName() + " X", botpose.getPosition().x * 39.37);
            telemetry.addData(getName() + " Y", botpose.getPosition().y * 39.37);
        }

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
        }
    }
}