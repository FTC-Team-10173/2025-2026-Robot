package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.robot.subsystems.TimedSubsystem;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    public final Drive drive;
    public final Shooter shooter;
    public final Intake intake;
    public final LED led;
    public final Limelight limelight;
    private final List<Subsystem> allSubsystems;
    private final IMU imu;
    private final FtcDashboard dashboard;
    private final TelemetryPacket packet;

    // TeleOp constructor
    public Robot(HardwareMap hardwareMap, RobotState robotState, DriverControls controls) {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        // initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        led = new LED(hardwareMap, robotState);
        limelight = new Limelight(hardwareMap, imu).setPipeline(0);
        shooter = new Shooter(hardwareMap, controls, robotState, limelight);
        intake = new Intake(hardwareMap, controls);
        drive = new Drive(hardwareMap, controls, limelight, imu);

        // Preferable telemetry order
        allSubsystems = new ArrayList<>();
        allSubsystems.add(new TimedSubsystem(drive));
        allSubsystems.add(new TimedSubsystem(shooter));
        allSubsystems.add(new TimedSubsystem(limelight));
        allSubsystems.add(new TimedSubsystem(intake));
        allSubsystems.add(new TimedSubsystem(led));
    }

    // Autonomous constructor
    public Robot(HardwareMap hardwareMap, RobotState robotState, Pose2d startPose) {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        // initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        led = new LED(hardwareMap, robotState);
        limelight = new Limelight(hardwareMap, imu).setPipeline(0);
        shooter = new Shooter(hardwareMap, robotState, limelight);
        intake = new Intake(hardwareMap);
        drive = new Drive(hardwareMap, limelight, imu, startPose);

        // Preferable telemetry order
        allSubsystems = new ArrayList<>();
        allSubsystems.add(drive);
        allSubsystems.add(shooter);
        allSubsystems.add(limelight);
        allSubsystems.add(intake);
        allSubsystems.add(led);
    }

    public void periodicAll() {
        for (Subsystem subsystem : allSubsystems) {
            subsystem.periodic();
        }
    }

    public void stopAll() {
        for (Subsystem subsystem : allSubsystems) {
            subsystem.stop();
        }
    }

    public void telemetryUpdateAll(Telemetry telemetry, Logger logger) {
        for (Subsystem subsystem : allSubsystems) {
            subsystem.updateTelemetry(telemetry, packet, logger);
        }
        dashboard.sendTelemetryPacket(packet);
    }

    public boolean areAllSubsystemsHealthy() {
        for (Subsystem subsystem : allSubsystems) {
            if (!subsystem.isHealthy()) {
                return false;
            }
        }
        return true;
    }

    public List<Subsystem> getHealthySubsystems() {
        List<Subsystem> healthySubsystems = new ArrayList<>();
        for (Subsystem subsystem : allSubsystems) {
            if (subsystem.isHealthy()) {
                healthySubsystems.add(subsystem);
            }
        }
        return healthySubsystems;
    }

    public List<Subsystem> getUnhealthySubsystems() {
        List<Subsystem> unhealthySubsystems = new ArrayList<>();
        for (Subsystem subsystem : allSubsystems) {
            if (!subsystem.isHealthy()) {
                unhealthySubsystems.add(subsystem);
            }
        }
        return unhealthySubsystems;
    }

    public static class SubsystemStatus {
        public String name;
        public boolean isHealthy;
        public double lastPeriodicTime;

        public SubsystemStatus(String name, boolean isHealthy, double lastPeriodicTime) {
            this.name = name;
            this.isHealthy = isHealthy;
            this.lastPeriodicTime = lastPeriodicTime;
        }
    }

    public List<SubsystemStatus> getSubsystemStatuses() {
        List<SubsystemStatus> statuses = new ArrayList<>();
        for (Subsystem subsystem : allSubsystems) {
            double lastTime = 0;
            if (subsystem instanceof TimedSubsystem) {
                lastTime = ((TimedSubsystem) subsystem).getLastPeriodicTime();
            }
            statuses.add(new SubsystemStatus(subsystem.getName(), subsystem.isHealthy(), lastTime));
        }
        return statuses;
    }
}
