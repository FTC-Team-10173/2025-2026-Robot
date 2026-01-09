package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.Limelight;

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

    // TeleOp constructor
    public Robot(HardwareMap hardwareMap, RobotState robotState, DriverControls controls) {
        // initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        led = new LED(hardwareMap, robotState);
        limelight = new Limelight(hardwareMap, imu).setPipeline(0);
        shooter = new Shooter(hardwareMap, controls, robotState, limelight);
        intake = new Intake(hardwareMap, controls);
        drive = new Drive(hardwareMap, controls, limelight, imu);

        // Preferable telemetry order
        allSubsystems = new ArrayList<>();
        allSubsystems.add(drive);
        allSubsystems.add(shooter);
        allSubsystems.add(limelight);
        allSubsystems.add(intake);
        allSubsystems.add(led);
    }

    // Autonomous constructor
    public Robot(HardwareMap hardwareMap, RobotState robotState) {
        // initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        led = new LED(hardwareMap, robotState);
        limelight = new Limelight(hardwareMap, imu).setPipeline(0);
        shooter = new Shooter(hardwareMap, robotState, limelight);
        intake = new Intake(hardwareMap);
        drive = new Drive(hardwareMap, limelight, imu);

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

    public void telemetryUpdateAll(Telemetry telemetry) {
        for (Subsystem subsystem : allSubsystems) {
            subsystem.updateTelemetry(telemetry);
        }
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
}
