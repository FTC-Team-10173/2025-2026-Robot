package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class RobotContainer {
    public final DriverControls controls;
    public final Robot robot;
    public RobotState robotState;
    public final Logger logger;
    public VoltageSensor voltageSensor;

    private int loopCount = 0;
    private final long startTime;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driverGamepad) {
        controls = new DriverControls(driverGamepad);
        robotState = new RobotState(controls);
        robot = new Robot(hardwareMap, robotState, controls);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        String matchName = "TeleOp_" + System.currentTimeMillis();
        logger = new Logger(matchName);

        startTime = System.currentTimeMillis();
    }

    public void periodic(Telemetry telemetry) {
        controls.driver.readButtons();

        robot.periodicAll();
        robotState.periodic();

        if (loopCount++ % 10 == 0) {
            updateTelemetry(telemetry);
        }

        logData();
    }

    private void logData() {
        Logger.DataPoint point = new Logger.DataPoint(
                (System.currentTimeMillis() - startTime)
        );

        point.shooterVelocity = robot.shooter.getVelocity();
        point.shooterTarget = robot.shooter.targetVel;
        point.gatePosition = robot.intake.getGateAngle();
        point.targetDistance = robot.limelight.results.distanceMeters * 39.37;
        Pose2d pose = robot.drive.getPose();
        point.posX = pose.position.x;
        point.posY = pose.position.y;
        point.heading = pose.heading.toDouble();

        Logger.loopTime loopTime = new Logger.loopTime();
        List<Robot.SubsystemStatus> statuses = robot.getSubsystemStatuses();
        for (Robot.SubsystemStatus status : statuses) {
            switch (status.name) {
                case "Drive":
                    loopTime.drive = status.lastPeriodicTime;
                    break;
                case "Shooter":
                    loopTime.shooter = status.lastPeriodicTime;
                    break;
                case "Intake":
                    loopTime.intake = status.lastPeriodicTime;
                    break;
                case "LED":
                    loopTime.led = status.lastPeriodicTime;
                    break;
                case "Limelight":
                    loopTime.limelight = status.lastPeriodicTime;
                    break;
            }
        }
        point.loopTime = loopTime;

        point.robotState = robotState.get().toString();
        point.voltage = voltageSensor.getVoltage();

        logger.log(point);
    }

    private void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("State", robotState.get());

        if (!robot.areAllSubsystemsHealthy()) {
            telemetry.addData("Subsystem Issues", robot.getUnhealthySubsystems());
        }

        robot.telemetryUpdateAll(telemetry);
    }

    public void onStop() {
        robot.stopAll();
        robotState.set(RobotState.State.IDLE);
        logger.save();
    }
}
