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
    public VoltageSensor voltageSensor;
    private int loopCount = 0;
    private final long startTime;
    Logger logger;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driverGamepad) {
        controls = new DriverControls(driverGamepad);
        robotState = new RobotState(controls);
        robot = new Robot(hardwareMap, robotState, controls);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        String matchName = "TeleOp_" + System.currentTimeMillis();

        startTime = System.currentTimeMillis();

        logger = new Logger(matchName);
    }

    public void periodic(Telemetry telemetry) {
        controls.driver.readButtons();

        robot.periodicAll();
        robotState.periodic();

        if (loopCount++ % 10 == 0) {
            updateTelemetry(telemetry);
        }
    }

    private void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("State", robotState.toString());
        logger.put("Robot State", robotState.toString());

        logger.put("Voltage", voltageSensor.getVoltage());

        if (!robot.areAllSubsystemsHealthy()) {
            telemetry.addData("Subsystem Issues", robot.getUnhealthySubsystems());
            logger.put("Subsystem Issues", robot.getUnhealthySubsystems());
        }

        robot.telemetryUpdateAll(telemetry, logger);
    }

    public void onStop() {
        robot.stopAll();
        robotState.set(RobotState.State.IDLE);
        logger.save();
    }
}
