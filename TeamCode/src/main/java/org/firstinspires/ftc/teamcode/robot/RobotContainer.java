package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotContainer {
    public final DriverControls controls;
    public final Robot robot;
    public RobotState robotState;
    public VoltageSensor voltageSensor;
    private int loopCount = 0;
    Logger logger;
    Telemetry telemetry;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driverGamepad, Telemetry telemetry) {
        controls = new DriverControls(driverGamepad);
        robotState = new RobotState(controls);
        robot = new Robot(hardwareMap, robotState, controls);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        String matchName = "TeleOp_" + System.currentTimeMillis();

        logger = new Logger(matchName);

        this.telemetry = telemetry;
    }

    public void periodic() {
        controls.driver.readButtons();

        robot.periodicAll();
        robotState.periodic();

        if (loopCount++ % 10 == 0) {
            updateTelemetry(telemetry);
        }
    }

    private void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("State", robotState.toString();
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
