package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotContainer {
    public final DriverControls controls;
    public final Robot robot;
    public RobotState robotState;

    private int loopCount = 0;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driverGamepad) {
        controls = new DriverControls(driverGamepad);
        robotState = new RobotState(controls);
        robot = new Robot(hardwareMap, robotState, controls);

        configureBindings();
    }

    /**
     * Configure button bindings for driver controls
     */
    private void configureBindings() {
    }

    /**
     * Periodic method to be called in main op mode loop
     */
    public void periodic(Telemetry telemetry) {
        controls.driver.readButtons();

        robot.periodicAll();
        robotState.periodic();

        if (loopCount++ % 10 == 0) {
            updateTelemetry(telemetry);
        }
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
    }
}
