package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotContainer {
    public final DriverControls controls;
    public final Robot robot;
    public RobotState robotState;

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
        // Reset robot yaw when BACK button is pressed
        controls.driver.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(robot.drive::resetYaw);
    }

    /**
     * Periodic method to be called in main op mode loop
     */
    public void periodic(Telemetry telemetry) {
        robot.periodicAll();

        robotState.periodic();

        updateTelemetry(telemetry);
    }

    private void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("State", robotState.get());

        if (!robot.areAllSubsystemsHealthy()) {
            telemetry.addData("Subsystem Issues", robot.getUnhealthySubsystems());
        }

        telemetry.addLine();

        robot.telemetryUpdateAll(telemetry);
    }
}
