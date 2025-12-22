package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotContainer {
    public final DriverControls controls;
    public final Robot robot;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driverGamepad) {
        controls = new DriverControls(driverGamepad);
        robot = new Robot(hardwareMap, controls);

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
        /*
         * Call periodic methods of all subsystems
         *
         * Order of calls:
         * 1 - Vision to get latest data
         * 2 - Drive to update pose
         * 3 - Shooter to adjust to new target if needed
         * 4 - Intake to respond to driver inputs
         * 5 - LED to update status
         */
        robot.vision.periodic();
        robot.drive.periodic();
        robot.shooter.periodic();
        robot.intake.periodic();
        robot.led.periodic();
        robot.robotState.periodic();

        /* Update telemetry with robot status */

        // Robot state
        telemetry.addData("Robot State", robot.robotState.toString());

        // Pose information
        telemetry.addData("Pose X", robot.drive.getPose().position.x);
        telemetry.addData("Pose Y", robot.drive.getPose().position.y);
        telemetry.addData("Pose Heading", robot.drive.getPose().heading.real);

        // Vision data
        telemetry.addData("Distance", robot.vision.distance);
        telemetry.addData("Bearing", robot.vision.bearing);

        // Shooter data
        robot.shooter.updateTelemetry(telemetry);
    }
}
