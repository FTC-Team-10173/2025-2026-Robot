package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotContainer {
    public final DriverControls controls;
    public final Robot robot;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driverGamepad) {
        controls = new DriverControls(driverGamepad);
        robot = new Robot(hardwareMap, controls);

        configureBindings();
    }

    private void configureBindings() {
        // Reset robot yaw when BACK button is pressed
        controls.driver.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(robot.drive::resetYaw);
    }

    public void periodic() {
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
    }
}
