package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
        // TODO: set up button bindings if needed
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
