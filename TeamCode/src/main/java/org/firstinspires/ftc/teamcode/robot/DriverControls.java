package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class DriverControls {
    public final GamepadEx driver;

    public DriverControls(GamepadEx driver) {
        this.driver = driver;
    }

    /**
     * Lock heading to goal command
     */
    public boolean lockDrivePressed() {
        return driver.getButton(GamepadKeys.Button.A);
    }

    /**
     * Half intake command
     */
    public double intakePower() {
        return driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    /**
     * Full intake command
     */
    public boolean fullIntakePressed() {
        return driver.getButton(GamepadKeys.Button.RIGHT_BUMPER);
    }

    /**
     * Top outtake command
     */
    public boolean topOuttakePressed() {
        return driver.getButton(GamepadKeys.Button.B);
    }

    /**
     * Spin shooter command
     */
    public boolean spinShooterPressed() {
        return driver.getButton(GamepadKeys.Button.LEFT_BUMPER);
    }
}
