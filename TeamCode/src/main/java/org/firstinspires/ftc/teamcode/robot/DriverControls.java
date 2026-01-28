// robot/DriverControls.java (fixed version)
package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Trigger;

public class DriverControls {
    public final GamepadEx driver;

    // Buttons
    public final GamepadButton shootButton;       // Left Bumper for shooting
    public final GamepadButton fullIntakeButton;  // Right Bumper for full intake
    public final GamepadButton yawResetButton;    // Y button for yaw reset
    public final GamepadButton lockDriveButton;   // X button for heading lock

    public final GamepadButton upButton;
    public final GamepadButton downButton;

    // Triggers
    public final Trigger shootTrigger;
    public final Trigger intakeTrigger;
    public final Trigger fullIntakeTrigger;
    public final Trigger outtakeTrigger;
    public final Trigger yawResetTrigger;
    public final Trigger lockDriveTrigger;
    public final Trigger upPower;
    public final Trigger downPower;

    public DriverControls(GamepadEx driver) {
        this.driver = driver;

        // Initialize buttons
        shootButton = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        fullIntakeButton = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        yawResetButton = driver.getGamepadButton(GamepadKeys.Button.BACK);
        lockDriveButton = driver.getGamepadButton(GamepadKeys.Button.A);

        upButton = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        downButton = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);

        // Create triggers
        shootTrigger = new Trigger(shootButton::get);
        fullIntakeTrigger = new Trigger(fullIntakeButton::get);
        intakeTrigger = new Trigger(() -> getRightTrigger() > 0.1);
        outtakeTrigger = new Trigger(() -> getLeftTrigger() > 0.1);
        yawResetTrigger = new Trigger(yawResetButton::get);
        lockDriveTrigger = new Trigger(lockDriveButton::get);

        upPower = new Trigger(upButton::get);
        downPower = new Trigger(downButton::get);
    }

    public double getRightTrigger() {
        return driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public double getLeftTrigger() {
        return driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    public void readButtons() {
        driver.readButtons();
    }
}