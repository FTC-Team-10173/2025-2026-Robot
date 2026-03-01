package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.button.*;
import com.arcrobotics.ftclib.gamepad.*;

public class DriverControls {
    public final GamepadEx driver;

    // Buttons
    public final GamepadButton shootButton;       // Left Bumper for shooting
    public final GamepadButton intakeButton;  // Right Bumper for full intake
    public final GamepadButton yawResetButton;    // Y button for yaw reset

    public final GamepadButton upTest;
    public final GamepadButton downTest;

    // Triggers
    public final Trigger shootTrigger;
    public final Trigger intakeTrigger;
    public final Trigger outtakeTrigger;
    public final Trigger yawResetTrigger;

    public final Trigger upTrigger;
    public final Trigger downTrigger;

    public DriverControls(GamepadEx driver) {
        this.driver = driver;

        // Initialize buttons
        shootButton = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        intakeButton = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        yawResetButton = driver.getGamepadButton(GamepadKeys.Button.BACK);

        upTest = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        downTest = driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);

        // Create triggers
        shootTrigger = new Trigger(shootButton::get);
        intakeTrigger = new Trigger(intakeButton::get);
        outtakeTrigger = new Trigger(this::outtakeTrigger);
        yawResetTrigger = new Trigger(yawResetButton::get);

        upTrigger = new Trigger(upTest::get);
        downTrigger = new Trigger(downTest::get);
    }

    public double getRightTrigger() {
        return driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public double getLeftTrigger() {
        return driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    public boolean outtakeTrigger() {
        return getRightTrigger() > 0.1 && getLeftTrigger() > 0.1;
    }

    public void readButtons() {
        driver.readButtons();
    }
}