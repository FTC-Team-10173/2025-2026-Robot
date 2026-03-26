package org.firstinspires.ftc.teamcode.robot;

import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

public class DriverControls {
    public final GamepadEx driver;

    // Buttons
    public final GamepadButton shootButton; // Left Bumper for shooting
    public final GamepadButton feedButton; // Right Bumper for full intake
    public final GamepadButton yawResetButton; // Back button for yaw reset
    public final GamepadButton lockTurretButton; // Y button to lock turret

    // Triggers
    public final Trigger shootTrigger;
    public final Trigger intakeTrigger;
    public final Trigger feedTrigger;
    public final Trigger outtakeTrigger;
    public final Trigger yawResetTrigger;
    public final Trigger lockTurretTrigger;

    public DriverControls(GamepadEx driver) {
        this.driver = driver;

        // Initialize buttons
        shootButton = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
        feedButton = driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
        yawResetButton = driver.getGamepadButton(GamepadKeys.Button.BACK);
        lockTurretButton = driver.getGamepadButton(GamepadKeys.Button.Y);

        // Create triggers
        shootTrigger = new Trigger(shootButton::get);
        intakeTrigger = new Trigger(this::intakeTrigger);
        feedTrigger = new Trigger(feedButton::get);
        outtakeTrigger = new Trigger(this::outtakeTrigger);
        yawResetTrigger = new Trigger(yawResetButton::get);
        lockTurretTrigger = new Trigger(lockTurretButton::get);
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

    public boolean intakeTrigger() {
        return getRightTrigger() > 0.1;
    }

    public void readButtons() {
        driver.readButtons();
    }
}