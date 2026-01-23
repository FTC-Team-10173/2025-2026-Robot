package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;

import java.util.function.Supplier;

public class LEDCommand extends CommandBase {
    private final LED led;
    private final Supplier<RobotState.State> stateSupplier;

    public LEDCommand(LED led, Supplier<RobotState.State> stateSupplier) {
        this.led = led;
        this.stateSupplier = stateSupplier;

        addRequirements(led);
    }

    @Override
    public void execute() {
        RobotState.State currentState = stateSupplier.get();
        led.setRobotState(currentState);
    }

    @Override
    public void end(boolean interrupted) {
        // Set to idle state when ended
        led.setRobotState(RobotState.State.IDLE);
    }
}