package org.firstinspires.ftc.teamcode.robot.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

public class LockTurretCommand extends InstantCommand {
    public LockTurretCommand(Turret turret) {
        super(
                turret::lockTurret
        );
    }
}