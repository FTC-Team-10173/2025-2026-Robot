package org.firstinspires.ftc.teamcode.robot.commands;

import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;

import java.util.*;

public class Scheduler {

    private final List<Command> activeCommands = new ArrayList<>();
    private final Map<Subsystem, Command> requirements = new HashMap<>();

    public void schedule(Command cmd) {
        for (Subsystem req : cmd.getRequirements()) {
            if (requirements.containsKey(req)) {
                Command conflicting = requirements.get(req);
                assert conflicting != null;
                interrupt(conflicting);
            }
        }

        for (Subsystem req : cmd.getRequirements()) {
            requirements.put(req, cmd);
        }

        cmd.initialize();
        activeCommands.add(cmd);
    }

    private void interrupt(Command cmd) {
        cmd.end(true);
        activeCommands.remove(cmd);

        // Free its subsystems
        for (Subsystem req : cmd.getRequirements()) {
            requirements.remove(req);
        }
    }


    public void run() {
        for (Command cmd : activeCommands) {
            cmd.execute();

            if (cmd.isFinished()) {
                cmd.end(false);
                activeCommands.remove(cmd);
            }
        }
    }
}
