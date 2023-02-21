package org.firstinspires.ftc.teamcode.commands.driveCommands.autoCommands.trajectoryCommands;

import com.arcrobotics.ftclib.command.Command;

public class DisplacementCommand extends MarkerCommand {
    public final double displacement;
    public DisplacementCommand(double displacement, Command command) {
        super(command);
        this.displacement = displacement;
    }
}