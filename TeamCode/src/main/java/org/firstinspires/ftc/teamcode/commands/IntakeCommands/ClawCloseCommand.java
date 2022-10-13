package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class ClawCloseCommand extends SequentialCommandGroup {

    public ClawCloseCommand(ClawServos clawServos) {
        addCommands(
                new InstantCommand(clawServos::clawClose)
        );
    }
}