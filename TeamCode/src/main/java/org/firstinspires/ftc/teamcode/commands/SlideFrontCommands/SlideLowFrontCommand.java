package org.firstinspires.ftc.teamcode.commands.SlideFrontCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideLowFrontCommand extends SequentialCommandGroup {
    public SlideLowFrontCommand(Slide slide, ClawMotors clawMotors, ClawServos clawServos, boolean FLIP) {
        addCommands(
                new InstantCommand(slide::slideLow, slide),
                new InstantCommand(() -> clawMotors.moveLowF(FLIP)),
                new WaitCommand(100),
                new InstantCommand(()-> clawServos.setClaw3Pos(FLIP))
        );
    }
}
