package org.firstinspires.ftc.teamcode.commands.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideMidBackCommand extends SequentialCommandGroup {
    public SlideMidBackCommand(Slide slide, ClawMotors clawMotors) {
        addCommands(
                new InstantCommand(slide::slideMid, slide),
                new InstantCommand(clawMotors::moveClawMidBack, clawMotors)
        );
    }
}