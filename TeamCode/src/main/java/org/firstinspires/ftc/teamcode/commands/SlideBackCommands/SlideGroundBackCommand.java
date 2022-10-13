package org.firstinspires.ftc.teamcode.commands.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideGroundBackCommand extends SequentialCommandGroup {
    public SlideGroundBackCommand(Slide slide, ClawMotors clawMotors) {
        addCommands(
                new InstantCommand(slide::slideGround, slide),
                new InstantCommand(clawMotors::moveClawGroundBack, clawMotors)
        );
    }
}
