package org.firstinspires.ftc.teamcode.commands.SlideFrontCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideGroundFrontCommand extends SequentialCommandGroup {
    public SlideGroundFrontCommand(Slide slide, ClawMotors clawMotors) {
        addCommands(
                new InstantCommand(slide::slideGround, slide),
                new InstantCommand(clawMotors::moveClawGroundFront, clawMotors)
        );
    }
}
