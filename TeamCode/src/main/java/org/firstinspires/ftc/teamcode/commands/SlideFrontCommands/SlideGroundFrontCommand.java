package org.firstinspires.ftc.teamcode.commands.SlideFrontCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideGroundFrontCommand extends SequentialCommandGroup {
    public SlideGroundFrontCommand(Slide slide, ClawMotors clawMotors, ClawServos clawServos, boolean FLIP) {
        addCommands(
                new InstantCommand(()-> clawServos.setClaw3Pos(FLIP)),
                new WaitCommand(100),
                new InstantCommand(slide::slideGround, slide),
                new InstantCommand(() -> clawMotors.moveGroundF(FLIP))
        );
    }
}
