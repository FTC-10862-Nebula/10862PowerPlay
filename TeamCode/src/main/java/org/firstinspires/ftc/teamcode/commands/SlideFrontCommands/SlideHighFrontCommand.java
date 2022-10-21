package org.firstinspires.ftc.teamcode.commands.SlideFrontCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideHighFrontCommand extends SequentialCommandGroup {
    public SlideHighFrontCommand(Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                new InstantCommand(slide::slideHigh, slide),
                new InstantCommand(clawMotors::moveClawHighFront, clawMotors),
                new InstantCommand(clawServos::setFClawPos)
        );
    }   
}
