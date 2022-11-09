package org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetFrontCommandT extends SequentialCommandGroup {
    public SlideResetFrontCommandT(Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(

                new InstantCommand(clawServos::clawClose, clawServos),
                new WaitCommand(400),
                new InstantCommand(clawMotors::moveIntakeF, clawMotors),
                new InstantCommand(slide::slideResting, slide),
                new InstantCommand(clawServos::setFClawPos),
                new WaitCommand(200),
                new InstantCommand(clawServos::clawOpen)
        );
    }
}
