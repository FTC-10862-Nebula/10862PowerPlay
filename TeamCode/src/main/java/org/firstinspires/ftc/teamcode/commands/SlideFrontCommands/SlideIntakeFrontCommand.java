package org.firstinspires.ftc.teamcode.commands.SlideFrontCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideIntakeFrontCommand extends SequentialCommandGroup {
    public SlideIntakeFrontCommand(Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                new InstantCommand(clawServos::setFClawPos),
                new InstantCommand(slide::slideResting, slide),
                new WaitCommand(150),
                new InstantCommand(clawMotors::moveIntakeF, clawMotors)
        );
    }
}
