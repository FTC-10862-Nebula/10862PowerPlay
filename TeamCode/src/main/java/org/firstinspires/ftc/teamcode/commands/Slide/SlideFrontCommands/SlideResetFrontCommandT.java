package org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetFrontCommandT extends SequentialCommandGroup {
    public SlideResetFrontCommandT(Slide slide, Arm arm, ClawServos clawServos){
        addRequirements(arm);
        addCommands(
                new WaitCommand(100),
                new InstantCommand(clawServos::clawClose, clawServos),
                new WaitCommand(200),
                new InstantCommand(arm::moveIntakeF, arm),
                new WaitCommand(100),
                new InstantCommand(slide::slideResting, slide),
                new InstantCommand(clawServos::setFClawPos),
                new WaitCommand(200),
                new InstantCommand(clawServos::clawOpen)
        );
    }
}
