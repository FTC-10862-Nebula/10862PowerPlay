package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideGroundBackCommand extends SequentialCommandGroup {
    public SlideGroundBackCommand(Slide slide, Arm arm, ClawServos clawServos) {
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(slide::slideGround, slide),
                new InstantCommand(arm::moveB, arm),
                new WaitCommand(650),
//                new ConditionalCommand(clawServos::setBClawPos,new WaitCommand(200), arm::moveB),
                new InstantCommand(clawServos::setBClawPos)

        );
    }
}
