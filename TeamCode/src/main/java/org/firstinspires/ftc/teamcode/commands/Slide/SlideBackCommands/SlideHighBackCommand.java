package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideHighBackCommand extends SequentialCommandGroup {
    public SlideHighBackCommand(Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(slide::slideHigh, slide),
                new InstantCommand(arm::moveHighB, arm),
                new WaitCommand(550),
                new InstantCommand(clawServos::setBClawPos)

        );
    }   
}
