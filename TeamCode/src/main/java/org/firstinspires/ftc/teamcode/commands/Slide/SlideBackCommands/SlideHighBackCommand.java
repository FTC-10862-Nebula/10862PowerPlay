package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideHighBackCommand extends SequentialCommandGroup {
    public SlideHighBackCommand(Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(slide::slideHigh, slide),
                new WaitCommand(150),
                new InstantCommand(clawMotors::moveHighB, clawMotors),
                new WaitCommand(200),
                new InstantCommand(clawServos::setBClawPos)

        );
    }   
}