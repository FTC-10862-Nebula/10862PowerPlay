package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetBackCommandT extends SequentialCommandGroup {
    public SlideResetBackCommandT(Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                new InstantCommand(clawServos::clawClose, clawServos),
                new InstantCommand(clawServos::setBClawPos),
                new WaitCommand(800),
                new InstantCommand(slide::slideResting, slide),
                new InstantCommand(clawMotors::moveIntakeB, clawMotors),
                new WaitCommand(600),
                new InstantCommand(clawServos::clawOpen)
        );
    }
}
