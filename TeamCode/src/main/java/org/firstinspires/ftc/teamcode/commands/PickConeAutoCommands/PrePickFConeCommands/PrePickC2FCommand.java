package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PrePickC2FCommand extends SequentialCommandGroup{
    public PrePickC2FCommand(Slide slide, ClawServos clawServos, Arm arm){
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(arm::moveIntakeFAuto),
                new InstantCommand(clawServos::setFClawPos),
                new InstantCommand(clawServos::clawOpen),
                new InstantCommand(slide::slideCone5)

        );
    }
}