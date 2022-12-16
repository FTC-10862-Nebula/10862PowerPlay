package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PrePick3FCommand extends ParallelCommandGroup {
    public PrePick3FCommand(Slide slide, ClawServos clawServos, Arm arm){
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(clawServos::clawClose),
//                        new InstantCommand(clawServos::clawAutoClose),
                        new InstantCommand(arm::moveIntakeFAuto),
                        new InstantCommand(slide::slideCone3)
                ),
                new InstantCommand(clawServos::setFClawPos),
                new WaitCommand(150),
                new InstantCommand(clawServos::clawOpen)
        );
    }
}