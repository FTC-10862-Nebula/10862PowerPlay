package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PrePickB2AutoCommand extends SequentialCommandGroup   {
    public PrePickB2AutoCommand(Slide slide, ClawServos clawServos, Arm arm){
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(clawServos::clawClose),
//                        new InstantCommand(clawServos::clawAutoClose),
                        new InstantCommand(arm::moveIntakeBAuto),
                        new InstantCommand(slide::slideCone2)
                ),
                new InstantCommand(clawServos::setBClawPos),
                new WaitCommand(150),
                new InstantCommand(clawServos::clawOpen)
        );
    }
}