package org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class ArmCone2BackCommand extends SequentialCommandGroup   {
    public ArmCone2BackCommand(Slide slide, Claw claw, Pivot pivot, TurnServo turnServo){
        addCommands(
                new WaitCommand(420),
                new ParallelCommandGroup(
                        new InstantCommand(claw::clawClose),
//                        new InstantCommand(claw::clawAutoClose),
                        new InstantCommand(pivot::moveIntakeBAuto)

                ),
                new WaitCommand(150),
                new ParallelCommandGroup(
                        new InstantCommand(slide::slideCone2),
                        new InstantCommand(turnServo::setBClawPos)
                ),
                new WaitCommand(210),
                new InstantCommand(claw::clawOpen)
        );
    }
}