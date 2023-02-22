package org.firstinspires.ftc.teamcode.commands.arm.frontside.old.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class ArmCone1FrontCommand extends ParallelCommandGroup {
    public ArmCone1FrontCommand(Slide slide, Claw claw, Pivot pivot, TurnServo turnServo){
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(claw::clawClose),
//                        new InstantCommand(claw::clawAutoClose),
                        new InstantCommand(pivot::moveIntakeFAuto),
                        new InstantCommand(slide::slideCone1)
                ),
                new InstantCommand(turnServo::setFClawPos),
                new WaitCommand(150),
                new InstantCommand(claw::clawOpen)
        );
    }
}