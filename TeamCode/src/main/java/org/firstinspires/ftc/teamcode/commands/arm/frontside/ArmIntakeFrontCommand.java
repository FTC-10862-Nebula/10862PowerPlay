package org.firstinspires.ftc.teamcode.commands.arm.frontside;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class ArmIntakeFrontCommand extends SequentialCommandGroup {
    public ArmIntakeFrontCommand(Slide slide, Pivot pivot, Claw claw, TurnServo turnServo){
        addRequirements(pivot);
        addCommands(
                new InstantCommand(turnServo::setFClawPos),
                new WaitCommand(800),
                new InstantCommand(claw::clawOpen)
        );
    }
}
