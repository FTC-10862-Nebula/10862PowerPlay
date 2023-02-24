package org.firstinspires.ftc.teamcode.commands.arm.backside;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class ArmIntakeBackCommand extends SequentialCommandGroup {
    public ArmIntakeBackCommand(Slide slide, Pivot pivot, Claw claw, TurnServo turnServo){
        addCommands(
                new InstantCommand(turnServo::setBClawPos),

                new InstantCommand(claw::clawClose),
                new InstantCommand(pivot::moveInitializationPosition),

                new WaitCommand(500),

                new InstantCommand(claw::clawOpen),
                new InstantCommand(slide::slideResting),

                new WaitCommand(700),
                new InstantCommand(pivot::encoderReset)

        );
    }
}
