package org.firstinspires.ftc.teamcode.commands.SensorCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class BlueIntakeCommand extends ParallelCommandGroup {

    public BlueIntakeCommand(MecanumDrive drivetrain, Slide slide, Claw claw, Arm arm, SensorColor sensorColor) {
        addRequirements(claw, sensorColor);
        addCommands(
//                new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(9),
                new ConditionalCommand(
                        new SequentialCommandGroup( //When True
                                new InstantCommand(claw::clawClose)
//                                new InstantCommand(slide::autoPickSlideUp)
                        ),
                        new SequentialCommandGroup( /*When False*/                                 new InstantCommand(claw::clawOpen)
                        ),
                        sensorColor::grabbedBlueCone    //What they R checking
                )
        );
    }
}