package org.firstinspires.ftc.teamcode.commands.SensorCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class RedIntakeCommand extends SequentialCommandGroup {

    public RedIntakeCommand(DrivetrainCOrrect drivetrainCorrect, Slide slide, ClawServos clawServos, Arm arm, SensorColor sensorColor) {
        addRequirements(sensorColor, clawServos);
        addCommands(
                new WaitUntilCommand(sensorColor::grabbedRedCone).withTimeout(9),
                new ConditionalCommand(
                        new SequentialCommandGroup( //When True
                                new InstantCommand(clawServos::clawClose)
//                                new InstantCommand(slide::autoPickSlideUp)

//                                new InstantCommand(clawServos::stopClaw)
                        ),
                        new SequentialCommandGroup( //When False
//                                new InstantCommand(slide::autoDropSlideUp)
//                                new DriveForwardCommand(drivetrain,5)

                                new InstantCommand(clawServos::clawOpen)
//                                new InstantCommand(clawServos::intakeClaw),
//                                new WaitCommand(100),
//                                new InstantCommand(clawServos::stopClaw)
                        ),
                        sensorColor::grabbedRedCone
                )
        );
    }
}