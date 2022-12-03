package org.firstinspires.ftc.teamcode.commands.SensorCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class BlueIntakeCommand extends SequentialCommandGroup {

    public BlueIntakeCommand(DrivetrainCOrrect drivetrainCorrect, Slide slide, ClawServos clawServos, Arm arm, SensorColor sensorColor) {
        addRequirements(clawServos, sensorColor);
        addCommands(
//                new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(9),
                new ConditionalCommand(
                        new SequentialCommandGroup( //When True
                                new InstantCommand(clawServos::clawClose)
//                                new InstantCommand(slide::autoPickSlideUp)
                        ),
                        new SequentialCommandGroup( /*When False*/                                 new InstantCommand(clawServos::clawOpen)
                        ),
                        sensorColor::grabbedBlueCone    //What they R checking
                )
        );
    }
}