package org.firstinspires.ftc.teamcode.commands.SensorCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class BlueIntakeAutoCommand extends ParallelCommandGroup {

    public BlueIntakeAutoCommand(Drivetrain drivetrain, Slide slide, Claw claw, SensorColor sensorColor) {
        addRequirements(claw, sensorColor, slide, drivetrain);
        addCommands(
                new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(9),
                new ConditionalCommand(
                        new SequentialCommandGroup( //When True
//                                new InstantCommand(claw::clawClose)
                        ),
                        new SequentialCommandGroup( //When False
                                new DriveForwardCommand(drivetrain, -2)
                        ),
                        sensorColor::grabbedBlueCone
                ),
                new InstantCommand(claw::clawClose)
        );
    }
}