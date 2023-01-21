package org.firstinspires.ftc.teamcode.commands.SensorCommands;

import com.acmerobotics.roadrunner.drive.Drive;
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

public class RedIntakeAutoCommand extends ParallelCommandGroup {

    public RedIntakeAutoCommand(Drivetrain drivetrain, Slide slide, Claw claw, SensorColor sensorColor) {
        addRequirements(sensorColor, claw, slide);
        addCommands(
                new WaitUntilCommand(sensorColor::grabbedRedCone).withTimeout(9),
                new ConditionalCommand(
                        new SequentialCommandGroup( //When True
//                                new InstantCommand(claw::clawClose)
                        ),
                        new SequentialCommandGroup( //When False
                                new DriveForwardCommand(drivetrain, -2)
                        ),
                        sensorColor::grabbedRedCone
                ),
                new InstantCommand(claw::clawClose)
        );
    }
}