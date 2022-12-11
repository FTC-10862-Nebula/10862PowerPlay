package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Back;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickC5BCommand extends SequentialCommandGroup  {
    public PickC5BCommand(Slide slide, ClawServos clawServos, Arm arm, Drivetrain drivetrain){
        addCommands(
//                new SlowDriveForwardCommand(drivetrain, -2.7),
//                new InstantCommand(clawServos::clawClose),
//                new WaitCommand(100),
//                new ParallelCommandGroup(
//                        new InstantCommand(slide:: slideLow),
//                        new DriveForwardCommand(drivetrain, 3.7)
//                )
//Test below
                new ParallelCommandGroup(
                        new InstantCommand(clawServos::clawClose),
                        new InstantCommand(slide:: slideLow),
                        new DriveForwardCommand(drivetrain, 3.7)
                )
        );
    }
}