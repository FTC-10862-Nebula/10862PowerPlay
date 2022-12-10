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

public class PickC5BCommand extends ParallelCommandGroup {
    public PickC5BCommand(Slide slide, ClawServos clawServos, Arm arm, Drivetrain drivetrain){
        addCommands(
//                new InstantCommand(arm::moveIntakeB),
//                new InstantCommand(clawServos::clawOpen),
//                new InstantCommand(slide::slideCone5),
//                new SlowDriveForwardCommand(drivetrain, 5),
//                new WaitCommand(200),
//                new InstantCommand(clawServos::clawClose),
//                new WaitCommand(50),
//                new InstantCommand(slide:: slideLow)

                new SlowDriveForwardCommand(drivetrain, -2.7),
                new InstantCommand(clawServos::clawClose),
                new WaitCommand(100),
                new InstantCommand(slide:: slideLow),
                new DriveForwardCommand(drivetrain, 3.7)
        );
    }
}