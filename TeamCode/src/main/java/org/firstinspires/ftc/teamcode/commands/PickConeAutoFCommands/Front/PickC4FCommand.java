package org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickC4FCommand extends SequentialCommandGroup{
    public PickC4FCommand(Slide slide, ClawServos clawServos, Arm arm, Drivetrain drivetrain){
        addCommands(
//                new InstantCommand(clawServos::clawClose),
//                new InstantCommand(arm::moveIntakeFAuto),
//                new InstantCommand(clawServos::setFClawPos),
//                new InstantCommand(clawServos::clawOpen),
//                new InstantCommand(slide::slideCone4),

                new SlowDriveForwardCommand(drivetrain, 3.5),
                new InstantCommand(clawServos::clawClose),
                new WaitCommand(100),
                new InstantCommand(slide:: slideLow),
                new DriveForwardCommand(drivetrain, -3.7)
        );
    }
}