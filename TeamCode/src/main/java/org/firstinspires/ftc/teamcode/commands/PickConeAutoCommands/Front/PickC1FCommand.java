package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Front;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickC1FCommand extends SequentialCommandGroup{
    public PickC1FCommand(Slide slide, ClawServos clawServos, Arm arm,  DrivetrainCOrrect drivetrainCorrect){
        addCommands(
//                new InstantCommand(clawServos::clawClose),
//                new InstantCommand(arm::moveIntakeFAuto),
//                new InstantCommand(clawServos::setFClawPos),
//                new InstantCommand(clawServos::clawOpen),
//                new InstantCommand(slide::slideCone1),

                new SlowDriveForwardCommand(drivetrainCorrect, 3),
                new InstantCommand(clawServos::clawClose),
                new WaitCommand(100),
                new InstantCommand(slide:: slideLow),
                new DriveForwardCommand(drivetrainCorrect, -4)
        );
    }
}