package org.firstinspires.ftc.teamcode.autons.AutonCommands.PrePlusHigh;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Front.PickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Back.SlideHighBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighAutonCommand extends SequentialCommandGroup{
    public RightHighAutonCommand(DrivetrainCOrrect drivetrainCorrect, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrainCorrect, -55),
                new TurnToCommand(drivetrainCorrect, 46.5, true),
                new SlideHighBAutoCommand(slide, arm, clawServos),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrainCorrect,-8.1),
                new WaitCommand(500),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrainCorrect,8),
                new PrePickC5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrainCorrect, 97),
                new DriveForwardCommand(drivetrainCorrect, 23),

                new PickC5FCommand(slide, clawServos, arm, drivetrainCorrect),
                new DriveForwardCommand(drivetrainCorrect, -25),
                new TurnToCommand(drivetrainCorrect, 33),
                new SlideHighBAutoCommand(slide, arm, clawServos),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrainCorrect, -8.1),
                new InstantCommand(clawServos::clawOpen, clawServos),
                new WaitCommand(400),
                new DriveForwardCommand(drivetrainCorrect, 4.5),





                new SlideResetAutonFCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrainCorrect, 90),
                new WaitCommand(200)
//                new InstantCommand(arm::moveReset, arm),
        );
    }
}