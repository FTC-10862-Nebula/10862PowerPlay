package org.firstinspires.ftc.teamcode.autons.AutonCommands.HighPrePlusOne;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Front.*;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.*;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Back.SlideHighBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Back.SlideLowBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class LeftHighPrePlusOneAutonCommand extends SequentialCommandGroup{
    public LeftHighPrePlusOneAutonCommand(DrivetrainCOrrect drivetrainCorrect, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
//                new TurnToCommand(drivetrain, 90),
//                new TurnToCommand(drivetrain, 180),
//                new TurnToCommand(drivetrain, 270),
//                new TurnToCommand(drivetrain, 360),
//                new TurnToCommand(drivetrain, 0),


                new DriveForwardCommand(drivetrainCorrect, -55),
                new TurnToCommand(drivetrainCorrect, 317.9),
                new SlideHighBAutoCommand(slide, arm, clawServos),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrainCorrect,-5.85),
                new WaitCommand(500),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(200),

                new SlowDriveForwardCommand(drivetrainCorrect,6),
                new PrePickC5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrainCorrect, 263.9),
                new DriveForwardCommand(drivetrainCorrect, 27),


                new PickC5FCommand(slide, clawServos, arm, drivetrainCorrect),
                new SlideLowBAutoCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrainCorrect, 218),
                new SlowDriveForwardCommand(drivetrainCorrect, -1.4),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrainCorrect, 1.8),
                new PrePickC4FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrainCorrect, 265),
                new WaitCommand(200),



                new PickC4FCommand(slide, clawServos, arm, drivetrainCorrect),
                new SlideLowBAutoCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrainCorrect, 212),
                new SlowDriveForwardCommand(drivetrainCorrect, -2),                new DropConeCommand(clawServos, slide, arm),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrainCorrect, 1.8),
                new PrePickC3FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrainCorrect, 267),

                new WaitCommand(200),


                new PickC3FCommand(slide, clawServos, arm, drivetrainCorrect),
                new SlideLowBAutoCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrainCorrect, 210),
                new SlowDriveForwardCommand(drivetrainCorrect, -2),                new DropConeCommand(clawServos, slide, arm),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrainCorrect, 1.8),
                new TurnToCommand(drivetrainCorrect, 269),
//                new PrePickC2FCommand(slide, clawServos, arm),
//                new WaitCommand(200),






//                new PickC3BCommand(slide, clawServos, arm, drivetrain),
//                new TurnToCommand(drivetrain, 140),
//                new SlideLowFrontCommand(slide, arm, clawServos),
//                new InstantCommand(clawServos::clawOpen, clawServos),
//                new WaitCommand(600),
//                new TurnToCommand(drivetrain, 270),
//
//                new SlideResetFrontCommandT(slide, arm, clawServos),
//                new WaitCommand(200),
//                new InstantCommand(arm::moveReset, arm),
//                new DriveForwardCommand(drivetrain, 50)
        new SlideResetAutonFCommand(slide, arm, clawServos)
        );
    }
}