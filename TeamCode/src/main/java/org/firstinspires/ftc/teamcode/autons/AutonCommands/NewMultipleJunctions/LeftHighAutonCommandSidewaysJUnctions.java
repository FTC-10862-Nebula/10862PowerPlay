package org.firstinspires.ftc.teamcode.autons.AutonCommands.NewMultipleJunctions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Front.PickC4FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Front.PickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePickC4FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Back.SlideLowBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Back.SlideMidBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideHighFAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class LeftHighAutonCommandSidewaysJUnctions extends SequentialCommandGroup{
    public LeftHighAutonCommandSidewaysJUnctions(DrivetrainCOrrect drivetrainCorrect, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(    //Turn is Counterclockwise
//                new TurnToCommand(drivetrain, 90),
//                new TurnToCommand(drivetrain, 180),
//                new TurnToCommand(drivetrain, 270),
//                new TurnToCommand(drivetrain, 360),
//                new TurnToCommand(drivetrain, 0),

                new SlideMidBAutoCommand(slide, arm, clawServos),
                new StrafeRightCommand(drivetrainCorrect, 52.7),
                new SlowDriveForwardCommand(drivetrainCorrect, -0.8),
                new DropAutoConeCommand(clawServos, slide, arm),

                new InstantCommand(clawServos::clawOpen),

                new WaitCommand(500),
                new PrePickC5FCommand(slide, clawServos, arm),
                new StrafeRightCommand(drivetrainCorrect, 16.5),
                new DriveForwardCommand(drivetrainCorrect, 26.5),


                new PickC5FCommand(slide, clawServos, arm, drivetrainCorrect),
                new SlideLowBAutoCommand(slide, arm, clawServos),
                new TurnCommand(drivetrainCorrect, -55.3),
                new SlowDriveForwardCommand(drivetrainCorrect, -3),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrainCorrect, 1.8),
                new PrePickC4FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrainCorrect, 3, true),
                new SlowDriveForwardCommand(drivetrainCorrect, 3),
                new InstantCommand(slide::slideCone4),



                new PickC4FCommand(slide, clawServos, arm, drivetrainCorrect),
                new DriveForwardCommand(drivetrainCorrect, -35.8),
                new SlideHighFAutoCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrainCorrect, 264.9),
                new SlowDriveForwardCommand(drivetrainCorrect, 2.1),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(300),
                new SlowDriveForwardCommand(drivetrainCorrect, -2),

//                new TurnToCommand(drivetrain, 270),


//                new PickC3FCommand(slide, clawServos, arm, drivetrain),
//                new SlideLowAutonBCommand(slide, arm, clawServos),
//                new TurnToCommand(drivetrain, 322),
//                new SlowDriveForwardCommand(drivetrain, -2),
//                new DropAutoConeCommand(clawServos, slide, arm),
//                new WaitCommand(400),
//                new SlowDriveForwardCommand(drivetrain, 1.8),
//                new TurnToCommand(drivetrain, 8),
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
                new SlideResetAutonFCommand(slide, arm, clawServos),
                new StrafeRightCommand(drivetrainCorrect, 19)
        );
    }
}