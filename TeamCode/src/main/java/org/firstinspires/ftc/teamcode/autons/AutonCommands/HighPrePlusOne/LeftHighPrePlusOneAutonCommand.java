package org.firstinspires.ftc.teamcode.autons.AutonCommands.HighPrePlusOne;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Front.*;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.*;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideLowBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class LeftHighPrePlusOneAutonCommand extends SequentialCommandGroup{
    public LeftHighPrePlusOneAutonCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
//                new TurnToCommand(drivetrain, 90),
//                new TurnToCommand(drivetrain, 180),
//                new TurnToCommand(drivetrain, 270),
//                new TurnToCommand(drivetrain, 360),
//                new TurnToCommand(drivetrain, 0),


                new DriveForwardCommand(drivetrain, -55),
                new TurnToCommand(drivetrain, 317.9),
                new SlideHighBCommand(slide, arm, clawServos, true),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrain,-5.85),
                new WaitCommand(500),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(200),

                new SlowDriveForwardCommand(drivetrain,6),
                new PrePick5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 263.9),
                new DriveForwardCommand(drivetrain, 27),


                new PickC5FCommand(slide, clawServos),
                new SlideLowBCommand(slide, arm, clawServos, true),
                new TurnToCommand(drivetrain, 218),
                new SlowDriveForwardCommand(drivetrain, -1.4),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new PrePick5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 265),
                new WaitCommand(200),



                new PickC4FCommand(slide, clawServos),
                new SlideLowBCommand(slide, arm, clawServos, true),
                new TurnToCommand(drivetrain, 212),
                new SlowDriveForwardCommand(drivetrain, -2),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new PrePick5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 267),

                new WaitCommand(200),


                new PickC3FCommand(slide, clawServos),
                new SlideLowBCommand(slide, arm, clawServos, true),
                new TurnToCommand(drivetrain, 210),
                new SlowDriveForwardCommand(drivetrain, -2),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new TurnToCommand(drivetrain, 269),
//                new PrePickC2FCommand(slide, clawServos, arm),
//                new WaitCommand(200),






//                new PickC3BCommand(slide, clawServos, arm, drivetrain),
//                new TurnToCommand(drivetrain, 140),
//                new SlideLowFCommand(slide, arm, clawServos),
//                new InstantCommand(clawServos::clawOpen, clawServos),
//                new WaitCommand(600),
//                new TurnToCommand(drivetrain, 270),
//
//                new SlideResetFCommandT(slide, arm, clawServos),
//                new WaitCommand(200),
//                new InstantCommand(arm::moveReset, arm),
//                new DriveForwardCommand(drivetrain, 50)
        new SlideResetUpAutonCommand(slide, arm, clawServos)
        );
    }
}