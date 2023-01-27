package org.firstinspires.ftc.teamcode.autons.AutonCommands.Old;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Pick.PickCFCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePick5FCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideLowBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class LeftHigh2AutonCommandSideways extends SequentialCommandGroup{
    public LeftHigh2AutonCommandSideways(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        addCommands(    //Turn is Counterclockwise
//                new TurnToCommand(drivetrain, 90),
//                new TurnToCommand(drivetrain, 180),
//                new TurnToCommand(drivetrain, 270),
//                new TurnToCommand(drivetrain, 360),
//                new TurnToCommand(drivetrain, 0),

                /*new StrafeRightCommand(drivetrain, 68),
                new TurnToCommand(drivetrain, 36.5),
                new SlideHighBCommand(slide, arm, claw, turnServo,true),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrain,-5.85),
                new WaitCommand(500),
                new DropConeCommand(claw, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain,6),
                new PrePick5FCommand(slide, claw, arm, turnServo),
                new TurnToCommand(drivetrain, 5),
                new DriveForwardCommand(drivetrain, 24.5),


                new PickCFCommand(slide, claw),
                new SlideLowBCommand(slide, arm, claw, turnServo,true),
                new TurnToCommand(drivetrain, 322, true),
                new SlowDriveForwardCommand(drivetrain, -3),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new PrePick5FCommand(slide, claw, arm,turnServo),
                new TurnToCommand(drivetrain, 3),
                new WaitCommand(200),



                new PickCFCommand(slide, claw),
                new SlideLowBCommand(slide, arm, claw, turnServo,true),
                new TurnToCommand(drivetrain, 322, true),
                new SlowDriveForwardCommand(drivetrain, -2),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new PrePick5FCommand(slide, claw, arm,turnServo),
                new TurnToCommand(drivetrain, 2),

                new WaitCommand(200),


                new PickCFCommand(slide, claw),
                new SlideLowBCommand(slide, arm, claw,turnServo, true),
                new TurnToCommand(drivetrain, 322),
                new SlowDriveForwardCommand(drivetrain, -2),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new TurnToCommand(drivetrain, 8),
//                new PrePick2FCommand(slide, claw, arm),
//                new WaitCommand(200),






//                new PickC3BCommand(slide, claw, arm, drivetrain),
//                new TurnToCommand(drivetrain, 140),
//                new SlideLowFCommand(slide, arm, claw),
//                new InstantCommand(claw::clawOpen, claw),
//                new WaitCommand(600),
//                new TurnToCommand(drivetrain, 270),
//
//                new SlideResetFCommandT(slide, arm, claw),
//                new WaitCommand(200),
//                new InstantCommand(arm::moveReset, arm),
//                new DriveForwardCommand(drivetrain, 50)
        new SlideResetUpAutonCommand(slide, arm, claw, turnServo)*/
        );
    }
}