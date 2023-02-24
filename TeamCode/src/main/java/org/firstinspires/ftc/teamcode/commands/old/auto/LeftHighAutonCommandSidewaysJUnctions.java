package org.firstinspires.ftc.teamcode.commands.old.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Deprecated
public class LeftHighAutonCommandSidewaysJUnctions extends SequentialCommandGroup{
    public LeftHighAutonCommandSidewaysJUnctions(Drivetrain drivetrain, Slide slide, Pivot pivot, Claw claw){
        addCommands(    //Turn is Counterclockwise
//                new TurnToCommand(drivetrain, 90),
//                new TurnToCommand(drivetrain, 180),
//                new TurnToCommand(drivetrain, 270),
//                new TurnToCommand(drivetrain, 360),
//                new TurnToCommand(drivetrain, 0),

                /*new SlideMidBCommand(slide, arm, claw, turnServo, true),
                new StrafeRightCommand(drivetrain, 52.7),
                new SlowDriveForwardCommand(drivetrain, -0.8),
                new DropAutoConeCommand(claw, slide, arm,true),
                new InstantCommand(claw::clawOpen),

                new WaitCommand(500),
                new PrePick5FCommand(slide, claw, arm, turnServo),
                new StrafeRightCommand(drivetrain, 16.5),
                new DriveForwardCommand(drivetrain, 26.5),


                new PickCFCommand(slide, claw),
                new SlideLowBCommand(slide, arm, claw, turnServo, true),
                new TurnCommand(drivetrain, -55.3),
                new SlowDriveForwardCommand(drivetrain, -3),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new PrePick5FCommand(slide, claw, arm, turnServo),
                new TurnToCommand(drivetrain, 3, true),
                new SlowDriveForwardCommand(drivetrain, 3),
                new InstantCommand(slide::slideCone4),



                new PickCFCommand(slide, claw),
                new DriveForwardCommand(drivetrain, -35.8),
                new SlideHighFCommand(slide, arm, claw, true),
                new TurnToCommand(drivetrain, 264.9),
                new SlowDriveForwardCommand(drivetrain, 2.1),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(300),
                new SlowDriveForwardCommand(drivetrain, -2),

//                new TurnToCommand(drivetrain, 270),


//                new PickC3FCommand(slide, claw, arm, drivetrain),
//                new SlideLowAutonBCommand(slide, arm, claw),
//                new TurnToCommand(drivetrain, 322),
//                new SlowDriveForwardCommand(drivetrain, -2),
//                new DropAutoConeCommand(claw, slide, arm),
//                new WaitCommand(400),
//                new SlowDriveForwardCommand(drivetrain, 1.8),
//                new TurnToCommand(drivetrain, 8),
//                new PrePick2FCommand(slide, claw, arm),
//                new WaitCommand(200),






//                new PickC3BCommand(slide, claw, arm, drivetrain),
//                new TurnToCommand(drivetrain, 140),
//                new SlideLowFCommand(slide, arm, claw),
//                new InstantCommand(claw::clawOpen, claw),
//                new WaitCommand(600),
//                new TurnToCommand(drivetrain, 270),
//
//                new SlideIntakeFCommandT(slide, arm, claw),
//                new WaitCommand(200),
//                new InstantCommand(arm::moveReset, arm),
//                new DriveForwardCommand(drivetrain, 50)
                new SlideResetUpAutonCommand(slide, arm, claw),
                new StrafeRightCommand(drivetrain, 19)*/
        );
    }
}