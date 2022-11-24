package org.firstinspires.ftc.teamcode.autons.AutonCommands.NewLeft;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front.PickC4FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front.PickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.PrePickConeCommands.PrePickC4FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.PrePickConeCommands.PrePickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideHighAutonBCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideLowAutonBCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideMidBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class LeftHighAutonCommandSidewaysJUnctions extends SequentialCommandGroup{
    public LeftHighAutonCommandSidewaysJUnctions(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(    //Turn is Counterclockwise
//                new TurnToCommand(drivetrain, 90),
//                new TurnToCommand(drivetrain, 180),
//                new TurnToCommand(drivetrain, 270),
//                new TurnToCommand(drivetrain, 360),
//                new TurnToCommand(drivetrain, 0),

                new StrafeRightCommand(drivetrain, 52),
                new SlideMidBAutoCommand(slide, arm, clawServos),
                new WaitCommand(100),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(200),
                new PrePickC5FCommand(slide, clawServos, arm),
                new StrafeRightCommand(drivetrain, 11),
                new DriveForwardCommand(drivetrain, 24.5),


                new PickC5FCommand(slide, clawServos, arm, drivetrain),
                new SlideLowAutonBCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 322, true),
                new SlowDriveForwardCommand(drivetrain, -3),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new PrePickC4FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 0),
                new WaitCommand(200),



                new PickC4FCommand(slide, clawServos, arm, drivetrain),
                new DriveForwardCommand(drivetrain, -24.5),
                new TurnToCommand(drivetrain, 36.5),
                new SlideHighAutonBCommand(slide, arm, clawServos),
                new SlowDriveForwardCommand(drivetrain, -3.2),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 2),
//                new PrePickC3FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 270, true),


//                new PickC3FCommand(slide, clawServos, arm, drivetrain),
//                new SlideLowAutonBCommand(slide, arm, clawServos),
//                new TurnToCommand(drivetrain, 322),
//                new SlowDriveForwardCommand(drivetrain, -2),
//                new DropConeCommand(clawServos, slide, arm),
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
                new StrafeRightCommand(drivetrain, 15)
        );
    }
}