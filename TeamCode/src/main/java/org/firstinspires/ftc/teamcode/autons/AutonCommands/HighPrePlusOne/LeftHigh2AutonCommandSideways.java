package org.firstinspires.ftc.teamcode.autons.AutonCommands.HighPrePlusOne;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front.PickC3FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front.PickC4FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front.PickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.PrePickConeCommands.PrePickC3FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.PrePickConeCommands.PrePickC4FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.PrePickConeCommands.PrePickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideHighBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideLowAutonBCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class LeftHigh2AutonCommandSideways extends SequentialCommandGroup{
    public LeftHigh2AutonCommandSideways(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(    //Turn is Counterclockwise
//                new TurnToCommand(drivetrain, 90),
//                new TurnToCommand(drivetrain, 180),
//                new TurnToCommand(drivetrain, 270),
//                new TurnToCommand(drivetrain, 360),
//                new TurnToCommand(drivetrain, 0),

                new StrafeRightCommand(drivetrain, 68),
                new TurnToCommand(drivetrain, 36.5),
                new SlideHighBAutoCommand(slide, arm, clawServos),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrain ,-5.85),
                new WaitCommand(500),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain ,6),
                new PrePickC5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 5),
                new DriveForwardCommand(drivetrain, 24.5),


                new PickC5FCommand(slide, clawServos, arm, drivetrain),
                new SlideLowAutonBCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 322, true),
                new SlowDriveForwardCommand(drivetrain, -3),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new PrePickC4FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 3),
                new WaitCommand(200),



                new PickC4FCommand(slide, clawServos, arm, drivetrain),
                new SlideLowAutonBCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 322, true),
                new SlowDriveForwardCommand(drivetrain, -2),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new PrePickC3FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 2),

                new WaitCommand(200),


                new PickC3FCommand(slide, clawServos, arm, drivetrain),
                new SlideLowAutonBCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 322),
                new SlowDriveForwardCommand(drivetrain, -2),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 1.8),
                new TurnToCommand(drivetrain, 8),
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