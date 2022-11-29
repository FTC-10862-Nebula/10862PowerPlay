package org.firstinspires.ftc.teamcode.autons.AutonCommands.NewMultipleJunctions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Back.PickC4BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Back.PickC5BCommand;

import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickC4BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickC5BCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideHighFAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideLowFAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideMidFAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighAutonCommandSidewaysJUnctions extends SequentialCommandGroup{
    public RightHighAutonCommandSidewaysJUnctions(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        /*
Turn is Counterclockwise
                new TurnToCommand(drivetrain, 90),
                new TurnToCommand(drivetrain, 180),
                new TurnToCommand(drivetrain, 270),
                new TurnToCommand(drivetrain, 360),
                new TurnToCommand(drivetrain, 0),
*/
        addCommands(
                new DropConeCommand(clawServos, slide, arm)
//                new SlideMidFAutoCommand(slide, arm, clawServos),
//                new StrafeRightCommand(drivetrain, 52),
//                new SlowDriveForwardCommand(drivetrain, 3),
//                new DropConeCommand(clawServos, slide, arm),
//
//                new InstantCommand(clawServos::clawOpen),
//
//                new WaitCommand(500),
//                new PrePickC5BCommand(slide, clawServos, arm),
//                new StrafeRightCommand(drivetrain, 17.9),
//                new DriveForwardCommand(drivetrain, -25.4),
//
//
//                new PickC5BCommand(slide, clawServos, arm, drivetrain),
//                new SlideLowFAutoCommand(slide, arm, clawServos),
//                new TurnCommand(drivetrain, 66),
//                new SlowDriveForwardCommand(drivetrain, 3.9),
//                new WaitCommand(200),
//                new DropConeCommand(clawServos, slide, arm),
//                new WaitCommand(200),
//                new SlowDriveForwardCommand(drivetrain, 2.1),
//                new PrePickC4BCommand(slide, clawServos, arm),
//                new TurnToCommand(drivetrain, 0),
//                new SlowDriveForwardCommand(drivetrain, -3),
//                new InstantCommand(slide::slideCone4),



//                new PickC4BCommand(slide, clawServos, arm, drivetrain),
//                new DriveForwardCommand(drivetrain, 33.1),
//                new SlideHighFAutoCommand(slide, arm, clawServos),
//                new TurnToCommand(drivetrain, 273.5, false),
//                new SlowDriveForwardCommand(drivetrain, 1),
//                new DropConeCommand(clawServos, slide, arm),
//                new WaitCommand(300),
//                new SlowDriveForwardCommand(drivetrain, 2),
//                new TurnToCommand(drivetrain, 270),
//
//
//        new SlideResetAutonFCommand(slide, arm, clawServos),
//                new StrafeRightCommand(drivetrain, 19)
        );
    }
}