package org.firstinspires.ftc.teamcode.autons.AutonCommands.NewMultipleJunctions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
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
                new SlideMidFAutoCommand(slide, arm, clawServos),
                new StrafeRightCommand(drivetrain, 52),
                new SlowDriveForwardCommand(drivetrain, 3.3),
                new DropAutoConeCommand(clawServos, slide, arm),
//                new WaitCommand(500),
                new StrafeRightCommand(drivetrain, 17.8),
                new PrePickC5BCommand(slide, clawServos, arm),
                new DriveForwardCommand(drivetrain, -24.4),
//                new TurnToCommand(drivetrain, 0),



                new PickC5BCommand(slide, clawServos, arm, drivetrain),
                new SlideLowFAutoCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 61, true),
//                new TurnToCommand(drivetrain, 62.2),
                new SlowDriveForwardCommand(drivetrain, 3.47),
                new WaitCommand(200),
                new DropAutoConeCommand(clawServos, slide, arm),
                new SlowDriveForwardCommand(drivetrain, -1.77),
                new PrePickC4BCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 0),
                new SlowDriveForwardCommand(drivetrain, -2.6),
                new InstantCommand(slide::slideCone4),



                new PickC4BCommand(slide, clawServos, arm, drivetrain),
                new TurnToCommand(drivetrain, 0),   //or remove
                new InstantCommand(slide::slideLow),    /*TTTTESTTTTT*/
                new InstantCommand(arm::moveReset),
//                new DriveForwardCommand(drivetrain, 32.5),

                new DriveForwardCommand(drivetrain, 28),
//                new StrafeLeftCommand(drivetrain, 1.51),
                new TurnToCommand(drivetrain, 271.2, true),
                new StrafeLeftCommand(drivetrain, 10.8),
                new WaitCommand(300),   //Just in case the pole is w0bbling
                new SlowDriveForwardCommand(drivetrain, -1.43),

//                new TurnToCommand(drivetrain, 270),       //To fix error


                new SlideHighFAutoCommand(slide, arm, clawServos),
                new SlowDriveForwardCommand(drivetrain, 0.85),
                new DropAutoConeCommand(clawServos, slide, arm),
                new SlowDriveForwardCommand(drivetrain, -3),
                new TurnToCommand(drivetrain, 271),


                new SlideResetAutonFCommand(slide, arm, clawServos)
        );
    }
}