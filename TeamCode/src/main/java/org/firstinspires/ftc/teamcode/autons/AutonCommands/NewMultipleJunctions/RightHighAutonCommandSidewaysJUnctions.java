package org.firstinspires.ftc.teamcode.autons.AutonCommands.NewMultipleJunctions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
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
                new SlideMidFAutoCommand(slide, arm, clawServos),
                new StrafeRightCommand(drivetrain, 51),
                new SlowDriveForwardCommand(drivetrain, 1),
                new DropAutoConeCommand(clawServos, slide, arm),
//                new InstantCommand(clawServos::clawOpen),
                new WaitCommand(500),
                new StrafeRightCommand(drivetrain, 16.4),
                new PrePickC5BCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 1.5, true),
                new DriveForwardCommand(drivetrain, -22.2),


                new PickC5BCommand(slide, clawServos, arm, drivetrain),
                new SlideLowFAutoCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 65, true),
//                new TurnCommand(drivetrain, 75),
                new SlowDriveForwardCommand(drivetrain, 3.3),
                new WaitCommand(200),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain, -1.77),
                new PrePickC4BCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 1.1, false),//turned from true
                new SlowDriveForwardCommand(drivetrain, -2),
                new InstantCommand(slide::slideCone4),



                new PickC4BCommand(slide, clawServos, arm, drivetrain),
                new InstantCommand(arm::moveReset),
                new DriveForwardCommand(drivetrain, 35),
                new StrafeRightCommand(drivetrain, 2),
                new TurnToCommand(drivetrain, 270),
                new TurnToCommand(drivetrain, 271.9), //To fix error


//                new TurnToCommand(drivetrain, 263),
//                new SlowDriveForwardCommand(drivetrain, -3),
//                new InstantCommand(arm::moveReset),
//                new StrafeLeftCommand(drivetrain, 41.5),

                new SlideHighFAutoCommand(slide, arm, clawServos),
                new SlowDriveForwardCommand(drivetrain, 1),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(300),
                new SlowDriveForwardCommand(drivetrain, -3),
//                new TurnToCommand(drivetrain, 270),


                new SlideResetAutonFCommand(slide, arm, clawServos)
        );
    }
}