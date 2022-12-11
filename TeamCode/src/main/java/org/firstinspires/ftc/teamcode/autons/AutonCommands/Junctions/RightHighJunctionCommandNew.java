package org.firstinspires.ftc.teamcode.autons.AutonCommands.Junctions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Back.PickC4BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Back.PickC5BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideLowFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighJunctionCommandNew extends SequentialCommandGroup{
    public RightHighJunctionCommandNew(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        /*
Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideMidFCommand(slide, arm, clawServos, true),
                        new StrafeRightCommand(drivetrain, 52)
                ),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(drivetrain, 1),
                        new DropAutoConeCommand(clawServos, slide, arm)
                ),
                new ParallelCommandGroup(
                        new StrafeRightCommand(drivetrain, 17.9),
                        new PrePickBAutoCommand(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -28.5),
//                new TurnToCommand(drivetrain, 0),


                new PickC5BCommand(slide, clawServos, arm, drivetrain),
                new ParallelCommandGroup(
//                        new TurnToCommand(drivetrain, 0),   //or remove
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 27.5)
                ),
//                new DriveForwardCommand(drivetrain, 27.5),
                new TurnCommand(drivetrain, 60),
                new TurnToCommand(drivetrain, 302, true),
                new DriveForwardCommand(drivetrain, 5),
                new DropAutoConeCommand(clawServos, slide, arm)
                /*new PickC5BCommand(slide, clawServos, arm, drivetrain),
                new ParallelCommandGroup(
                        new SlideLowFCommand(slide, arm, clawServos, true),
                        new TurnToCommand(drivetrain, 61, true)
                ),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(drivetrain, 3.47),
                        new WaitCommand(200),
                        new DropAutoConeCommand(clawServos, slide, arm)
                ),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(drivetrain, -1.77),
                        new PrePickBAutoCommand(slide, clawServos, arm)
                ),
                new TurnToCommand(drivetrain, 0),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(drivetrain, -2.6),
                        new InstantCommand(slide::slideCone4)
                ),*/


//                new PickC4BCommand(slide, clawServos, arm, drivetrain),
//                new ParallelCommandGroup(
//                        new TurnToCommand(drivetrain, 0),   //or remove
//                        new SlideHighFCommand(slide, arm, clawServos, true)
//                ),
//                new DriveForwardCommand(drivetrain, 24),
//                new TurnToCommand(drivetrain, 220, true),
//                new ParallelCommandGroup(
//                        new SlowDriveForwardCommand(drivetrain, -1.43),
//                        new DropAutoConeCommand(clawServos, slide, arm)
//                ),

//                new SlowDriveForwardCommand(drivetrain, -3),
//                new ParallelCommandGroup(
//                        new TurnToCommand(drivetrain, 271),
//                        new SlideResetUpAutonCommand(slide, arm, clawServos)
//                )
        );
    }
}