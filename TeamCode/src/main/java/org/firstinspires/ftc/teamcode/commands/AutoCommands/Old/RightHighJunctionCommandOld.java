package org.firstinspires.ftc.teamcode.commands.AutoCommands.Old;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;

import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoConeCommands.Pick.PickCBCommand;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoConeCommands.PrePickBConeCommands.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideLowFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;


public class RightHighJunctionCommandOld extends SequentialCommandGroup{
    public RightHighJunctionCommandOld(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        /*
Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideMidFCommand(slide, arm, claw, turnServo, true),
                        new StrafeRightCommand(drivetrain, 52)
                ),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(drivetrain, 1),
                        new DropAutoConeCommand(claw, slide, arm,true)
                        ),
                new ParallelCommandGroup(
                        new StrafeRightCommand(drivetrain, 17.8),
                        new PrePickB5Command(slide, claw, arm, turnServo)
                ),
                new DriveForwardCommand(drivetrain, -24.4),
//                new TurnToCommand(drivetrain, 0),



                new PickCBCommand(slide, claw),
                new ParallelCommandGroup(
                        new SlideLowFCommand(slide, arm, claw, turnServo, true),
                        new TurnToCommand(drivetrain, 61, true)
                ),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(drivetrain, 3.47),
                        new WaitCommand(200),
                        new DropAutoConeCommand(claw, slide, arm,true)
                        ),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(drivetrain, -1.77),
                        new PrePickB5Command(slide, claw, arm, turnServo)
                ),
                new TurnToCommand(drivetrain, 0),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(drivetrain, -2.6),
                        new InstantCommand(slide::slideCone4)
                ),


                new PickCBCommand(slide, claw),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 0),   //or remove
                        new SlideHighFCommand(slide, arm, claw, turnServo, true)
                ),
                new InstantCommand(arm::moveReset),

                new DriveForwardCommand(drivetrain, 28),
//                new StrafeLeftCommand(drivetrain, 1.51),
                new TurnToCommand(drivetrain, 271.2, true),
                new StrafeLeftCommand(drivetrain, 10.8),
                new WaitCommand(300),   //Just in case the pole is w0bbling



//                new SlowDriveForwardCommand(drivetrain, 0.85),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(drivetrain, -1.43),
                        new DropAutoConeCommand(claw, slide, arm,true)
                        ),

                new SlowDriveForwardCommand(drivetrain, -3),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 271),
                        new SlideResetUpAutonCommand(slide, arm, claw, turnServo)
                )
        );
    }
}