package org.firstinspires.ftc.teamcode.autons;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePick5FCommand;
import org.firstinspires.ftc.teamcode.commands.SensorCommands.Auto.IntakeAutoCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideMidBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;


public class LeftStrafe extends SequentialCommandGroup{
    public LeftStrafe(Drivetrain drivetrain, Slide slide, Arm arm, TurnServo turnServo, SensorColor sensorColor, Claw claw){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new StrafeRightCommand(drivetrain, 38.5),
                        new SlideMidBCommand(slide, arm, claw, turnServo, true)
                ),
                new DropAutoConeCommand(claw, slide, arm, true),


                new StrafeRightCommand(drivetrain, 12),
                new PrePick5FCommand(slide, claw, arm, turnServo),
                new DriveForwardCommand(drivetrain, 24),
                new IntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),
                new ParallelCommandGroup(
                        new DriveForwardCommand(drivetrain, -23),
                        new SlideMidBCommand(slide, arm, claw, turnServo, true )
                ),
                new StrafeLeftCommand(drivetrain, 12),
                new DropAutoConeCommand(claw, slide, arm, true),



//                new ParallelCommandGroup(
//                        new StrafeLeftCommand(drivetrain, 12),
//                        new PrePickB5Command(slide, claw, arm, turnServo)
//                ),
//                new DriveForwardCommand(drivetrain, -15),
//                new IntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),
//                new ParallelCommandGroup(
//                        new DriveForwardCommand(drivetrain, 15),
//                        new SlideMidFCommand(slide, arm, claw, turnServo, true )
//                ),
//                new StrafeRightCommand(drivetrain, 12),
//                new DropAutoConeCommand(claw, slide, arm, true),



//                new ParallelCommandGroup(
//                        new StrafeLeftCommand(drivetrain, 8),
//                        new PrePickB5Command(slide, claw, arm, turnServo)
//                ),
//                new DriveForwardCommand(drivetrain, -15),
//                new IntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),
//                new ParallelCommandGroup(
//                        new DriveForwardCommand(drivetrain, 15),
//                        new SlideMidFCommand(slide, arm, claw, turnServo, true )
//                ),
//                new StrafeRightCommand(drivetrain, 8),
//                new DropAutoConeCommand(claw, slide, arm, true),

                new StrafeRightCommand(drivetrain, 9.5),
                new SlideResetUpAutonCommand(slide, arm, claw, turnServo)
                );
    }
}