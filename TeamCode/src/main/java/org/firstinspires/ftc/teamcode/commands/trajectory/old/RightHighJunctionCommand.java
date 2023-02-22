package org.firstinspires.ftc.teamcode.commands.trajectory.old;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.trajectory.autoConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.trajectory.autoConeCommands.prePickB.PrePickB3Command;
import org.firstinspires.ftc.teamcode.commands.trajectory.autoConeCommands.prePickB.PrePickB4Command;
import org.firstinspires.ftc.teamcode.commands.trajectory.autoConeCommands.prePickB.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.drive.autoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.autoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.drive.autoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.autoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.outtake.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;


public class RightHighJunctionCommand extends SequentialCommandGroup{
    public RightHighJunctionCommand(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideMidFCommand(slide, arm, claw, turnServo, true),
                        new StrafeRightCommand(drivetrain, 52.51)
                ),
                new DropAutoConeCommand(claw, slide, arm,true),
                new ParallelCommandGroup(
                        new StrafeRightCommand(drivetrain, 20.15),
                        new PrePickB5Command(slide, claw, arm, turnServo)
                ),
                new DriveForwardCommand(drivetrain, -26.6),



                /***Cone 5***/
                new PickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                        new DriveForwardCommand(drivetrain, 29.9)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, -58.76),//61 ish
                        new DriveForwardCommand(drivetrain, 5.2),
                        new DropAutoConeCommand(claw, slide, arm,true),
                        new DriveForwardCommand(drivetrain, -5)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 1, true),
                        new PrePickB4Command(slide, claw, arm, turnServo)
                ),
                new DriveForwardCommand(drivetrain, -31.7),



                /***Cone 4***/
                new PickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                        new DriveForwardCommand(drivetrain, 30.9)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, -56.91),//oprg:300 to -60
                        new DriveForwardCommand(drivetrain, 5.1),
                        new DropAutoConeCommand(claw, slide, arm,true),
                        new DriveForwardCommand(drivetrain, -3.7)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 1.3, true),
                        new PrePickB3Command(slide, claw, arm, turnServo)
                ),
                new DriveForwardCommand(drivetrain, -31.)



//                /***Cone 3***/
//                new PickConeCommand(slide, claw),
//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, true),
//                        new DriveForwardCommand(drivetrain, 29.3)
//                ),
//                new TurnCommand(drivetrain, -53.5),//oprg:300 to -60
//                new DriveForwardCommand(drivetrain, 5.95),
//                new DropAutoConeCommand(claw, slide, arm,true),
//
//
//
//
//                //Parking
//                new ParallelCommandGroup(
//                        new SlideResetUpAutonCommand(slide, arm, claw),
//                        new TurnToCommand(drivetrain, 270)
//                ),
//                new DriveForwardCommand(drivetrain, -4)
//                new StrafeRightCommand(drivetrain, 40)
        );
    }
}