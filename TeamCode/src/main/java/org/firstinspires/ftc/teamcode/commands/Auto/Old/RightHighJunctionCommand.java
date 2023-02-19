package org.firstinspires.ftc.teamcode.commands.Auto.Old;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PrePickB.PrePickB3Command;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PrePickB.PrePickB4Command;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PrePickB.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndOutake.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
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