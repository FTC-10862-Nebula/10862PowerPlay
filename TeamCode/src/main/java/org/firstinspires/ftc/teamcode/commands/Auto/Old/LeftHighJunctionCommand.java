package org.firstinspires.ftc.teamcode.commands.Auto.Old;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndOutake.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PrePickB.PrePickB3Command;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PrePickB.PrePickB4Command;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PrePickF.PrePick5FCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBCommands.SlideMidBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;


public class LeftHighJunctionCommand extends SequentialCommandGroup{
    public LeftHighJunctionCommand(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideMidBCommand(slide, arm, claw, turnServo, true),
                        new StrafeRightCommand(drivetrain, 51.9)
                ),
                new DropAutoConeCommand(claw, slide, arm,true),
                new ParallelCommandGroup(
                        new StrafeRightCommand(drivetrain, 20.6),
                        new PrePick5FCommand(slide, claw, arm, turnServo)
                ),
                new DriveForwardCommand(drivetrain, 26.85),



                /***Cone 5***/
                new PickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new SlideHighBCommand(slide, arm, claw, turnServo, true),
                        new DriveForwardCommand(drivetrain, 29)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, 52.5),//61 ish
                        new DriveForwardCommand(drivetrain, -2.5),
                        new DropAutoConeCommand(claw, slide, arm,true),
                        new DriveForwardCommand(drivetrain, 4.88)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 1, true),
                        new PrePickB4Command(slide, claw, arm, turnServo)
                ),
                new DriveForwardCommand(drivetrain, 29.9),



                /***Cone 4***/
                new PickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new SlideHighBCommand(slide, arm, claw, turnServo, true),
                        new DriveForwardCommand(drivetrain, -30)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, 50.5),//oprg:300 to -60
                        new DriveForwardCommand(drivetrain, -2.3),
                        new DropAutoConeCommand(claw, slide, arm,true),
                        new DriveForwardCommand(drivetrain, 4.5)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 1.5, true),
                        new PrePickB3Command(slide, claw, arm, turnServo)
                ),
                new DriveForwardCommand(drivetrain, 30.5),



                /***Cone 3***/
                new PickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new SlideHighBCommand(slide, arm, claw, turnServo, true),
                        new DriveForwardCommand(drivetrain, -28.8)
                ),
                new TurnCommand(drivetrain, -46.5),//oprg:300 to -60
                new DriveForwardCommand(drivetrain, 2.3),
                new DropAutoConeCommand(claw, slide, arm,true),



                //Parking - Remove!!!!!!!!!!!!
                new ParallelCommandGroup(
                        new SlideResetUpAutonCommand(slide, arm, claw, turnServo),
                        new TurnToCommand(drivetrain, 270)
                ),
                new DriveForwardCommand(drivetrain, -7),
                new StrafeLeftCommand(drivetrain, 36)
        );
    }
}