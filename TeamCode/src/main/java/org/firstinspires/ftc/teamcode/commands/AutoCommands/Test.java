package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoConeCommands.Pick.PickCBCommand;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoConeCommands.PrePickBConeCommands.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;


public class Test extends SequentialCommandGroup{
    public Test(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(57.9, -2.8), Math.toRadians(325))    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SplineCommand(drivetrain, new Vector2d(56.4, 26.75), Math.toRadians(90), true)   //Load
                ),
                new PickCBCommand(slide, claw),



                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(57.9, -2.8), Math.toRadians(325))    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SplineCommand(drivetrain, new Vector2d(56.3, 26.75), Math.toRadians(90), true)   //Load
                ),
                new PickCBCommand(slide, claw),


                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(57.8, -2.8), Math.toRadians(325))    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SplineCommand(drivetrain, new Vector2d(56.2, 26.75), Math.toRadians(90),  true)   //Load
                ),
                new PickCBCommand(slide, claw),


                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(57.9, -2.8), Math.toRadians(325))    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SplineCommand(drivetrain, new Vector2d(56.1, 26.75), Math.toRadians(90), true)   //Load
                ),
                new PickCBCommand(slide, claw)


//                /**Cone 1**/
//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, true),
//                        new SplineCommand(drivetrain, new Vector2d(58.8, -6), Math.toRadians(337), PoseStorage.cycle)    //Cycle
//                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
//
//                new TurnToCommand(drivetrain, 0)
        );
    }
}