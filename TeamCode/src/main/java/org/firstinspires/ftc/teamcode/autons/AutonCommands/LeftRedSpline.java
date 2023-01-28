package org.firstinspires.ftc.teamcode.autons.AutonCommands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowSplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.SensorCommands.Auto.RedIntakeAutoCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


public class LeftRedSpline extends SequentialCommandGroup{
    public LeftRedSpline(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo, SensorColor sensorColor){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(57.2, -2.1), Math.toRadians(322.5))    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(55.35, 26.78), Math.toRadians(90), true)   //Load
                ),
                new RedIntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),



                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(57.2, -2.1), Math.toRadians(322.5), PoseStorage.cycle)    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(55.35, 26.78), Math.toRadians(90), PoseStorage.load, true)   //Load
                ),
                new RedIntakeAutoCommand(drivetrain, slide, claw, sensorColor, true)
//
//
//
//
//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, turnServo,true),
//                        new SplineCommand(drivetrain, new Vector2d(57.2, -3.7), Math.toRadians(318), PoseStorage.cycle)    //Cycle
//                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
//                new ParallelCommandGroup(
//                        new PrePickB5Command(slide, claw, arm, turnServo),
//                        new SlowSplineCommand(drivetrain, new Vector2d(56.4, 26.1), Math.toRadians(90), PoseStorage.load, true)   //Load
//                ),
//                new RedIntakeAutoCommand(drivetrain, slide, claw, sensorColor, true)

//                /**Cone 1**/
//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, true),
//                        new SplineCommand(drivetrain, new Vector2d(58.8, -6), Math.toRadians(337), PoseStorage.cycle)    //Cycle
//                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
//                  new ParallelCommandGroup(
//                  new SlideResetUpAutonCommand(slide, arm, claw, turnServo),
//                  new SlowSplineCommand(drivetrain, new Vector2d(56.4, 26.1), Math.toRadians(90), PoseStorage.load, true)   //Load
//                  ),

//                new DriveForwardCommand(drivetrain, 1)

        );
    }
}