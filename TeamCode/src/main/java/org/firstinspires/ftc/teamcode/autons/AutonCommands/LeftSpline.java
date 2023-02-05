package org.firstinspires.ftc.teamcode.autons.AutonCommands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowSplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.SensorCommands.Auto.IntakeAutoCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.util.LeftSplineValues;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


public class LeftSpline extends SequentialCommandGroup{
    public LeftSpline(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo, SensorColor sensorColor){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(LeftSplineValues.ToHigh.x1, LeftSplineValues.ToHigh.y1), Math.toRadians(LeftSplineValues.ToHigh.heading1))    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo)  ,
                        new SlowSplineCommand(drivetrain, new Vector2d(54, 22), Math.toRadians(LeftSplineValues.ToCone.heading), true)   //Load
                ),
                new IntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),



//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, turnServo,true),
//                        new SplineCommand(drivetrain, new Vector2d(57.2, -2.8), Math.toRadians(327.4), PoseStorage.cycle)    //Cycle
//                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
//                new ParallelCommandGroup(
//                        new PrePickB5Command(slide, claw, arm, turnServo),
//                        new SlowSplineCommand(drivetrain, new Vector2d(55.1, 27.), Math.toRadians(90), PoseStorage.load, true)   //Load
//                ),
//                new IntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),


//                /**Cone 1**/
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(LeftSplineValues.ToHigh.x1, LeftSplineValues.ToHigh.y1), Math.toRadians(LeftSplineValues.ToHigh.heading1), PoseStorage.cycle)    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new SlideResetUpAutonCommand(slide, arm, claw, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(LeftSplineValues.ToCone.x, LeftSplineValues.ToCone.y), Math.toRadians(LeftSplineValues.ToCone.heading), PoseStorage.load, true)   //Load
                )

        );
    }
}