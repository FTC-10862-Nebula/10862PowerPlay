package org.firstinspires.ftc.teamcode.autons.AutonCommands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowSplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.SensorCommands.Auto.RedIntakeAutoCommand;
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
//                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(57.2, -3.7), Math.toRadians(319))    //Cycle
                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
//                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(56.4, 25.9), Math.toRadians(89.9), true)   //Load
                ),
                new RedIntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),



                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(57.2, -3.7), Math.toRadians(319), PoseStorage.cycle)    //Cycle
                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
//                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(56.4, 25.9), Math.toRadians(89.9), PoseStorage.load, true)   //Load
                ),
                new RedIntakeAutoCommand(drivetrain, slide, claw, sensorColor, true)

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