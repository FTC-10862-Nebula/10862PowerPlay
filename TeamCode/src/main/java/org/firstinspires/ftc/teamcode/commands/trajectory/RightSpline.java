package org.firstinspires.ftc.teamcode.commands.trajectory;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.trajectory.autoConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.trajectory.autoConeCommands.prePickB.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.drive.autoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.outtake.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


public class RightSpline extends SequentialCommandGroup{
    public RightSpline(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(58.8, 8.4), Math.toRadians(32))    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SplineCommand(drivetrain, new Vector2d(52.5, -23), Math.toRadians(268), true)   //Load
                ),
                new PickConeCommand(slide, claw),


                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(58.8, 8.4), Math.toRadians(32), PoseStorage.cycle)    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SplineCommand(drivetrain, new Vector2d(52.5, -23), Math.toRadians(268), PoseStorage.load, true)   //Load
                ),
                new PickConeCommand(slide, claw)


//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, true),
//                        new SplineCommand(drivetrain, new Vector2d(58.8, 8.4), Math.toRadians(32), PoseStorage.cycle)    //Cycle
//                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
//                new ParallelCommandGroup(
//                        new PrePickB5Command(slide, claw, arm),
//                        new SplineCommand(drivetrain, new Vector2d(52.5, -23), Math.toRadians(268), PoseStorage.load, true)   //Load
//                ),
//
//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, true),
//                        new SplineCommand(drivetrain, new Vector2d(58.8, 8.4), Math.toRadians(32), PoseStorage.cycle)    //Cycle
//                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
//                new ParallelCommandGroup(
//                        new PrePickB5Command(slide, claw, arm),
//                        new SplineCommand(drivetrain, new Vector2d(52.5, -23), Math.toRadians(268), PoseStorage.load, true)   //Load
//                ),
//
//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, true),
//                        new SplineCommand(drivetrain, new Vector2d(58.8, 8.4), Math.toRadians(32), PoseStorage.cycle)    //Cycle
//                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
//                new ParallelCommandGroup(
//                        new PrePickB5Command(slide, claw, arm),
//                        new SplineCommand(drivetrain, new Vector2d(52.5, -23), Math.toRadians(268), PoseStorage.load, true)   //Load
//                ),
//
//
//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, true),
//                        new SplineCommand(drivetrain, new Vector2d(58.8, 8.4), Math.toRadians(32), PoseStorage.cycle)    //Cycle
//                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
//                new ParallelCommandGroup(
//                        new PrePickB5Command(slide, claw, arm),
//                        new SplineCommand(drivetrain, new Vector2d(52.5, -23), Math.toRadians(268), PoseStorage.load, true)   //Load
//                ),
//
//
//                /**Cone 1**/
//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, true),
//                        new SplineCommand(drivetrain, new Vector2d(58, 8.5), Math.toRadians(23), PoseStorage.cycle)    //Cycle
//                ),
//                new DropAutoConeCommand(claw, slide, arm, true),
//
//                new SplineCommand(drivetrain, new Vector2d(58,8.5), Math.toRadians(0))
//                new TurnToCommand(drivetrain, 0) - Can't Use without IMU
        );
    }
}