package org.firstinspires.ftc.teamcode.commands.old.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.arm.intake.AutoPickConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone5BackCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.arm.outtake.AutoDropConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmHighFrontCommand;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Deprecated
public class RightSpline extends SequentialCommandGroup{
    public RightSpline(Drivetrain drivetrain, Slide slide, Pivot pivot, Claw claw, TurnServo turnServo){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new ArmHighFrontCommand(slide, pivot, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(58.8, 8.4), Math.toRadians(32))    //Cycle
                ),
                new AutoDropConeCommand(claw, slide, pivot, true),
                new ParallelCommandGroup(
                        new ArmCone5BackCommand(slide, claw, pivot, turnServo),
                        new SplineCommand(drivetrain, new Vector2d(52.5, -23), Math.toRadians(268), true)   //Load
                ),
                new AutoPickConeCommand(slide, claw),


                new ParallelCommandGroup(
                        new ArmHighFrontCommand(slide, pivot, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(58.8, 8.4), Math.toRadians(32), PoseStorage.cycle)    //Cycle
                ),
                new AutoDropConeCommand(claw, slide, pivot, true),
                new ParallelCommandGroup(
                        new ArmCone5BackCommand(slide, claw, pivot, turnServo),
                        new SplineCommand(drivetrain, new Vector2d(52.5, -23), Math.toRadians(268), PoseStorage.load, true)   //Load
                ),
                new AutoPickConeCommand(slide, claw)


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