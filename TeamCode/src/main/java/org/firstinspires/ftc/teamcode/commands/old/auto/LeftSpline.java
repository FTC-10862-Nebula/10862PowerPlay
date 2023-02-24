package org.firstinspires.ftc.teamcode.commands.old.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.arm.intake.AutoPickConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone4BackCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.SlowSplineCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.arm.outtake.AutoDropConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone5BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmMidFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.util.LeftSplineValues;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Deprecated
public class LeftSpline extends SequentialCommandGroup{
    public LeftSpline(Drivetrain drivetrain, Slide slide, Pivot pivot, Claw claw, TurnServo turnServo){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new ArmHighFrontCommand(slide, pivot, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(LeftSplineValues.AToHighOne.x1,LeftSplineValues.AToHighOne.y1), Math.toRadians(LeftSplineValues.AToHighOne.heading1))    //Cycle
                ),
                new AutoDropConeCommand(claw, slide, pivot, true),
                new ParallelCommandGroup(
                        new ArmCone5BackCommand(slide, claw, pivot, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(LeftSplineValues.BToConeOne.x, LeftSplineValues.BToConeOne.y2), Math.toRadians(LeftSplineValues.BToConeOne.heading2), true)   //Load
                ),
                new AutoPickConeCommand(slide, claw),
//        new IntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),


                new ParallelCommandGroup(
                        new ArmMidFrontCommand(slide, pivot, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(LeftSplineValues.BToMid.x3,LeftSplineValues.BToMid.y3), Math.toRadians(LeftSplineValues.BToMid.heading3))   //Cycle
                ),
                new WaitCommand(400),
                new AutoDropConeCommand(claw, slide, pivot, true),
                new ParallelCommandGroup(
                        new ArmCone4BackCommand(slide, claw, pivot, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(LeftSplineValues.CToConeOne.x2,LeftSplineValues.CToConeOne.y2), Math.toRadians(LeftSplineValues.CToConeOne.heading2), true)   //Load
                ),
                new AutoPickConeCommand(slide, claw),
//        new IntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),



                new ParallelCommandGroup(
                        new ArmMidFrontCommand(slide, pivot, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(LeftSplineValues.CToMid.x3,LeftSplineValues.CToMid.y3), Math.toRadians(LeftSplineValues.CToMid.heading3))   //Cycle
                ),
                new WaitCommand(400),
                new AutoDropConeCommand(claw, slide, pivot, true),
                new ParallelCommandGroup(
                        new SlideResetUpAutonCommand(slide, pivot, claw, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(LeftSplineValues.DToConeOne.x2, LeftSplineValues.DToConeOne.y2), Math.toRadians(LeftSplineValues.DToConeOne.heading2), PoseStorage.cycle, true)   //Load
                )

        );
    }
}