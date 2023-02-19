package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowSplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoConeCommands.Pick.PickCFCommand;
import org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoConeCommands.PrePickBConeCommands.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.util.LeftSplineValues;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


public class LeftSpline extends SequentialCommandGroup{
    public LeftSpline(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, claw, turnServo, true),
                        new SplineCommand(drivetrain, new Vector2d(LeftSplineValues.AToHighOne.x1,LeftSplineValues.AToHighOne.y1), Math.toRadians(LeftSplineValues.AToHighOne.heading1))    //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo)  ,
                        new SlowSplineCommand(drivetrain, new Vector2d(LeftSplineValues.BToConeOne.x, LeftSplineValues.BToConeOne.y2), Math.toRadians(LeftSplineValues.BToConeOne.heading2), true)   //Load
                ),
                new PickCFCommand(slide, claw),
//        new IntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),


                new ParallelCommandGroup(
                        new SlideMidFCommand(slide, arm, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(LeftSplineValues.BToMid.x3,LeftSplineValues.BToMid.y3), Math.toRadians(LeftSplineValues.BToMid.heading3))   //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, claw, arm, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(LeftSplineValues.CToConeOne.x2,LeftSplineValues.CToConeOne.y2), Math.toRadians(LeftSplineValues.CToConeOne.heading2), true)   //Load
                ),
                new PickCFCommand(slide, claw),
//        new IntakeAutoCommand(drivetrain, slide, claw, sensorColor, true),



                new ParallelCommandGroup(
                        new SlideMidFCommand(slide, arm, claw, turnServo,true),
                        new SplineCommand(drivetrain, new Vector2d(LeftSplineValues.CToMid.x3,LeftSplineValues.CToMid.y3), Math.toRadians(LeftSplineValues.CToMid.heading3))   //Cycle
                ),
                new DropAutoConeCommand(claw, slide, arm, true),
                new ParallelCommandGroup(
                        new SlideResetUpAutonCommand(slide, arm, claw, turnServo),
                        new SlowSplineCommand(drivetrain, new Vector2d(LeftSplineValues.DToConeOne.x2, LeftSplineValues.DToConeOne.y2), Math.toRadians(LeftSplineValues.DToConeOne.heading2), PoseStorage.cycle, true)   //Load
                )

        );
    }
}