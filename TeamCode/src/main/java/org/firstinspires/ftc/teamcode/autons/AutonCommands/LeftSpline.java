package org.firstinspires.ftc.teamcode.autons.AutonCommands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.MuliplyCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Pick.PickCFCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickB3Command;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickB4Command;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePick5FCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideMidBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


public class LeftSpline extends SequentialCommandGroup{
    public LeftSpline(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new SplineCommand(drivetrain, new Vector2d(58, -8.5), Math.toRadians(337))    //Cycle
                ),
                new DropAutoConeCommand(clawServos, slide, arm, true),
                new ParallelCommandGroup(
                        new PrePickB5Command(slide, clawServos, arm),
                        new SplineCommand(drivetrain, new Vector2d(55.8, 25), Math.toRadians(90), true)   //Load
                ),


                new MuliplyCommand(4,
                        new ParallelCommandGroup(
                                new SlideHighFCommand(slide, arm, clawServos, true),
                                new SplineCommand(drivetrain, new Vector2d(58, -8.5), Math.toRadians(337), PoseStorage.cycle)    //Cycle
                        ),
                        new DropAutoConeCommand(clawServos, slide, arm, true),
                        new ParallelCommandGroup(
                                new PrePickB5Command(slide, clawServos, arm),
                                new SplineCommand(drivetrain, new Vector2d(55.8, 25), Math.toRadians(90), PoseStorage.load, true)   //Load
                        )
                ),


                /**Cone 1**/
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new SplineCommand(drivetrain, new Vector2d(58, -8.5), Math.toRadians(337), PoseStorage.cycle)    //Cycle
                ),
                new DropAutoConeCommand(clawServos, slide, arm, true),

                new TurnToCommand(drivetrain, 0)
        );
    }
}