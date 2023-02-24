package org.firstinspires.ftc.teamcode.commands.old.auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.arm.intake.AutoPickConeCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.arm.outtake.AutoDropConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone3BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone4BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.old.auto.ArmCone5FrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.ArmHighBackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.ArmMidBackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Deprecated
public class LeftHighJunctionCommand extends SequentialCommandGroup{
    public LeftHighJunctionCommand(Drivetrain drivetrain, Slide slide, Pivot pivot, Claw claw, TurnServo turnServo){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new ArmMidBackCommand(slide, pivot, claw, turnServo, true),
                        new StrafeRightCommand(drivetrain, 51.9)
                ),
                new AutoDropConeCommand(claw, slide, pivot,true),
                new ParallelCommandGroup(
                        new StrafeRightCommand(drivetrain, 20.6),
                        new ArmCone5FrontCommand(slide, claw, pivot, turnServo)
                ),
                new DriveForwardCommand(drivetrain, 26.85),



                /***Cone 5***/
                new AutoPickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new ArmHighBackCommand(slide, pivot, claw, turnServo, true),
                        new DriveForwardCommand(drivetrain, 29)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, 52.5),//61 ish
                        new DriveForwardCommand(drivetrain, -2.5),
                        new AutoDropConeCommand(claw, slide, pivot,true),
                        new DriveForwardCommand(drivetrain, 4.88)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 1, true),
                        new ArmCone4BackCommand(slide, claw, pivot, turnServo)
                ),
                new DriveForwardCommand(drivetrain, 29.9),



                /***Cone 4***/
                new AutoPickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new ArmHighBackCommand(slide, pivot, claw, turnServo, true),
                        new DriveForwardCommand(drivetrain, -30)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, 50.5),//oprg:300 to -60
                        new DriveForwardCommand(drivetrain, -2.3),
                        new AutoDropConeCommand(claw, slide, pivot,true),
                        new DriveForwardCommand(drivetrain, 4.5)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 1.5, true),
                        new ArmCone3BackCommand(slide, claw, pivot, turnServo)
                ),
                new DriveForwardCommand(drivetrain, 30.5),



                /***Cone 3***/
                new AutoPickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new ArmHighBackCommand(slide, pivot, claw, turnServo, true),
                        new DriveForwardCommand(drivetrain, -28.8)
                ),
                new TurnCommand(drivetrain, -46.5),//oprg:300 to -60
                new DriveForwardCommand(drivetrain, 2.3),
                new AutoDropConeCommand(claw, slide, pivot,true),



                //Parking - Remove!!!!!!!!!!!!
                new ParallelCommandGroup(
                        new SlideResetUpAutonCommand(slide, pivot, claw, turnServo),
                        new TurnToCommand(drivetrain, 270)
                ),
                new DriveForwardCommand(drivetrain, -7),
                new StrafeLeftCommand(drivetrain, 36)
        );
    }
}