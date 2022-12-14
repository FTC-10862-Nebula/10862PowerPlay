package org.firstinspires.ftc.teamcode.autons.AutonCommands.Junctions;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Back.*;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.*;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighJunctionCommandNew extends SequentialCommandGroup{
    public RightHighJunctionCommandNew(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new SlideMidFCommand(slide, arm, clawServos, true),
                        new StrafeRightCommand(drivetrain, 51.)
                ),
                new DropAutoConeCommand(clawServos, slide, arm),
                new ParallelCommandGroup(
                        new StrafeRightCommand(drivetrain, 20.1),
                        new PrePickB5AutoCommand(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -26.85),



                /***Cone 5***/
                new PickC5BCommand(slide, clawServos),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 29)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, -61.6),
                        new DriveForwardCommand(drivetrain, 4.6)
                ),
                new ParallelCommandGroup(
                        new DropAutoConeCommand(clawServos, slide, arm),
                        new DriveForwardCommand(drivetrain, -4.73)
                ),
                new ParallelCommandGroup(
                        new TurnCommand(drivetrain, 61.7),
                        new PrePickB4AutoCommand(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -29.87),



                /***Cone 4***/
                new PickC4BCommand(slide, clawServos, arm, drivetrain),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 30)
                ),
                new SequentialCommandGroup(
                        new TurnToCommand(drivetrain, -62. , true),//oprg:300 to -60
                        new DriveForwardCommand(drivetrain, 4.5),
                        new DropAutoConeCommand(clawServos, slide, arm),
                        new DriveForwardCommand(drivetrain, -4.58)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 0, true),
//                        new TurnCommand(drivetrain, 62),
                        new PrePickB3AutoCommand(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -31.2),



                /***Cone 3***/
                new PickC3BCommand(slide, clawServos, arm, drivetrain),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 28)
                ),
                new TurnToCommand(drivetrain, -59, true),//oprg:300 to -60
                new DriveForwardCommand(drivetrain, 4.45),
                new DropAutoConeCommand(clawServos, slide, arm),



                //Parking
                new ParallelCommandGroup(
                        new SlideResetUpAutonCommand(slide, arm, clawServos),
                        new TurnToCommand(drivetrain, 270)
                ),
                new DriveForwardCommand(drivetrain, -4),
                new StrafeRightCommand(drivetrain, 32)
        );
    }
}