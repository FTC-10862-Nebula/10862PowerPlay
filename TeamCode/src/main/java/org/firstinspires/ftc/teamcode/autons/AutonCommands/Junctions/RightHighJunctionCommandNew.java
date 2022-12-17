package org.firstinspires.ftc.teamcode.autons.AutonCommands.Junctions;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Pick.*;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.*;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighJunctionCommandNew extends SequentialCommandGroup{
    public RightHighJunctionCommandNew(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        /*Turn is Counterclockwise*/
        addCommands(
//                new DropAutoConeCommand(clawServos, slide, arm,true),

                new ParallelCommandGroup(
                        new SlideMidFCommand(slide, arm, clawServos, true),
                        new StrafeRightCommand(drivetrain, 53.15)
                ),
                new DropAutoConeCommand(clawServos, slide, arm,true),
                new ParallelCommandGroup(
                        new StrafeRightCommand(drivetrain, 20.8),
                        new PrePickB5Command(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -26.85),



                /***Cone 5***/
                new PickCBCommand(slide, clawServos),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 29)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, -60),//61 ish
                        new DriveForwardCommand(drivetrain, 4.5),
                        new DropAutoConeCommand(clawServos, slide, arm,true)
//                        new DriveForwardCommand(drivetrain, -0.5)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 0.8, true),
                        new PrePickB4Command(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -31.6),



                /***Cone 4***/
                new PickCBCommand(slide, clawServos),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 30)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, -58),//oprg:300 to -60
                        new DriveForwardCommand(drivetrain, 3),
                        new DropAutoConeCommand(clawServos, slide, arm,true)
//                        new DriveForwardCommand(drivetrain, -0.5)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(drivetrain, 1.2, true),
                        new PrePickB3Command(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -32.6),



                /***Cone 3***/
                new PickCBCommand(slide, clawServos),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 28.8)
                ),
                new TurnCommand(drivetrain, -55.45),//oprg:300 to -60
                new DriveForwardCommand(drivetrain, 4),
                new DropAutoConeCommand(clawServos, slide, arm,true),




                //Parking
                new ParallelCommandGroup(
                        new SlideResetUpAutonCommand(slide, arm, clawServos),
                        new TurnToCommand(drivetrain, 270)
                ),
                new DriveForwardCommand(drivetrain, -4)
//                new StrafeRightCommand(drivetrain, 40)
        );
    }
}