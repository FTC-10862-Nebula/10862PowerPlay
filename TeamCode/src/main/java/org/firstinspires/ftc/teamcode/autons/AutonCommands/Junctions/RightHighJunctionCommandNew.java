package org.firstinspires.ftc.teamcode.autons.AutonCommands.Junctions;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Back.PickC3BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Back.PickC4BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Back.PickC5BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideLowFCommand;
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
//                new WaitCommand(300),
                new ParallelCommandGroup(
                        new StrafeRightCommand(drivetrain, 20.1),
                        new PrePickBAutoCommand(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -26.8),
//                new TurnToCommand(drivetrain, 0),

                /***Cone 5***/

                new PickC5BCommand(slide, clawServos, arm, drivetrain),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 29)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(drivetrain, -71.8),
                        new DriveForwardCommand(drivetrain, 4),
                        new DropAutoConeCommand(clawServos, slide, arm),
                        new DriveForwardCommand(drivetrain, -3.8)
                ),
                new ParallelCommandGroup(
//                        new TurnToCommand(drivetrain, 2.4, true),
                        new TurnCommand(drivetrain, 68.5),
                        new PrePickBAutoCommand(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -33.5),


                /***Cone 4***/
                new PickC4BCommand(slide, clawServos, arm, drivetrain),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 28.8)
                ),
                new SequentialCommandGroup(
                        new TurnToCommand(drivetrain, -68.7 , true),//oprg:300 to -60
                        new DriveForwardCommand(drivetrain, 3.1),
                        new DropAutoConeCommand(clawServos, slide, arm),
                        new DriveForwardCommand(drivetrain, -3.8)
                ),
                new ParallelCommandGroup(
        //                        new TurnToCommand(drivetrain, 2.4, true),
                        new TurnCommand(drivetrain, 68.5),
                        new PrePickBAutoCommand(slide, clawServos, arm)
                ),
                new DriveForwardCommand(drivetrain, -33.5),


                /***Cone 3***/
                new PickC3BCommand(slide, clawServos, arm, drivetrain),
                new ParallelCommandGroup(
                        new SlideHighFCommand(slide, arm, clawServos, true),
                        new DriveForwardCommand(drivetrain, 26.5)
                ),
                new TurnToCommand(drivetrain, -68.7 , true),//oprg:300 to -60
                new DriveForwardCommand(drivetrain, 3.1),
                new DropAutoConeCommand(clawServos, slide, arm),



                //Parking
                new TurnToCommand(drivetrain, 270),
                new StrafeRightCommand(drivetrain, 25)
        );
    }
}