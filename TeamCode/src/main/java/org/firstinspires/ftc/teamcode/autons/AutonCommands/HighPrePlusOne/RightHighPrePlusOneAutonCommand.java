package org.firstinspires.ftc.teamcode.autons.AutonCommands.HighPrePlusOne;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Front.PickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Back.SlideLowBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Back.SlideHighBAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighPrePlusOneAutonCommand extends SequentialCommandGroup{
    public RightHighPrePlusOneAutonCommand(DrivetrainCOrrect drivetrainCorrect, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrainCorrect, -55),
                new TurnToCommand(drivetrainCorrect, 47, true),
                new SlideHighBAutoCommand(slide, arm, clawServos),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrainCorrect,-8.1),
                new WaitCommand(500),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(200),

                new SlowDriveForwardCommand(drivetrainCorrect,8),
                new PrePickC5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrainCorrect, 97),
                new DriveForwardCommand(drivetrainCorrect, 25),

                new PickC5FCommand(slide, clawServos, arm, drivetrainCorrect),
                new SlideLowBAutoCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrainCorrect, 148),
                new SlowDriveForwardCommand(drivetrainCorrect, -2),
//                new InstantCommand(clawServos::clawOpen, clawServos),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrainCorrect, 3),
                new TurnToCommand(drivetrainCorrect, 90),


                new SlideResetAutonFCommand(slide, arm, clawServos),
                new WaitCommand(200)
//                new InstantCommand(arm::moveReset, arm),
        );
    }
}