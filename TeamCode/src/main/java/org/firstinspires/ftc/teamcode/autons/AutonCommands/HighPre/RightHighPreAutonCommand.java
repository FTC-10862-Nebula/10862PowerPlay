package org.firstinspires.ftc.teamcode.autons.AutonCommands.HighPre;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Front.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.Back.SlideHighBAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighPreAutonCommand extends SequentialCommandGroup {
    public RightHighPreAutonCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, -55),
                new TurnToCommand(drivetrain, 48, true),
                new SlideHighBAutoCommand(slide, arm, clawServos),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrain,-8.1),
                new WaitCommand(500),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain,8),
                new TurnToCommand(drivetrain, 90),
                new SlideResetAutonFCommand(slide, arm, clawServos)
//                new InstantCommand(arm::moveReset, arm),
        );

//        try{
//            addCommands(
//            new InstantCommand(slide::slideLow),
//            new InstantCommand(arm::moveIntakeFAuto),
//
//            new InstantCommand(clawServos::clawOpen)
//            );
//        }catch (Exception e){
//            addCommands(
//                    new InstantCommand(slide::slideLow),
//                    new InstantCommand(arm::moveIntakeFAuto),
//
//                    new InstantCommand(clawServos::clawOpen)
//            );
//        }
//        new Thread(
//                () -> {
//                    slide.slideLow();
//                    arm.moveIntakeFAuto();
//                    new WaitCommand(12);
//                    clawServos.clawOpen();
//
//                }
//        ).start();

//        new ThreadPool()
//        new Thread().destroy();
//        new Thread().stop();
//        new InstantCommand();
    }

}