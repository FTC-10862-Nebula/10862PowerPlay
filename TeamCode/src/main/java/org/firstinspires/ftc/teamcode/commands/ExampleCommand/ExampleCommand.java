package org.firstinspires.ftc.teamcode.commands.ExampleCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ExampleCommand extends SequentialCommandGroup{
    public ExampleCommand(Drivetrain drivetrain, Slide slide,  Arm arm,ClawServos clawServos, double driveDistance){
//        addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
        addCommands(
//                //Commands
//                new DriveForwardCommand(drivetrain, driveDistance),
//new InstantCommand(slide::slideMid),
//new InstantCommand(arm::moveHighBAuto)
//                new InstantCommand(() ->
//                        new Thread(() -> {
////                            drivetrain.distanceCommand(2);
////                            new DriveForwardCommand(drivetrain, 2);
//                            clawServos.clawClose();
//                            slide.slideMid();
//                            arm.moveF();
//                        }).start())
        );
    }
}