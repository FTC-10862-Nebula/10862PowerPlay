package org.firstinspires.ftc.teamcode.commands.ExampleCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class FaithCommand extends SequentialCommandGroup{
    public FaithCommand(Slide slide, ClawMotors clawMotors, Drivetrain drivetrain ){
//        addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
        addCommands(
                //Commands
//        new InstantCommand(()-> new clawMotors.moveMidF((clawMotors.getFlip())));,
//        new InstantCommand(slide::slideMid),
      new InstantCommand(slide::slideMid)   ,
//          new InstantCommand(drivetrain::)

                new DriveForwardCommand(drivetrain, 30)
        );
    }
}