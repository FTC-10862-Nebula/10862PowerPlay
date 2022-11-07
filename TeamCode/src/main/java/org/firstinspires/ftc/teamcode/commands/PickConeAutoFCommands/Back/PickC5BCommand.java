package org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Back;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickC5BCommand extends SequentialCommandGroup{
    public PickC5BCommand(Slide slide, ClawServos clawServos, ClawMotors clawMotors, Drivetrain drivetrain){
        addCommands(
                new InstantCommand(clawMotors::moveIntakeB),
                new InstantCommand(clawServos::clawOpen),
                new InstantCommand(slide::slideCone5),
                new SlowDriveForwardCommand(drivetrain, 5),
                new WaitCommand(200),
                new InstantCommand(clawServos::clawClose),
                new WaitCommand(50),
                new InstantCommand(slide:: slideLow)
        );
    }
}