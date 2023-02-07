package org.firstinspires.ftc.teamcode.commands.SensorCommands.Teleop;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class BlueIntakeTeleopCommand extends SequentialCommandGroup {

    public BlueIntakeTeleopCommand(Slide slide, Claw claw, SensorColor sensorColor, Arm arm) {
        addRequirements(claw, sensorColor, slide, arm);
        addCommands(
                new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(14),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(claw::clawClose),
                                new WaitCommand(200),
                                new InstantCommand(arm::moveReset)
                        ),
                        new InstantCommand(),
                        ()-> (sensorColor.grabbedBlueCone() && claw.isClawOpen() && arm.shouldSensorWork)
                )
        );
    }
}