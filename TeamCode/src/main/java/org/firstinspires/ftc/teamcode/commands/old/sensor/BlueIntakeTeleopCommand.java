package org.firstinspires.ftc.teamcode.commands.old.sensor;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
@Deprecated
public class BlueIntakeTeleopCommand extends SequentialCommandGroup {

    public BlueIntakeTeleopCommand(Slide slide, Claw claw, SensorColor sensorColor, Pivot pivot) {
        addRequirements(claw, sensorColor, slide, pivot);
        addCommands(
                new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(14),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(claw::clawClose)
//                                new WaitCommand(200),
//                                new InstantCommand(arm::moveReset)
                        ),
                        new InstantCommand(),
                        ()-> (sensorColor.grabbedBlueCone() && claw.isClawOpen() && pivot.shouldSensorWork)
                )
        );
    }
}