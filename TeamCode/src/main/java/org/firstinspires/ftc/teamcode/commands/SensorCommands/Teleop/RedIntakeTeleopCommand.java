package org.firstinspires.ftc.teamcode.commands.SensorCommands.Teleop;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class RedIntakeTeleopCommand extends SequentialCommandGroup {
    public RedIntakeTeleopCommand(Slide slide, Claw claw, SensorColor sensorColor, Arm arm) {
        addRequirements(sensorColor, claw, slide, arm);
        addCommands(
                new WaitUntilCommand(sensorColor::grabbedRedCone).withTimeout(14),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(claw::clawClose),
                                new InstantCommand(arm::moveReset)
                        ),

                        new InstantCommand(),
                        ()-> (sensorColor.grabbedRedCone() && claw.isClawOpen())
                )
        );
    }
}