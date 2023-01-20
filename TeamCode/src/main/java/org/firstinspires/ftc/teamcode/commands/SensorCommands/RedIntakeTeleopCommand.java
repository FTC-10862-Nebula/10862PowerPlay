package org.firstinspires.ftc.teamcode.commands.SensorCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class RedIntakeTeleopCommand extends ParallelCommandGroup {

    public RedIntakeTeleopCommand(Slide slide, Claw claw, SensorColor sensorColor) {
        addRequirements(sensorColor, claw, slide);
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup( //When True
                                new InstantCommand(claw::clawClose),
                                new InstantCommand(slide::slidePickUp)

                        ),
                        new SequentialCommandGroup( //When False
                        ),
                        sensorColor::grabbedRedCone
                )
        );
    }
}