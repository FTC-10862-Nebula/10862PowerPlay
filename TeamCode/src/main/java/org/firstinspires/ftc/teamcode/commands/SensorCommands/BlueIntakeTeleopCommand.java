package org.firstinspires.ftc.teamcode.commands.SensorCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class BlueIntakeTeleopCommand extends ParallelCommandGroup {

    public BlueIntakeTeleopCommand(Slide slide, Claw claw, SensorColor sensorColor) {
        addRequirements(claw, sensorColor, slide);
        addCommands(
//                new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(9),
                new ConditionalCommand(
                        new SequentialCommandGroup( //When True
                                new InstantCommand(claw::clawClose),
                                new InstantCommand(slide::slidePickUp)
                        ),
                        new SequentialCommandGroup( /*When False*/
                        ),
                        sensorColor::grabbedBlueCone    //What they R checking
                )
        );
    }
}