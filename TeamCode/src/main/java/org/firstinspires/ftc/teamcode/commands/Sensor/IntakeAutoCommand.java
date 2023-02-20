package org.firstinspires.ftc.teamcode.commands.Sensor;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.Auto.AutoConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class IntakeAutoCommand extends SequentialCommandGroup {

    public IntakeAutoCommand(Drivetrain drivetrain, Slide slide, Claw claw, SensorColor sensorColor, boolean isBack) {
        addRequirements(claw, sensorColor, slide, drivetrain);
        if (isBack){
            addCommands(
                    new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(9),
                    new ConditionalCommand(
                            new SequentialCommandGroup( //When True
//                                new InstantCommand(claw::clawClose)
                            ),
                            new SequentialCommandGroup( //When False
                                    new DriveForwardCommand(drivetrain, -0.6)
                            ),
                            ()-> ((sensorColor.grabbedBlueCone()||sensorColor.grabbedRedCone()) && claw.isClawOpen())
                    ),
                    new PickConeCommand(slide, claw)
            );
        } else{
            addCommands(
                    new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(9),
                    new ConditionalCommand(
                            new SequentialCommandGroup( //When True
//                                new InstantCommand(claw::clawClose)
                            ),
                            new SequentialCommandGroup( //When False
                                    new DriveForwardCommand(drivetrain, -0.6)
                            ),
                            ()-> ((sensorColor.grabbedBlueCone()||sensorColor.grabbedRedCone()) && claw.isClawOpen())
                    ),
                    new PickConeCommand(slide, claw)
            );
        }
    }
}