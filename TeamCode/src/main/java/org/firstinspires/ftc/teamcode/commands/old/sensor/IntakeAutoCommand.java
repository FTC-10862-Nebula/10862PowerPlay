package org.firstinspires.ftc.teamcode.commands.old.sensor;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.arm.intake.AutoPickConeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
@Deprecated
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
                    new AutoPickConeCommand(slide, claw)
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
                    new AutoPickConeCommand(slide, claw)
            );
        }
    }
}