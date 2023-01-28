package org.firstinspires.ftc.teamcode.commands.SensorCommands.Auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Pick.PickCBCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Pick.PickCFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class RedIntakeAutoCommand extends SequentialCommandGroup {

    public RedIntakeAutoCommand(Drivetrain drivetrain, Slide slide, Claw claw, SensorColor sensorColor, boolean isBack) {
        addRequirements(sensorColor, claw, slide, drivetrain);
        if (isBack){
            addCommands(
                    new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(9),
                    new ConditionalCommand(
                            new SequentialCommandGroup( //When True
//                                new InstantCommand(claw::clawClose)
                            ),
                            new SequentialCommandGroup( //When False
                                    new DriveForwardCommand(drivetrain, -2)
                            ),
                            ()-> (sensorColor.grabbedBlueCone() && claw.isClawOpen())
                    ),
                    new PickCBCommand(slide, claw)
            );
        } else{
            addCommands(
                    new WaitUntilCommand(sensorColor::grabbedBlueCone).withTimeout(9),
                    new ConditionalCommand(
                            new SequentialCommandGroup( //When True
//                                new InstantCommand(claw::clawClose)
                            ),
                            new SequentialCommandGroup( //When False
                                    new DriveForwardCommand(drivetrain, -2)
                            ),
                            ()-> (sensorColor.grabbedBlueCone() && claw.isClawOpen())
                    ),
                    new PickCFCommand(slide, claw)
            );
        }
    }
}