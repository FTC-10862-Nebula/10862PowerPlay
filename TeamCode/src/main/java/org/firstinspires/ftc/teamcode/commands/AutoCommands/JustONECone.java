package org.firstinspires.ftc.teamcode.commands.AutoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndOutake.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;


public class JustONECone extends SequentialCommandGroup{
    public JustONECone(Drivetrain drivetrain, Slide slide, Arm arm, Claw claw, TurnServo turnServo, SensorColor sensorColor){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new DriveForwardCommand(drivetrain, 53.91),
                        new SlideHighFCommand(slide, arm, claw, turnServo, true)
                        ),
                new StrafeLeftCommand(drivetrain, 7.52),

                new DropAutoConeCommand(claw, slide, arm, true),
                new WaitCommand(200),

                new SlideResetUpAutonCommand(slide, arm, claw, turnServo)
                );
    }
}