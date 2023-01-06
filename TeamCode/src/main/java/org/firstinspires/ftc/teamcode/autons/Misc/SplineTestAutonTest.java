package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Disabled
@Autonomous(group = "RED/BLUE")
public class SplineTestAutonTest extends MatchOpMode {

    // Subsystems
    private MecanumDrive drivetrain;

    @Override
    public void robotInit() {
        drivetrain = new MecanumDrive(hardwareMap, telemetry, false);
        drivetrain.init();
    }

    public void matchStart() {
//        waitForStart();
        schedule(
                new SequentialCommandGroup(
//                        new SplineCommand(drivetrain, new Vector2d(-24, -70), Math.toRadians(-360), true),
                        new WaitCommand(1000),
                        new SplineCommand(drivetrain, new  Vector2d(0, -68), Math.toRadians(320)),
                        new WaitCommand(1000),
                        new SplineCommand(drivetrain, new Vector2d(-24, -70), Math.toRadians(180)),
                        new WaitCommand(1000),
                        new SplineCommand(drivetrain, new  Vector2d(0, -68), Math.toRadians(320))
                )
        );
    }
};