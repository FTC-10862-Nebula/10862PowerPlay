package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
@Disabled
@Autonomous(group = "RED/BLUE")
public class SplineTestAutonTest extends MatchOpMode {
//    private ATDetector tagDetector;

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    // Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
    private Drivetrain drivetrain;


    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

    }

    public void matchStart() {
//        waitForStart();
        schedule(
                new SequentialCommandGroup(
                        new StrafeRightCommand(drivetrain, 52),
                        new StrafeRightCommand(drivetrain, 17),
//                        new SplinetoSplineCommand(drivetrain, new Pose2d(-30, -50), 0, true),
                        new DriveForwardCommand(drivetrain, -24.4),
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