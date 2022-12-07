package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Disabled
@Autonomous(name = "SplineTests", group = "RED/BLUE")
public class SplineTestAuton extends MatchOpMode {
//    private ATDetector tagDetector;

    public static double startPoseX = 0;
    public static double startPoseY = 0;
    public static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx leftFront, leftRear, rightRear, rightFront;
    // Gamepad
    private GamepadEx driverGamepad;
    // Subsystems
    private Drivetrain drivetrain;

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
    }

    public void matchStart() {
            schedule(
                new SequentialCommandGroup(
                        new SplineCommand(drivetrain, new Vector2d(-26, -30), -90)
                )
            );
    }
};