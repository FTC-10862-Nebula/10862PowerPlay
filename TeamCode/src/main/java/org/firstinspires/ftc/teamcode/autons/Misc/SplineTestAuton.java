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
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDriveCorrect;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;

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
    private DrivetrainCOrrect drivetrainCorrect;

    @Override
    public void robotInit() {
        drivetrainCorrect = new DrivetrainCOrrect(new SampleMecanumDriveCorrect(hardwareMap), telemetry, hardwareMap);
        drivetrainCorrect.init();
        drivetrainCorrect.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
    }

    public void matchStart() {
            schedule(
                new SequentialCommandGroup(
                        new SplineCommand(drivetrainCorrect, new Vector2d(-26, -30), -90)
                )
            );
    }
};