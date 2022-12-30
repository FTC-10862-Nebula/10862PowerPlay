package org.firstinspires.ftc.teamcode.TeleOps.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autons.AutonCommands.RightHighJunctionCommandNew;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultArmCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

public class MotorAuton extends MatchOpMode {
//    private ATDetector tagDetector;

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    // Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
    private MotorSubsystem motorSubsystem;

    @Override
    public void robotInit() {
        motorSubsystem = new MotorSubsystem(telemetry, hardwareMap);
    }

    public void matchStart() {
//        waitForStart();
        schedule(
                new InstantCommand(motorSubsystem::setUpSpeed),
                new WaitCommand(3000),
                new InstantCommand(motorSubsystem::stop),
                new WaitCommand(5000),
                new InstantCommand(motorSubsystem::setDownSpeed),
                new WaitCommand(3000),
                new InstantCommand(motorSubsystem::stop)

                );
    }


};