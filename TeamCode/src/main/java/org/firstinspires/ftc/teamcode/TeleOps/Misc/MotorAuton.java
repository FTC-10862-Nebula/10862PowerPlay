package org.firstinspires.ftc.teamcode.TeleOps.Misc;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Misc.MotorSubsystem;

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