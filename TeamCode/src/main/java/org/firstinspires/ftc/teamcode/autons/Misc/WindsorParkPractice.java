package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDriveCorrect;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Disabled
@Autonomous(name = "WindsorParkPRACTICE", group = "RED/BLUE")
public class WindsorParkPractice extends MatchOpMode {
//    private ATDetector tagDetector;

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx clawMotor;
    private ServoEx clawS1, clawS3;
        private ServoEx clawS2;
//    private CRServo clawS2;
    private MotorEx leftFront, leftRear, rightRear, rightFront;
    private MotorEx liftMotor1, liftMotor2;

    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;


    @Override
    public void robotInit() {
        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDriveCorrect(hardwareMap), telemetry, hardwareMap);
        arm = new Arm(clawMotor, telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

    }

@Override
    public void matchStart() {
            schedule(
                    new SequentialCommandGroup(
                        new DriveForwardCommand(drivetrain, 50),
                        new TurnCommand(drivetrain, 90),
                        new DriveForwardCommand(drivetrain, 50),
                        new TurnCommand(drivetrain, 90),
                        new DriveForwardCommand(drivetrain, 50),
                        new TurnCommand(drivetrain, 90),
                        new DriveForwardCommand(drivetrain, 50),
                        new TurnCommand(drivetrain, 90)
                    )
            );
    }
};