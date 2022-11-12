package org.firstinspires.ftc.teamcode.autons.Autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autons.Commands.High.RightHighPreAutonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Autonomous(name = "RightHighPreAuton", group = "Test")
public class RightHighPreAuton extends MatchOpMode {
//    private ATDetector tagDetector;

    private static final double startPoseX = 0;
    private static final double startPoseY = 0;
    private static final double startPoseHeading = 0;
    private int tagNum = 0;

    //Motors and Servos
    private MotorEx armMotor;
    private ServoEx clawS1, clawS3;
    private ServoEx clawS2;
    //    private CRServo clawS2;
    private MotorEx leftFront, leftRear, rightRear, rightFront;
    private MotorEx liftMotor1, liftMotor2;

    //Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;
    private Vision vision;

    @Override
    public void robotInit() {
        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        arm = new Arm(armMotor, telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

        vision = new Vision(hardwareMap, "Webcam 1", telemetry);
//        vision.init(hardwareMap);
        while (!isStarted() && !isStopRequested())
        {
            vision.updateTagOfInterest();
            vision.tagToTelemetry();
            telemetry.update();
//            new InstantCommand(vision::getsend, vision);

        }
        this.matchStart();
    }

    public void matchStart() {tagNum = vision.getTag();

        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: { //Left
                schedule(
                        new SequentialCommandGroup(
                        new RightHighPreAutonCommand(drivetrain, slide, arm, clawServos),
                        new DriveForwardCommand(drivetrain, -25),
                        new TurnToCommand(drivetrain, 180)
                        )
                );
                return;
            }
            case 2: { //Mid
                schedule(
                        new SequentialCommandGroup(
                                new RightHighPreAutonCommand(drivetrain, slide, arm, clawServos),
                                new TurnToCommand(drivetrain, 180)
                        )
                );
                return;
            }
            default: { //High
                schedule(
                        new SequentialCommandGroup(
                                new RightHighPreAutonCommand(drivetrain, slide, arm, clawServos),
                                new DriveForwardCommand(drivetrain, 25),
                                new TurnToCommand(drivetrain, 180)
                        )
                );
                return;
            }
//            default: {
//                schedule(
//                        new SequentialCommandGroup(
//                                new RightHighPreAutonCommand(drivetrain, slide, arm, clawServos),
//                                new DriveForwardCommand(drivetrain, -23),
//                                new TurnToCommand(drivetrain, 180)
//                        )
//                );
//                return;
//            }
        }

//        schedule(autonGroup);
    }
}