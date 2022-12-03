package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDriveCorrect;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Autonomous(name = "TestAutonExample", group = "Test")
public class TestVisionEx extends MatchOpMode {
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
    private DrivetrainCOrrect drivetrainCorrect;
    private Slide slide;
    private Vision vision;

    @Override
    public void robotInit() {
        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        arm = new Arm(armMotor, telemetry, hardwareMap);
        drivetrainCorrect = new DrivetrainCOrrect(new SampleMecanumDriveCorrect(hardwareMap), telemetry, hardwareMap);
        drivetrainCorrect.init();
        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
        drivetrainCorrect.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

        vision = new Vision(hardwareMap, "Webcam 1", telemetry);
        while (!isStarted() && !isStopRequested())
        {
            vision.updateTagOfInterest();
            vision.tagToTelemetry();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
        tagNum = vision.getTag();

        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: {
                autonGroup = new SequentialCommandGroup(
                        //Commands
                );
            }
            case 2: {
                autonGroup = new SequentialCommandGroup(
                        //Commands
                );

            }
            case 3: {
                autonGroup =new SequentialCommandGroup(
                        //Commands
                );
            }
            default: {
               autonGroup = new SequentialCommandGroup(
                       //Default Commands
                );
            }
        }

        schedule(autonGroup);   //Schedule the Commands

    }
}