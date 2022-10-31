package org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.driveTrain.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Autonomous(name = "Test2Mine", group = "RED")
public class Test2 extends MatchOpMode {
//    private ATDetector tagDetector;

    private static final double startPoseX = 0;
    private static final double startPoseY = 0;
    private static final double startPoseHeading = 0;
    private int tagNum = 0;

    //Motors and Servos
    private MotorEx clawMotor;
    private ServoEx clawS1, clawS3;
    //    private ServoEx clawS2;
    private CRServo clawS2;
    private MotorEx leftFront, leftRear, rightRear, rightFront;
    private MotorEx liftMotor1, liftMotor2;

    // Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
    private ClawMotors clawMotors;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;
    private Vision vision;


    @Override
    public void robotInit() {
        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        clawMotors = new ClawMotors(clawMotor, telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

        vision = new Vision(hardwareMap, "Webcam 1");
//        vision.init(hardwareMap);
    }

    public void matchStart() {
        tagNum = vision.getTag();
        if(tagNum==1)
        {
            new SequentialCommandGroup(
                            //Low
//                            new DriveForwardCommand(drivetrain, 12)
                    );
        }
        else if(tagNum==2)
        {
            new SequentialCommandGroup(
                    //Low
//                    new DriveForwardCommand(drivetrain, 12)
            );
        }
        else if(tagNum==3)
        {
            new SequentialCommandGroup(
                    //Low
//                    new DriveForwardCommand(drivetrain, 12)
            );
        }
        else
        {
            new SequentialCommandGroup(
                    //Low
//                    new DriveForwardCommand(drivetrain, 12)
            );
        }
//        schedule(
//                new SelectCommand(new HashMap<Object, Command>() {{
//
//                    put(1, new SequentialCommandGroup(
//                            //Low
//                            new DriveForwardCommand(drivetrain, 12)
//                    ));
//                    put(2, new SequentialCommandGroup(
//                            //Mid
//                            new DriveForwardCommand(drivetrain, 30)
//
//
//                    ));
//                    put(3, new SequentialCommandGroup(
//                            //High
//                            new DriveForwardCommand(drivetrain, 30)
//
//                    ));
//                }}, vision::getTag)
//        );
    }

    public void periodic(){
//        vision.tagToTelemetry();
    }
}