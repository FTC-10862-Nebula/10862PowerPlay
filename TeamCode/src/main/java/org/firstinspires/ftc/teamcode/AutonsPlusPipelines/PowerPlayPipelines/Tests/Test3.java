package org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveTrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Misc.VisionSubsystemNOUSE;

@Autonomous
public class Test3 extends LinearOpMode {
    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;
    //Motors and Servos
    MotorEx clawMotor;
    ServoEx clawS1, clawS3;
    //    private ServoEx clawS2;
    private CRServo clawS2;
    MotorEx leftFront, leftRear, rightRear, rightFront;
    MotorEx liftMotor1, liftMotor2;

    // Subsystems
    ClawMotors clawMotors;
    ClawServos clawServos;
    Drivetrain drivetrain;
    Slide slide;
    //       Vision vision;


    @Override
    public void runOpMode() throws InterruptedException {
        VisionSubsystemNOUSE vision = new VisionSubsystemNOUSE(this);
        ClawServos clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        Drivetrain drivetrain =new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        Slide slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);

        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
//        vision.init();

        while (!isStarted() && !isStopRequested())
        {
           vision.updateTagOfInterest();
           vision.printTagData();
           telemetry.update();
        }

        switch (vision.getTagOfInterest().id) {
            case 1: {
                new SequentialCommandGroup(
//                        new DriveForwardCommand(drivetrain, 30)
                );

//                new LeftMidAutonCommand(drivetrain, slide, clawMotors, clawServos);
            }
            case 2: {
                new SequentialCommandGroup(
//                        new DriveForwardCommand(drivetrain, 30)
                );

//                new LeftMidAutonCommand(drivetrain, slide, clawMotors, clawServos);

            }
            case 3: {
                new SequentialCommandGroup(
//                        new DriveForwardCommand(drivetrain, 30)
                );

//                new LeftMidAutonCommand(drivetrain, slide, clawMotors, clawServos);

            }
        }
    }
}