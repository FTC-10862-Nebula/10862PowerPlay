package org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.NewVisionStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.AutonPathings.Commands.LeftMidAutonCommand;
import org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.AutonPathings.Commands.RightHighAutonCommand;
import org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.AutonPathings.Commands.RightMidAutonCommand;
import org.firstinspires.ftc.teamcode.driveTrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Stuff.VisionSubsystem;

@Autonomous
public class VisionTest2  extends LinearOpMode {
    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;
    //Motors and Servos
    MotorEx clawMotor;
    ServoEx clawS1, clawS2, clawS3;
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
        VisionSubsystem vision = new VisionSubsystem(this);
        ClawServos clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        Drivetrain drivetrain =new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry);
        Slide slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);


        //Subsytem Init
//        vision = new Vision(hardwareMap, "Webcam 1", telemetry);
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
        vision.init();

        while (!isStarted() && !isStopRequested())
        {
           vision.updateTagOfInterest();
           vision.printTagData();
           telemetry.update();
        }
        switch (vision.getTagOfInterest().id) {
            case 1: {
                new LeftMidAutonCommand(drivetrain, slide, clawMotors, clawServos);
            }
            case 2: {
                new LeftMidAutonCommand(drivetrain, slide, clawMotors, clawServos);

            }
            case 3: {
                new LeftMidAutonCommand(drivetrain, slide, clawMotors, clawServos);

            }
        }
    }
}