package org.firstinspires.ftc.teamcode.autons.Autons.Right;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autons.AutonCommands.NewMultipleJunctions.RightHighAutonCommandSidewaysJUnctions;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDriveCorrect;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Autonomous(name = "RightJunctionAuton", group = "Test")
public class RightJunctionAuton extends MatchOpMode {
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

    public void matchStart() {tagNum = vision.getTag();

//        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: { //Left
                schedule(
                        new SequentialCommandGroup(
                                new RightHighAutonCommandSidewaysJUnctions(drivetrainCorrect, slide, arm, clawServos),
                                new StrafeLeftCommand(drivetrainCorrect, 17.5)
                        )
                );
                return;
            }
            case 2: { //Mid
                schedule(
                        new SequentialCommandGroup(
                                new RightHighAutonCommandSidewaysJUnctions(drivetrainCorrect, slide, arm, clawServos),
                                new StrafeRightCommand(drivetrainCorrect, 13.5)
                        )
                );
                return;
            }
            default: { //High
                schedule(
                        new SequentialCommandGroup(
                                new RightHighAutonCommandSidewaysJUnctions(drivetrainCorrect, slide, arm, clawServos),
                                new StrafeRightCommand(drivetrainCorrect, 44)
                        )
                );
                return;
            }
        }
    }
}