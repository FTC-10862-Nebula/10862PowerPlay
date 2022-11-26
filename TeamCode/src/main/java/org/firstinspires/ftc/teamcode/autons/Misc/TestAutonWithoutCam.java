package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autons.AutonCommands.NewMultipleJunctions.LeftHigh2AutonCommandSideways;
import org.firstinspires.ftc.teamcode.autons.AutonCommands.NewMultipleJunctions.LeftHighAutonCommandSidewaysJUnctions;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Autonomous(name = "TestAutonWithoutCam", group = "RED/BLUE")
public class TestAutonWithoutCam extends MatchOpMode {
//    private ATDetector tagDetector;

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx armMotor;
    private ServoEx clawS1, clawS3;
    private ServoEx clawS2;
//    private CRServo clawS2;
//    private MotorEx leftFront, leftRear, rightRear, rightFront;
    private MotorEx liftMotor1, liftMotor2;

    // Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;


    @Override
    public void robotInit() {
        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        arm = new Arm(armMotor, telemetry, hardwareMap);
        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

        while (!isStarted() && !isStopRequested()){
            waitForStart();
        }
    }

    public void matchStart() {
        schedule(
//                new StrafeRightCommand(drivetrain, 10),
//                new WaitCommand(100),
//                new StrafeRightCommand(drivetrain, 13),
//                new WaitCommand(100),
//                new StrafeRightCommand(drivetrain, -10),
//                new WaitCommand(100),
//                new StrafeLeftCommand(drivetrain, 13)

                new LeftHighAutonCommandSidewaysJUnctions(drivetrain, slide, arm, clawServos)
//new LeftHigh2AutonCommandSideways(drivetrain, slide, arm, clawServos)
//               new LeftHighPrePlusOneAutonCommand(drivetrain, slide, arm, clawServos)



//new Sin
//                new RightHighPreAutonCommand(drivetrain, slide, arm, clawServos)

//                new SequentialCommandGroup(
//
////                        new InstantCommand(slide::slideLow),
////                        new InstantCommand(arm::moveIntakeFAuto),
//////                    new WaitCommand(12);
////                                new InstantCommand(clawServos::clawOpen),
////new RightHighPreAutonCommand(drivetrain, slide, arm, clawServos),
////                        new In vnstantCommand(arm::moveIntakeF),
//new DriveForwardCommand(drivetrain, 12),
////                        new InstantCommand(slide::slideMid),
////                        new InstantCommand(clawServos::clawClose),
////                        new InstantCommand(arm::moveIntakeF)
//new PrePickC5FCommand(slide, clawServos, arm)
////                        new RightHighPreAuton Command(drivetrain, slide, arm, clawServos),
////                        new DriveForwardCommand(drivetrain, -23),
////                        new TurnToCommand(drivetrain, 180)

//                RunExp1.main(drivetrain, slide, arm),
//        new WaitCommand(100),
//        RunExp2.main(drivetrain, slide, arm)



        );

    }
};