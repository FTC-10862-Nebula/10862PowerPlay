package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Treads.thing.Drivethreadcomment;
import org.firstinspires.ftc.teamcode.Treads.thing.Drivethreadcomment22;
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

//    //Motors and Servos
//    private MotorEx armMotor;
//    private ServoEx clawS1, clawS3;
//    private ServoEx clawS2;
////    private CRServo clawS2;
////    private MotorEx leftFront, leftRear, rightRear, rightFront;
//    private MotorEx liftMotor1, liftMotor2;

    // Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;


    @Override
    public void robotInit() {
        clawServos = new ClawServos( telemetry, hardwareMap);
        arm = new Arm( telemetry, hardwareMap);
        slide = new Slide(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

        while (!isStarted() && !isStopRequested()){
            waitForStart();
        }
    }

    public void matchStart() {
        waitForStart();
        schedule(
                new Drivethreadcomment(drivetrain, slide, arm, clawServos),
                new WaitCommand(1000),
                new Drivethreadcomment22(drivetrain, slide, arm, clawServos)
//                RunExp1.main(drivetrain, slide, arm),
//                new WaitCommand(1500),
//                RunExp2.main(drivetrain, slide, arm)
        );
                new SequentialCommandGroup(

//                        RunExp1.main(drivetrain, slide, arm);

//                        new StrafeRightCommand(drivetrain, 12)
//                        new RightHighAutonCommandSidewaysJUnctions(drivetrain, slide, arm, clawServos),
//                        new StrafeRightCommand(drivetrain, 28)

//                new LeftHighAutonCommandSidewaysJUnctions(drivetrain, slide, arm, clawServos)
//              new LeftHigh2AutonCommandSideways(drivetrain, slide, arm, clawServos)
//               new LeftHighPrePlusOneAutonCommand(drivetrain, slide, arm, clawServos)


//        )
        );

    }
};