package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideResetFrontCommandT;


import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideGroundFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideLowFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideMidFrontCommand;
import org.firstinspires.ftc.teamcode.driveTrain.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrain.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Config
@TeleOp(name = "MainTeleop")
public class MainTeleOp extends MatchOpMode {

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx clawMotor;
    private ServoEx clawS1, clawS3;
    //    private ServoEx clawS2;
    private CRServo clawS2;
    private MotorEx leftFront, leftRear, rightRear, rightFront;
    private MotorEx liftMotor1, liftMotor2;

//    private Encoder leftEncoder, rightEncoder, frontEncoder;

    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;


    // Subsystems
    private ClawMotors clawMotors;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;
//    private StandardTrackingWheelLocalizer standardTrackingWheelLocalizer;
    //    private Vision vision;


    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        clawMotors = new ClawMotors(clawMotor, telemetry, hardwareMap);
        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
//        vision = new Vision(hardwareMap, "Webcam 1", telemetry);
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));


        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false));

//        standardTrackingWheelLocalizer = new StandardTrackingWheelLocalizer(leftEncoder, rightEncoder, frontEncoder, hardwareMap);
    }

    //Buttons
    private Button intakeTrigger, outtakeTrigger;
    private Button intakeButton, outtakeButton;
    private Button slowModeBumper;
    public Button resetFrontButton, resetBackButton;
    public Button plusClaw3Button, subClaw3Button;
    public Button plusClaw1Button, subClaw1Button;
    public Button s3FButton, s3BButton;


    public Button slideUpButton, slideDownButton, clawRaiseTrigger, clawLowerTrigger;
    public Button groundFSlideButton, lowFSlideButton, midFSlideButton, highFSlideButton;
    public Button clawMotorResetButton, slideEncoderResetButton;
    public Button robotDriveButton, fieldDriveButton;
    public Button setFlipTrueButton, setFlipFalseButton;


    @Override
    public void configureButtons() {
        robotDriveButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
                .whenPressed(new DefaultDriveCommand(drivetrain,driverGamepad,true));
        fieldDriveButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.BACK))
                .whenPressed(new DefaultDriveCommand(drivetrain,driverGamepad,false));

        //Slowmode - D1
            slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER))
                    .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));

        //Claw Servo Intake/Outtake - D1
            intakeTrigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                    .whenPressed(new PickConeCommand(clawServos));
            outtakeTrigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                    .whenPressed(new DropConeCommand(clawServos, slide));
        //Claw Servo 3 Buttons - D1
            s3FButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y))
                    .whenPressed(clawServos::setFClawPos);
            s3BButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                    .whenPressed(clawServos::setBClawPos);

        //reset everything - MOVE TO DRIVER2
//            resetFrontButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.B))
////                    .whenPressed(slide::slideResting);
//                    .whenPressed(new SlideResetFrontCommandT(slide, clawMotors, clawServos, clawMotors.getFlip()));

        //Claw Servo Manual Rotation
            plusClaw3Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP))
                    .whenPressed(clawServos::addClaw3Pos);
            subClaw3Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN))
                    .whenPressed(clawServos::subClaw3Pos);

        //Claw Servo Manual In/Out
            plusClaw1Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT))
                    .whenPressed(clawServos::addClaw1Pos);
            subClaw1Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT))
                    .whenPressed(clawServos::subClaw1Pos);



//        //Slide Manual - D2
            slideUpButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(slide::upSlideManual)
                    .whenReleased(slide::stopSlide));
            slideDownButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(slide::downSlideManual)
                    .whenReleased(slide::stopSlide));

        //Claw Manual - D2
            clawRaiseTrigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
                    .whenPressed(clawMotors::raiseClawManual)
                    .whenReleased(clawMotors::stopClaw));
            clawLowerTrigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
                    .whenPressed(clawMotors::lowerClawManual)
                    .whenReleased(clawMotors::stopClaw));

        //Slide positions
            setFlipTrueButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                    .whenPressed(new InstantCommand(clawMotors::setFlipTrue)));
            setFlipFalseButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new InstantCommand(clawMotors::setFlipFalse)));

            groundFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(new SlideGroundFrontCommand(slide, clawMotors, clawServos, clawMotors.getFlip())));
            lowFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new SlideLowFrontCommand(slide, clawMotors, clawServos, clawMotors.getFlip())));
            midFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new SlideMidFrontCommand(slide, clawMotors, clawServos, clawMotors.getFlip())));
            highFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(new SlideHighFrontCommand(slide, clawMotors, clawServos, clawMotors.getFlip())));

        //PIDF Controllers Resets
            clawMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                    .whenPressed(clawMotors::encoderReset);
            slideEncoderResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.BACK))
                    .whenPressed(slide::encoderReset);
    }

    @Override
    public void matchLoop() { }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){}
}
