package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideGroundBackCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideHighBackCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideLowBackCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideMidBackCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideResetBackCommandT;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideGroundFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideLowFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideMidFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideResetFrontCommandT;
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
    private ServoEx clawS1, clawS2, clawS3;
    private MotorEx leftFront, leftRear, rightRear, rightFront;
    private MotorEx liftMotor1, liftMotor2;

    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;


    // Subsystems
    private ClawMotors clawMotors;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;
//    private Vision vision;


    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        clawMotors = new ClawMotors(clawMotor, telemetry, hardwareMap);
        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry);
        drivetrain.init();
        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
//        vision = new Vision(hardwareMap, "Webcam 1", telemetry);
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad));
    }

    //Buttons
    private Button intakeTrigger, outtakeTrigger;
    private Button intakeButton, outtakeButton;
    private Button slowModeBumper;
    public Button resetFrontButton, resetBackButton;
    public Button plusClaw3Button, subClaw3Button;
    public Button plusClaw1Button, subClaw1Button;
    public Button s3FButton, s3BButton;


    public Button slideUpButton, slideDownButton, clawRaiseButton, clawLowerButton;
    public Button groundBSlideButton, lowBSlideButton, midBSlideButton, highBSlideButton;
    public Button groundFSlideButton, lowFSlideButton, midFSlideButton, highFSlideButton;
    public Button clawMotorResetButton, slideEncoderResetButton;
    public Button one, two;


    @Override
    public void configureButtons() {
        //Slowmode - D1
            slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER))
                    .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));

//        //Claw Servo Intake/Outtake - D1
//            intakeTrigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
//                    .whenPressed(new PickConeCommand(clawServos));
//            outtakeTrigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
//                    .whenPressed(new DropConeCommand(clawServos));
        //Claw Servo 3 Buttons - D1
            s3FButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y))
                    .whenPressed(clawServos::setFClawPos);
            s3BButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                    .whenPressed(clawServos::setBClawPos);

        //reset everything
            resetBackButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.X))
                    .whenPressed(slide::slideResting);
//                    .whenPressed(new SlideResetBackCommandT(slide, clawMotors, clawServos));
            resetFrontButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.B))
                    .whenPressed(slide::slideResting);
//                    .whenPressed(new SlideResetFrontCommandT(slide, clawMotors, clawServos));

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



        //Intake Button - D2
        intakeButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(new PickConeCommand(clawServos));
        outtakeButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(new DropConeCommand(clawServos));

//        //Slide Manual - D2
//            slideUpButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
//                    .whenPressed(slide::upSlideManual)
//                    .whenReleased(slide::stopSlide));
//            slideDownButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
//                    .whenPressed(slide::downSlideManual)
//                    .whenReleased(slide::stopSlide));

        //Claw Manual - D2
            clawRaiseButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(clawMotors::raiseClawManual)
                    .whenReleased(clawMotors::stopClaw));
            clawLowerButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(clawMotors::lowerClawManual)
                    .whenReleased(clawMotors::stopClaw));

        //Slide positions
            groundBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
                    .whenPressed(new SlideGroundBackCommand(slide, clawMotors, clawServos)));
            lowBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                    .whenPressed(new SlideLowBackCommand(slide, clawMotors, clawServos)));
            midBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
                    .whenPressed(new SlideMidBackCommand(slide, clawMotors, clawServos)));
//            highBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
//                    .whenPressed(new SlideHighBackCommand(slide, clawMotors, clawServos)));

            groundFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(new SlideGroundFrontCommand(slide, clawMotors, clawServos)));
            lowFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new SlideLowFrontCommand(slide, clawMotors, clawServos)));
            midFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new SlideMidFrontCommand(slide, clawMotors, clawServos)));
//            highFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
//                    .whenPressed(new SlideHighFrontCommand(slide, clawMotors, clawServos)));

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
    public void robotPeriodic(){ }
}
