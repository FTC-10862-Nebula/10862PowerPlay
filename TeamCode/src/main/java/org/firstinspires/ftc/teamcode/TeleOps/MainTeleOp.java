package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private CRServo clawS2;
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
    }

    //Buttons
    private Button intakeTrigger, outtakeTrigger;
    private Button slowModeBumper;
    public Button slideUpButton, slideDownButton;
    public Button groundBSlideButton, lowBSlideButton, midBSlideButton, highBSlideButton;
    public Button groundFSlideButton, lowFSlideButton, midFSlideButton, highFSlideButton;
    public Button resetEveryThingButton, openClawButton, clawMotorResetButton, slideEncoderReset;

    public Button one, two, three, four;

    @Override
    public void configureButtons() {
        one = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
                .whenPressed(clawServos::intakeClaw)
                .whenPressed(clawServos::clawClose)
                .whenReleased(clawServos::stopClaw)
                );
        two = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
                //.whenPressed(clawServos::outtakeClaw)
                .whenPressed(clawServos::clawOpen)
                .whenReleased(clawServos::stopClaw));

        three = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(clawServos::addClawPos));
//        one = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
//                    .whenPressed(clawMotors::moveClawGroundFront));
//        two = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
//                    .whenPressed(clawMotors::moveClawLowFront));
//        three = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
//                   .whenPressed(clawMotors::moveClawMidFront));
//        four = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
//                .whenPressed(clawMotors::moveClawHighFront));

        //slowmode for the drivetrain
//            slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER))
//                    .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));

//        //Slide Manual
//            slideUpButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
//                    .whenPressed(slide::upSlideManual).whenReleased(slide::stopSlide));
//            slideDownButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
//                    .whenPressed(slide::downSlideManual).whenReleased(slide::stopSlide));

        //Slide positions
//            groundBackSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
//                    .whenPressed(new SlideGroundBackCommand(slide, clawMotors)));
//            lowBackSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
//                    .whenPressed(new SlideLowBackCommand(slide, clawMotors)));
//            midBackSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
//                    .whenPressed(new SlideMidBackCommand(slide, clawMotors)));
//            highBackSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
//                    .whenPressed(new SlideHighBackCommand(slide, clawMotors, clawServos)));
//
//            groundFrontSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
//                    .whenPressed(new SlideGroundFrontCommand(slide, clawMotors)));
//            lowFrontSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
//                    .whenPressed(new SlideLowFrontCommand(slide, clawMotors)));
//            midFrontSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
//                    .whenPressed(new SlideMidFrontCommand(slide, clawMotors)));
//            highFrontSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
//                    .whenPressed(new SlideHighFrontCommand(slide, clawMotors, clawServos)));

        //reset everything
//            resetEveryThingButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN))
//                    .whenPressed(new SlideResetBackCommandT(slide, clawMotors, clawServos));

        //Claw Servo Intake/Outtake
//            intakeTrigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
//                    .whenPressed(clawServos::clawClose);
//            outtakeTrigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
//                    .whenPressed(clawServos::clawOpen);
//            intakeTrigger = (new GamepadButton(operatorGamepad, GamepadKeys.Button.BACK))
//                    .whenPressed(clawServos::clawClose);
//            outtakeTrigger = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP))
//                    .whenPressed(clawServos::clawOpen);
//
//            openClawButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT))
//                    .whenPressed(clawServos::clawOpen);

        //PIDF Controllers Resets
//            clawMotorResetButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
//                    .whenPressed(clawMotors::encoderReset);
//            slideResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
//                    .whenPressed(slide::encoderReset);

        //Outaking the freight motion
//            dropButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER))
//                    .whenPressed(new TeleOpDropFreightCommand(drivetrain));
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
