package org.firstinspires.ftc.teamcode.TeleOps.IndividualTestsAndSubsystems.SubsystemTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrain.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Config
@TeleOp(name = "Slide Teleop")
public class SlideTeleop extends MatchOpMode {
    private static final double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx liftMotor1, liftMotor2;

    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;

    // Subsystems
    private Slide slide;


    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
    }

    //Buttons
    public Button slideUpButton, slideDownButton;   //Manual
    public Button groundSButton, lowSButton, midSButton, highSButton;   //Positions
    public Button resetEveryThingButton, slideEncoderResetButton; //Reset Buttons

    @Override
    public void configureButtons() {
        //Slide Manual
//            slideUpButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
//                    .whenPressed(slide::upSlideManual)
//                    .whenReleased(slide::stopSlide));
//            slideDownButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
//                    .whenPressed(slide::downSlideManual)
//                    .whenReleased(slide::stopSlide));

            Button cone5 = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(slide::slideCone5));
            Button cone4 = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                    .whenPressed(slide::slideCone4));
            Button cone3 = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
                    .whenPressed(slide::slideCone3));
            Button cone2 = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(slide::slideCone2));
            Button cone1 = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(slide::slideCone1));


//        //Slide positions
//            groundSButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
//                    .whenPressed(slide::slideGround));
//            lowSButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
//                    .whenPressed(slide::slideLow));
//            midSButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
//                    .whenPressed(slide::slideMid));
//            highSButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
//                    .whenPressed(slide::slideHigh));

        //reset everything
            resetEveryThingButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN))
                    .whenPressed(slide::slideResting);

        //Encoder Reset
            slideEncoderResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
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
