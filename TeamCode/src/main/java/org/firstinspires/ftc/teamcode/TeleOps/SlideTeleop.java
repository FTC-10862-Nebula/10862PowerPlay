package org.firstinspires.ftc.teamcode.TeleOps;

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
    private static double startPoseX = 0;
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
            slideUpButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(slide::upSlideManual).whenReleased(slide::stopSlide));
            slideDownButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(slide::downSlideManual).whenReleased(slide::stopSlide));

        //Slide positions
            groundSButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(slide::slideGround));
            lowSButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(slide::slideGround));
            midSButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(slide::slideGround));
            highSButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(slide::slideGround));

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
