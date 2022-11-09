package org.firstinspires.ftc.teamcode.TeleOps.IndividualTestsAndSubsystems.SubsystemTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

//@Disabled
@Config
@TeleOp(name = "ClawMotorTeleop")
public class ClawMotorTeleop extends MatchOpMode {
    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx clawMotor;
    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;
    // Subsystems
    private Arm arm;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        arm = new Arm(clawMotor, telemetry, hardwareMap);
    }

    //Buttons
    public Button intakeF, groundF, lowF, midF, highF;
    public Button intakeB, groundB, lowB, midB, highB;
    public Button clawMotorResetButton;
    public Button one, two;

    @Override
    public void configureButtons() {
//        one = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
//                .whenPressed(clawMotors::setPower));

        intakeF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(arm::moveIntakeF));
        highF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(arm::moveHighF));

        intakeB = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(arm::moveIntakeB));
        highB = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(arm::moveHighB));

        clawMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                .whenPressed(arm::encoderReset);
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

