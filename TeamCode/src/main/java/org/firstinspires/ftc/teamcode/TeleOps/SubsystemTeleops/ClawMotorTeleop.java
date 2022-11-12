package org.firstinspires.ftc.teamcode.TeleOps.SubsystemTeleops;

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

    //Motors and Servos
    private MotorEx clawMotor;
    // Gamepad
    private GamepadEx operatorGamepad;
    // Subsystems
    private Arm arm;

    @Override
    public void robotInit() {
        operatorGamepad = new GamepadEx(gamepad2);

        arm = new Arm(clawMotor, telemetry, hardwareMap);
    }

    //Buttons
    public Button intakeF, moveF, highF;
    public Button intakeB, moveB, highB;
    public Button clawMotorResetButton;

    @Override
    public void configureButtons() {
//        one = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
//                .whenPressed(clawMotors::setPower));

        intakeF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(arm::moveIntakeF));
        moveF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenPressed(arm::moveF));
        highF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenPressed(arm::moveHighF));

        intakeB = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(arm::moveIntakeB));
        moveB = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(arm::moveB));
        highB = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(arm::moveHighB));

        clawMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                .whenPressed(arm::encoderReset);

        Button armRaiseButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(arm::raiseClawManual)
                .whenReleased(arm::stopClaw));
        Button armLowerButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(arm::lowerClawManual)
                .whenReleased(arm::stopClaw));
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

