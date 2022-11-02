package org.firstinspires.ftc.teamcode.TeleOps.IndividualTestsAndSubsystems.SubsystemTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOps.GamepadTrigger;
import org.firstinspires.ftc.teamcode.driveTrain.MatchOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;

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
    private ClawMotors clawMotors;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        clawMotors = new ClawMotors(clawMotor, telemetry, hardwareMap);
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
                .whenPressed(new InstantCommand(()->clawMotors.moveIntakeF(clawMotors.getFlip()))));
        groundF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(()->clawMotors.moveGroundF(clawMotors.getFlip()))));
        lowF= (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(()->clawMotors.moveLowF(clawMotors.getFlip()))));
        midF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()->clawMotors.moveMidF(clawMotors.getFlip()))));
        highF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(()->clawMotors.moveHighF(clawMotors.getFlip()))));

        clawMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                .whenPressed(clawMotors::encoderReset);
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

