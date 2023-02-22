package org.firstinspires.ftc.teamcode.opmode.teleop.misc;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;

//@Disabled
@Config
@TeleOp(group = "Subsystem test")
public class TeleOpArmOnly extends MatchOpMode {
    // Gamepad
    private GamepadEx operatorGamepad;
    // Subsystems
    private Pivot pivot;

    @Override
    public void robotInit() {
        operatorGamepad = new GamepadEx(gamepad2);
        pivot = new Pivot(telemetry, hardwareMap);
    }

    @Override
    public void configureButtons() {
            Button intakeButonAuto = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                    .whenPressed(new InstantCommand(pivot::moveIntakeBAuto)));
            Button backAuto = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(pivot::moveBAuto)));
            Button highAuto = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new InstantCommand(pivot::moveHighBAuto)));


            Button groundBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(new InstantCommand(pivot::moveHighB)));
            Button lowBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new InstantCommand(pivot::moveB)));
            Button midBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new InstantCommand(pivot::moveGroundB)));
            Button highBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(new InstantCommand(pivot::moveIntakeB)));

            Button resetArmUp = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(pivot::moveInitializationPosition));

//            PIDF Controllers Resets
            Button armMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                    .whenPressed(pivot::encoderReset);





//            new GamepadButton(operatorGamepad, GamepadKeys.Button.START)
//                    .whenPressed(arm::setArm)
//                    .whenPressed(()->(arm.setArm(2));
    }

    @Override
    public void matchLoop() {
    }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){
    }
}
