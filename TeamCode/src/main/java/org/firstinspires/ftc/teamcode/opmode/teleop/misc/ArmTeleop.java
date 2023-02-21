package org.firstinspires.ftc.teamcode.opmode.teleop.misc;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
//@Disabled
@Config
@TeleOp
public class ArmTeleop extends MatchOpMode {
    // Gamepad
    private GamepadEx operatorGamepad;
    // Subsystems
    private Arm arm;

    @Override
    public void robotInit() {
        operatorGamepad = new GamepadEx(gamepad2);
        arm = new Arm(telemetry, hardwareMap);
    }

    @Override
    public void configureButtons() {
            Button intakeButonAuto = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                    .whenPressed(new InstantCommand(arm::moveIntakeBAuto)));
            Button backAuto = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(arm::moveBAuto)));
            Button highAuto = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new InstantCommand(arm::moveHighBAuto)));


            Button groundBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(new InstantCommand(arm::moveHighB)));
            Button lowBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new InstantCommand(arm::moveB)));
            Button midBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new InstantCommand(arm::moveGroundB)));
            Button highBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(new InstantCommand(arm::moveIntakeB)));

            Button resetArmUp = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(arm::moveReset));

//            PIDF Controllers Resets
            Button armMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                    .whenPressed(arm::encoderReset);





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
