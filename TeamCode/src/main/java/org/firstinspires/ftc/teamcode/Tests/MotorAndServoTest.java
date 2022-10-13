package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOps.GamepadTrigger;
import org.firstinspires.ftc.teamcode.driveTrain.MatchOpMode;

@Config
@TeleOp(name = "Motor/ServoTest")
public class MotorAndServoTest extends OpMode {
    // Gamepads
    private GamepadEx driverGamepad;
    // Motors and Servos
    private MotorEx motor;

    @Override
    public void init() {
        driverGamepad = new GamepadEx(gamepad1);
        motor = hardwareMap.get(MotorEx.class, "leftRear");
    }


    @Override
    public void loop() {
        if(gamepad1.right_bumper){
            motor.set(0.5);
        }
        else if(gamepad1.left_bumper){
            motor.set(-0.5);
        }
        telemetry.addData("\n\nMotor Speed: ",motor.get());

        telemetry.update();
    }

//    @Override
//    public void matchLoop() { }
//    @Override
//    public void disabledPeriodic() { }
//    @Override
//    public void matchStart() { }
//    @Override
//    public void robotPeriodic() { }
}
