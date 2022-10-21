package org.firstinspires.ftc.teamcode.TeleOps.IndividualTestsAndSubsystems.SubsystemTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOps.GamepadTrigger;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.driveTrain.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
@Disabled
@Config
@TeleOp(name = "ClawOnlyTeleop")
public class ClawOnlyTeleop extends MatchOpMode {
//
//    //Motors and Servos
//    private MotorEx clawMotor;
//    private ServoEx clawS1, clawS2, clawS3;
//
//    // Gamepad
//    private GamepadEx driverGamepad, operatorGamepad;
//
//
//    // Subsystems
//    private ClawMotors clawMotors;
//    private ClawServos clawServos;


    @Override
    public void robotInit() {
//        driverGamepad = new GamepadEx(gamepad1);
//        operatorGamepad = new GamepadEx(gamepad2);
//
//        clawMotors = new ClawMotors(clawMotor, telemetry, hardwareMap);
//        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
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
//        one = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
//                .whenPressed( new DropConeCommand(clawServos)));
//        two = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
//                .whenPressed( new PickConeCommand(clawServos)));
//
//        three = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
//                .whenPressed(clawServos::addClaw1Pos));



//        one = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
//                    .whenPressed(clawMotors::moveClawGroundFront));
//        two = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
//                    .whenPressed(clawMotors::moveClawLowFront));
//        three = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
//                   .whenPressed(clawMotors::moveClawMidFront));
//        four = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
//                .whenPressed(clawMotors::moveClawHighFront));

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
