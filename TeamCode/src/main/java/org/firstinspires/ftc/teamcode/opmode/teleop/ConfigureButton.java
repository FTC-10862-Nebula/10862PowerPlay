package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.GamepadTrigger;
import org.firstinspires.ftc.teamcode.commands.driveCommands.teleopCommands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.intakeAndOutake.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.intakeAndOutake.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideBCommands.SlideIntakeBCommandT;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideGroundFCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideLowFCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class ConfigureButton extends SequentialCommandGroup{
    public ConfigureButton(GamepadEx driverGamepad, GamepadEx operatorGamepad, Drivetrain drivetrain, Arm arm, Slide slide, TurnServo turnServo, Claw claw){

        //Drive Stuff - D1
        Button reinializeIMUButton= (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
                .whenPressed( new InstantCommand(drivetrain::reInitializeIMU));

//        Button fieldDriveButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.BACK))
//                .whenPressed(new DefaultDriveCommand(drivetrain, driverGamepad, true));

        //Slowmode - D1
        Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));

        Button intakeD2Trigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                .whenPressed(new PickConeCommand(claw, slide, arm));
        Button outtakeD2Trigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                .whenPressed(new DropConeCommand(claw, slide, arm));

        //Slide positions - D2
        Button groundBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenPressed(new SlideGroundFCommand(slide, arm, claw, turnServo, false)));
        Button lowBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenPressed(new SlideLowFCommand(slide, arm, claw, turnServo, false)));
        Button midBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whenPressed(new SlideMidFCommand(slide, arm, claw, turnServo, false)));
        Button highBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(new SlideHighFCommand(slide, arm, claw, turnServo, false)));

        Button resetBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SlideIntakeBCommandT(slide, arm, claw, turnServo)));
        Button resetArmUp = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(arm::moveReset));

        //Slide Manual - D2
        Button slideUpButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT))
                .whileHeld(slide::upSlideManual)
                .whenReleased(slide::stopSlide);
        Button slideDownButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT))
                .whileHeld(slide::downSlideManual)
                .whenReleased(slide::stopSlide);

        //Arm Manual - D2
        Button armRaiseButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER))
                .whileHeld(arm::raiseClawManual)
                .whenReleased(arm::stopArm);
        Button armLowerButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(arm::lowerClawManual)
                .whenReleased(arm::stopArm);

        //Claw Servo 3 Buttons - D1
            /*Button s3FButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y))
                    .whenPressed(claw::setFClawPos);
            Button s3BButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                    .whenPressed(claw::setBClawPos);*/



//            PIDF Controllers Resets
        Button armMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                .whenPressed(arm::encoderReset);
        Button slideEncoderResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.BACK))
                .whenPressed(slide::encoderReset);

            /*
            Button groundFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(new SlideGroundFCommand(slide, clawMotors, claw)));
            Button lowFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new SlideLowFCommand(slide, clawMotors, claw)));
            Button midFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new SlideMidFCommand(slide, clawMotors, claw)));
            Button highFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(new SlideHighFCommand(slide, clawMotors, claw)));


            //Claw Servo Manual Rotation - D1
            Button plusClaw3Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP))
                    .whenPressed(claw::addClaw3Pos);
            Button subClaw3Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN))
                    .whenPressed(claw::subClaw3Pos);
            //Claw Servo Manual In/Out - D1
            Button plusClaw1Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT))
                    .whenPressed(claw::addClaw1Pos);
            Button subClaw1Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT))
                    .whenPressed(claw::subClaw1Pos);
            */
    }
}