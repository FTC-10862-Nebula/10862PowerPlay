package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotorSubsystem extends SubsystemBase {


    public static double UP_SPEED = -0.2;
    public static double DOWN_SPEED = 0.2;

    private int clawPos = 0;

    Telemetry telemetry;
    private MotorEx testMotor;

    public MotorSubsystem(MotorEx testMotor, Telemetry tl, HardwareMap hw) {
        this.testMotor = testMotor;
        this.testMotor = new MotorEx(hw, "clawM");

        //Reverse claw motor
        this.testMotor.setInverted(true);


        this.telemetry = tl;
    }

    @Override
    public void periodic() {
//        Util.logger(this, telemetry, Level.INFO, "Claw Encoder Pos: ", clawMotor.getCurrentPosition());
//        Util.logger(this, telemetry, Level.INFO, "Claw Pos: ", clawPos);
    }



    /****************************************************************************************/

    public void raiseClawManual() {
        testMotor.set(UP_SPEED);
    }
    public void lowerClawManual() {
        testMotor.set(DOWN_SPEED);
    }

    public void stopClaw() {
        testMotor.stopMotor();
    }

    /****************************************************************************************/

    public void encoderReset() {
        testMotor.resetEncoder();
    }

    /****************************************************************************************/


//    public void setLift(double angle) {
//        automatic = true;
//        controller.setSetPoint(angle);
//    }
}
