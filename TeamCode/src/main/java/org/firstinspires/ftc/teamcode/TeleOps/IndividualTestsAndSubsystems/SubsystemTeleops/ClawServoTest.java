package org.firstinspires.ftc.teamcode.TeleOps.IndividualTestsAndSubsystems.SubsystemTeleops;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ClawServoTest")
public class ClawServoTest extends OpMode {
    //hardware initialization stuff
    Servo servo1;
    CRServo clawS2;
    double pos = 0.6;

    public static double CLOSE_POS_S1 = 0.9;
    public static double OPEN_POS_S1 = 0.75;

    public static double CLOSE_POS_S2 = 0.83;
    public static double OPEN_POS_S2 = 0.7;

    public static double INTAKE_POWER = -1;
    public static double OUTTAKE_POWER = 1;

    public static double FRONT_POS_S3 = 0.16;
    public static double BACK_POS_S3 = 0.83;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "clawS1");
        clawS2 = hardwareMap.get(CRServo.class, "clawS2");
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
//        if(gamepad1.a){
//            pos -= 0.001;
//        }
//        else if(gamepad1.b){
//            pos += 0.001;
//        }
//
//
//        if(gamepad1.right_bumper){
//            clawS2.set(1);
//            telemetry.addData("Se21: ", servo1.getPosition());
//
//        }
//        else if(gamepad1.left_bumper){
//            clawS2.set(-1);
//            telemetry.addData("Sev-1: ", servo1.getPosition());
//
//        }
//        else{
//            clawS2.stop();
//        }


        pos = Math.min(Math.max(pos, 0), 1);
        servo1.setPosition(Math.min(Math.max(pos, 0), 1));
        telemetry.addData("Servo pos: ",servo1.getPosition());
        telemetry.addData("Desired pos: ", pos);
        telemetry.update();

    }
}