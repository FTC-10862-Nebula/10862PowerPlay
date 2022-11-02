package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

@Config
public class ClawMotors extends SubsystemBase {

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.002, 0.2, 0, 0.0);
    //I = 0.0008
    private PIDFController controller;
    private boolean automatic;

    public static double CPR = 384.5;
    public static double UP_SPEED = -0.55;
    public static double DOWN_SPEED = 0.55;

    private double encoderOffset = 0;
    public static int INIT_POS = 0;

//    public static int INTAKE_POS_BACK = -273;
//    public static int GROUND_POS_BACK = -267;
//    public static int LOW_POS_BACK = -250;
//    public static int MID_POS_BACK = -240;
//    public static int HIGH_POS_BACK = -180;

    public static int INTAKE_POS_FRONT = 273;
    public static int GROUND_POS_FRONT = 267;
    public static int LOW_POS_FRONT = 250;
    public static int MID_POS_FRONT = 240;
    public static int HIGH_POS_FRONT = 180;

    private static double POWER = 0.8;
    private static boolean FLIP = false;

    private int clawPos = 0;

    Telemetry telemetry;
    private MotorEx clawMotor;

    public ClawMotors(MotorEx clawMotor, Telemetry tl, HardwareMap hw) {
        this.clawMotor = clawMotor;
        this.clawMotor = new MotorEx(hw, "clawM");

        //Reverse claw motor
        this.clawMotor.setInverted(true);
        this.clawMotor.resetEncoder();

        this.clawMotor.setDistancePerPulse(360 / CPR);

        controller = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f, getAngle(), getAngle());
        controller.setTolerance(10);

        this.telemetry = tl;
        automatic = false;
        setOffset();
        FLIP = false;
    }

    @Override
    public void periodic() {
        if (automatic) {
            controller.setF(pidfCoefficients.f * Math.cos(Math.toRadians(controller.getSetPoint())));

            double output = controller.calculate(getAngle());
            if (output >= 1) output = 1;
            if (output <= -1) output = -1;


            clawMotor.set(output * POWER);
        }

        Util.logger(this, telemetry, Level.INFO, "Claw Encoder Pos: ", clawMotor.getCurrentPosition());
        Util.logger(this, telemetry, Level.INFO, "Claw Pos: ", clawPos);
    }


    private double getEncoderDistance() {
        return clawMotor.getDistance() - encoderOffset;
    }
    public void setAutomatic(boolean auto) {
        this.automatic = auto;
    }
    public double getAngle() {
        return getEncoderDistance();
    }

    /****************************************************************************************/

    public void raiseClawManual() {
        automatic = false;
        clawMotor.set(UP_SPEED);
    }
    public void lowerClawManual() {
        automatic = false;
        clawMotor.set(DOWN_SPEED);
    }

    public void stopClaw() {
        clawMotor.stopMotor();
        controller.setSetPoint(getAngle());
        automatic = false;
    }

    /****************************************************************************************/

    public void moveIntakeF(boolean FLIP) {
        automatic = true;
        if(FLIP){
            controller.setSetPoint(-INTAKE_POS_FRONT);
            clawPos = 5;
        } else{
            controller.setSetPoint(INTAKE_POS_FRONT);
            clawPos = 0;
        }


    }
    public void moveGroundF(boolean FLIP) {
        automatic = true;
        if(FLIP){
            controller.setSetPoint(-GROUND_POS_FRONT);
            clawPos = 6;
        } else {
            controller.setSetPoint(GROUND_POS_FRONT);
            clawPos = 1;
        }
    }
    public void moveLowF(boolean FLIP) {
        automatic = true;
        if(FLIP){
            telemetry.addLine("NEG LOWPOSF");
            controller.setSetPoint(-LOW_POS_FRONT);
            clawPos = 7;
        } else {
            telemetry.addLine("POS LOWPOSF");
            controller.setSetPoint(LOW_POS_FRONT);
            clawPos = 2;
        }
    }
    public void moveMidF(boolean FLIP) {
        automatic = true;
        telemetry.addLine("in moveMid");

        if(FLIP){
            telemetry.addLine("NEG MIDPOSF");

            controller.setSetPoint(-MID_POS_FRONT);
            clawPos = 8;
        } else{
            telemetry.addLine("POS MIDPOSF");

            controller.setSetPoint(MID_POS_FRONT);
        clawPos = 3;
        }
    }
    public void moveHighF(boolean FLIP) {
        automatic = true;
        if(FLIP){
            controller.setSetPoint(-HIGH_POS_FRONT);
            clawPos = 9;
        } else {
            controller.setSetPoint(HIGH_POS_FRONT);
            clawPos = 4;
        }
    }

    public void encoderReset() {
        clawMotor.resetEncoder();
    }

    /****************************************************************************************/

    public boolean getFlip(){
        return FLIP;
    }

    public void setFlipTrue(){
        FLIP = true;
        telemetry.addLine("FLIP = true");
        //True means Back
    }

    public void setFlipFalse(){
        FLIP = false;
        telemetry.addLine("FLIP = false");
        //False means Front
    }

//    public void setLift(double angle) {
//        automatic = true;
//        controller.setSetPoint(angle);
//    }

    public boolean atTargetAngle() {
        return controller.atSetPoint();
    }

    public void setOffset() {
        resetEncoder();
        controller.setSetPoint(getAngle());
    }
    public void clawEncoderReset() {
        clawPos = 0;
    }
    public void resetEncoder() {
        clawEncoderReset();
    }

    public void moveClawToCorrectHeight() {
        if(clawPos == 0) {
            moveIntakeF(false);
        } else if(clawPos == 1) {
            moveGroundF(false);
        } else if(clawPos == 2) {
            moveLowF(false);
        } else if(clawPos == 3) {
            moveMidF(false);
        } else if(clawPos == 4) {
            moveHighF(false);

        } else if(clawPos == 5) {
            moveIntakeF(true);
        } else if(clawPos == 6) {
            moveGroundF(true);
        } else if(clawPos == 7) {
            moveLowF(true);
        } else if(clawPos == 8) {
            moveMidF(true);
        } else if(clawPos == 9) {
            moveHighF(true);
        }
    }
}
