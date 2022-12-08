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
public class Arm extends SubsystemBase {

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.0025, 0.2, 0, 0.0);
    //I = 0.0008
    private PIDFController controller;
    private boolean armAutomatic;

    public static double CPR = 384.5;
    public static double UP_SPEED = -0.55;
    public static double DOWN_SPEED = 0.55;

    private double encoderOffset = 0;
    private double offsetNum = 0;
    public static int INIT_POS = 0;
//    private static double newSetPosition;

    public static int INTAKE_POS_BACK = -312,
                        POS_BACK = -252,
                        HIGH_POS_BACK = -145;
    public static int HIGH_POS_AUTO_BACK = -140;
    public static int INTAKE_POS_AUTO_BACK = -237;
    public static int POS_AUTO_BACK = -160;

    public static int INTAKE_POS_FRONT = -INTAKE_POS_BACK,
            POS_FRONT = -POS_BACK,
            HIGH_POS_FRONT = -HIGH_POS_BACK;
    public static int HIGH_POS_AUTO_FRONT = -HIGH_POS_AUTO_BACK,
            INTAKE_POS_AUTO_FRONT = -INTAKE_POS_AUTO_BACK,
            POS_AUTO_FRONT = -POS_AUTO_BACK;



    private static double POWER = 0.93;
    private static int clawPos = 0;

    Telemetry telemetry;
    private MotorEx armMotor;

    public Arm(Telemetry tl, HardwareMap hw) {
//        this.armMotor = armMotor;
        this.armMotor = new MotorEx(hw, "clawM");

        //Reverse claw motor
         this.armMotor.setInverted(true);
        this.armMotor.resetEncoder();
//        this.armMotor.setZeroPowerBehavior(BRAKE);
        this.armMotor.setDistancePerPulse(360 / CPR);

        this.armMotor.set(0);

        controller = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f, getAngle(), getAngle());
        controller.setTolerance(10);

        this.telemetry = tl;
        armAutomatic = false;
        setOffset();
    }

    @Override
    public void periodic() {
        if (armAutomatic) {

            controller.setF(pidfCoefficients.f * Math.cos(Math.toRadians(controller.getSetPoint())));

            double output = controller.calculate(getAngle());
            telemetry.addData("CLaw Motor Output:", output);

//            if (output >= 1) output = 1;
//            if (output <= -1) output = -1;
//            armMotor.set(output);

            armMotor.set(output * POWER);

        }
        Util.logger(this, telemetry, Level.INFO, "Claw Encoder Pos: ", armMotor.getCurrentPosition());
        Util.logger(this, telemetry, Level.INFO, "Claw Pos: ", clawPos);
    }


    private double getEncoderDistance() {
        return armMotor.getDistance() - encoderOffset;
    }
    public void setAutomatic(boolean auto) {
        this.armAutomatic = auto;
    }
    public double getAngle() {
        return getEncoderDistance();
    }

    /****************************************************************************************/

    public void raiseClawManual() {
        armAutomatic = false;
        armMotor.set(UP_SPEED);
    }
    public void lowerClawManual() {
        armAutomatic = false;
        armMotor.set(DOWN_SPEED);
    }

    public void stopClaw() {
        armMotor.stopMotor();
//        controller.setSetPoint(getAngle());
        armAutomatic = false;
    }

    /****************************************************************************************/

    public void moveReset(){
        armAutomatic = true;
        controller.setSetPoint(INIT_POS);
        clawPos = 0;
    }
    public void moveIntakeF() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_FRONT);
        clawPos = 1;
    }
    public void moveF() {
        armAutomatic = true;
        controller.setSetPoint(POS_FRONT);
        clawPos = 2;
    }
    public void moveHighF() {
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_FRONT);
        clawPos = 3;
    }

    public void moveIntakeB() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_BACK);
        clawPos = 4;
    }
    public void moveB() {
        armAutomatic = true;
        controller.setSetPoint(POS_BACK);
        clawPos = 5;
    }
    public void moveHighB() {
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_BACK);
        clawPos = 6;
    }
    public void moveHighBAuto(){
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_AUTO_BACK);
        clawPos = 7;
    }
    public void moveHighFAuto(){
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_AUTO_FRONT);
        clawPos = 8;
    }
    public void moveIntakeFAuto() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_AUTO_FRONT);
        clawPos = 9;
    }
    public void moveBAuto() {
        armAutomatic = true;
        controller.setSetPoint(POS_AUTO_BACK);
        clawPos = 10;
    }
    public void moveFAuto() {
        armAutomatic = true;
        controller.setSetPoint(POS_AUTO_FRONT);
        clawPos = 11;
    }
    public void moveIntakeBAuto() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_AUTO_BACK);
        clawPos = 12;
    }

    public void dropArmTeleop(){
        switch (clawPos){
            case 3:
                controller.setSetPoint(HIGH_POS_FRONT+75);
                return;
            case 6:
                controller.setSetPoint(HIGH_POS_BACK-75);
                return;
        }
    }

    public void dropArmAuto(){
        switch (clawPos){
//            case 2:
//                controller.setSetPoint(POS_FRONT+40);
//                return;
//            case 3:
//                controller.setSetPoint(HIGH_POS_FRONT+75);
//                return;
//            case 5:
//                controller.setSetPoint(POS_BACK-40);
//                return;
//            case 6:
//                controller.setSetPoint(HIGH_POS_BACK-75);
//                return;
            case 7:
                controller.setSetPoint(HIGH_POS_AUTO_BACK-45);
                return;
            case 8:
                controller.setSetPoint(HIGH_POS_AUTO_FRONT+45);
                return;
            case 9:
                controller.setSetPoint(INTAKE_POS_AUTO_FRONT+25);
                return;
            case 10:
                controller.setSetPoint(POS_AUTO_BACK-35);
                return;
            case 11:
                controller.setSetPoint(POS_AUTO_FRONT+35);
                return;
            case 12:
                controller.setSetPoint(INTAKE_POS_AUTO_BACK-25);
                return;
        }
    }

    public void encoderReset() {
        armMotor.resetEncoder();
        telemetry.addLine("ARM RESET");
    }

    /****************************************************************************************/


//    public void setLift(double angle) {
//        automatic = true;
//        controller.setSetPoint(angle);
//    }

    public boolean atTargetAngle() {
        return controller.atSetPoint();
    }

//    public void setPower(){
//        clawMotor.set(5);
//    }

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

//    public void moveClawToCorrectHeight() {
//        if(clawPos == 0) {
//            moveIntakeF();
//        } else if(clawPos == 1) {
//            moveHighF();
//        } else if(clawPos == 2) {
//            moveIntakeB();
//        } else if(clawPos == 3) {
//            moveHighB();
//        }
//    }
}
