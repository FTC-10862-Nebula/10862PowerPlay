package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

@Config
public class Arm extends SubsystemBase {

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.0025, 0.2, 0, 0.0);
    //I = 0.0008
    private final PIDFController controller;
    private boolean armAutomatic;
    private final AnalogInput potentiometer;

    public static double CPR = 384.5;
//    public static double UP_SPEED = -0.55;
//    public static double DOWN_SPEED = 0.55;

    private final double encoderOffset = 0;
//    private double offsetNum = 0;
    public static int INIT_POS = 0;

    public static int INTAKE_POS_BACK = -318,
                        POS_BACK = -252,
                        HIGH_POS_BACK = -158,
                        GROUND_POS_BACK = -240;
    public static int HIGH_POS_AUTO_BACK = -129 ,
                        INTAKE_POS_AUTO_BACK = -270,
                        POS_AUTO_BACK = -165;

    public static int INTAKE_POS_FRONT = -INTAKE_POS_BACK,
                        POS_FRONT = -POS_BACK,
                        HIGH_POS_FRONT = -HIGH_POS_BACK,
                        GROUND_POS_FRONT = -GROUND_POS_BACK;
    public static int HIGH_POS_AUTO_FRONT = -HIGH_POS_AUTO_BACK,
                        INTAKE_POS_AUTO_FRONT = -INTAKE_POS_AUTO_BACK,
                        POS_AUTO_FRONT = -POS_AUTO_BACK;
    public enum ArmPos{
        RESET,
        INTAKE_BACK, BACK, HIGH_BACK, GROUND_BACK,
        INTAKE_FRONT, FRONT, HIGH_FRONT, GROUND_FRONT,
        AUTO_INTAKE_BACK, AUTO_BACK, AUTO_HIGH_BACk,
        AUTO_INTAKE_FRONT, AUTO_FRONT, AUTO_HIGH_FRONT,
    }
    ArmPos armPos = ArmPos.RESET;



    private final static double POWER = 0.93;
//    private static int armPos = 0;

    Telemetry telemetry;
    private final MotorEx armMotor;

    public Arm(Telemetry tl, HardwareMap hw) {
        armMotor = new MotorEx(hw, "clawM");

        //Reverse claw motor
        armMotor.setInverted(true);
        armMotor.resetEncoder();
//        this.armMotor.setZeroPowerBehavior(BRAKE);
        armMotor.setDistancePerPulse(360 / CPR);

        armMotor.set(0);

        controller = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f, getAngle(), getAngle());
        controller.setTolerance(10);

        this.telemetry = tl;
        armAutomatic = false;
        setOffset();
//        armPos = ArmPos.RESET;
        potentiometer = hw.get(AnalogInput.class, "Potentiometer");
//        potentiometer = new AnalogInput(null, 0);
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
        Util.logger(this, telemetry, Level.INFO, "Claw Pos: ", armPos);
        telemetry.addData("Current Voltage", potentiometer.getVoltage());

    }


    private double getEncoderDistance() {
        return armMotor.getDistance() - encoderOffset;
    }
    public double getAngle() {
        return getEncoderDistance();
    }

    /****************************************************************************************/

    public void raiseClawManual() {
//        armAutomatic = false;
//        armMotor.set(UP_SPEED);
        armAutomatic = true;
        if((armMotor.getCurrentPosition()<275)){
            controller.setSetPoint(armMotor.getCurrentPosition()+5);
        }
//        else return;
    }
    public void lowerClawManual() {
//        armAutomatic = false;
//        armMotor.set(DOWN_SPEED);
        armAutomatic = true;
        if((armMotor.getCurrentPosition()>-275)){
            controller.setSetPoint(armMotor.getCurrentPosition()-5);
        }
//        else return;
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
        armPos = ArmPos.RESET;
    }
    public void moveIntakeF() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_FRONT);
        armPos = ArmPos.INTAKE_FRONT;
    }
    public void moveIntakeB() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_BACK);
        armPos = ArmPos.INTAKE_BACK;
    }
    public void moveGroundB(){
        armAutomatic = true;
        controller.setSetPoint(GROUND_POS_BACK);
        armPos = ArmPos.GROUND_BACK;
    }
    public void moveGroundF(){
        armAutomatic = true;
        controller.setSetPoint(GROUND_POS_FRONT);
        armPos = ArmPos.GROUND_FRONT;
    }
    public void moveF() {
        armAutomatic = true;
        controller.setSetPoint(POS_FRONT);
        armPos = ArmPos.FRONT;
    }
    public void moveB() {
        armAutomatic = true;
        controller.setSetPoint(POS_BACK);
        armPos = ArmPos.BACK;
    }
    public void moveHighF() {
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_FRONT);
        armPos = ArmPos.HIGH_FRONT;
    }
    public void moveHighB() {
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_BACK);
        armPos = ArmPos.HIGH_BACK;
    }



    public void moveHighBAuto(){
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_AUTO_BACK);
        armPos = ArmPos.AUTO_HIGH_BACk;
    }
    public void moveHighFAuto(){
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_AUTO_FRONT);
        armPos = ArmPos.AUTO_HIGH_FRONT;
    }
    public void moveIntakeFAuto() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_AUTO_FRONT);
        armPos = ArmPos.AUTO_INTAKE_FRONT;
    }
    public void moveIntakeBAuto() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_AUTO_BACK);
        armPos = ArmPos.AUTO_INTAKE_BACK;
    }
    public void moveBAuto() {
        armAutomatic = true;
        controller.setSetPoint(POS_AUTO_BACK);
        armPos = ArmPos.AUTO_BACK;
    }
    public void moveFAuto() {
        armAutomatic = true;
        controller.setSetPoint(POS_AUTO_FRONT);
        armPos = ArmPos.AUTO_FRONT;
    }



    public void dropArmTeleop(){
        switch (armPos){
//            case 2:
//                controller.setSetPoint(POS_FRONT+40);
//                return;
            case HIGH_FRONT:
                controller.setSetPoint(HIGH_POS_FRONT+75);
                return;
//            case 5:
//                controller.setSetPoint(POS_BACK-40);
//                return;
            case HIGH_BACK:
                controller.setSetPoint(HIGH_POS_BACK-75);
                return;
        }
    }

    public void dropArmAuto(){
        switch (armPos){
            case AUTO_HIGH_BACk:
                controller.setSetPoint(HIGH_POS_AUTO_BACK-59);
                return;
            case AUTO_HIGH_FRONT:
                controller.setSetPoint(HIGH_POS_AUTO_FRONT+50);
                return;
            case AUTO_INTAKE_FRONT:
                controller.setSetPoint(INTAKE_POS_AUTO_FRONT+25);
                return;
            case AUTO_BACK:
                controller.setSetPoint(POS_AUTO_BACK-35);
                return;
            case AUTO_FRONT:
                controller.setSetPoint(POS_AUTO_FRONT+35);
                return;
            case AUTO_INTAKE_BACK:
                controller.setSetPoint(INTAKE_POS_AUTO_BACK-25);
                return;
        }
    }

    public void encoderReset() {
        armMotor.resetEncoder();
        telemetry.addLine("ARM RESET");
    }

    /****************************************************************************************/


    public void setOffset() {
        resetEncoder();
        controller.setSetPoint(getAngle());
    }
    public void clawEncoderReset() {
        armPos = ArmPos.RESET;
    }
    public void resetEncoder() {
        clawEncoderReset();
    }

    //Check Documentation if confused
    public double getPotentiometerAngle(){
        double angle = potentiometer.getVoltage()*81.8;
        return Range.scale(potentiometer.getVoltage(), 0, potentiometer.getMaxVoltage(), 0, 270);
    }
}
