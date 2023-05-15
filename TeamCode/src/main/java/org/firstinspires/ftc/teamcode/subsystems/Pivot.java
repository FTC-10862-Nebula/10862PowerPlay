package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.logging.Level;

@Config
public class Pivot extends SubsystemBase {

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(.005, 0.00, 0.0,0);
    private final PIDFController controller;
    private boolean armAutomatic;
    public boolean shouldSensorWork = true;


    public static double CPR = 384.5;
    public static double UP_SPEED = -0.55;
    public static double DOWN_SPEED = 0.55;

    public int INIT_POS = 0;
    public int TELE_OP_START_POS = -350;

    public static int INTAKE_POS_BACK = -0,
                        POS_BACK = -455,
                        HIGH_POS_BACK = -458 ,
                        GROUND_POS_BACK = -480,
                        DROP_BACK = -700;
    public static int HIGH_POS_AUTO_BACK = -156,
                        INTAKE_POS_AUTO_BACK = -275,
                        POS_AUTO_BACK = -222,
                        DROP_AUTO_BACK = -390;

    public static int INTAKE_POS_FRONT = -INTAKE_POS_BACK,
                        POS_FRONT = -POS_BACK,
                        HIGH_POS_FRONT = -HIGH_POS_BACK,
                        GROUND_POS_FRONT = -GROUND_POS_BACK,
    DROP_FRONT = -DROP_BACK;
    public static int HIGH_POS_AUTO_FRONT = -HIGH_POS_AUTO_BACK,
                        INTAKE_POS_AUTO_FRONT = -INTAKE_POS_AUTO_BACK,
                        POS_AUTO_FRONT = -POS_AUTO_BACK,
                        DROP_AUTO_FRONT = -DROP_AUTO_BACK;
    public enum ArmPos {
        RESET,
        INTAKE_BACK, BACK, HIGH_BACK, GROUND_BACK, DROP_BACK,
        INTAKE_FRONT, FRONT, HIGH_FRONT, GROUND_FRONT, DROP_FRONT,
        AUTO_INTAKE_BACK, AUTO_BACK, AUTO_HIGH_BACK, AUTO_DROP_BACK,
        AUTO_INTAKE_FRONT, AUTO_FRONT, AUTO_HIGH_FRONT,AUTO_DROP_FRONT,
//        public double ArmPos(int rt){
//            int fsr =rt;
//        }
    }
    ArmPos armPos = ArmPos.RESET;



    private final static double POWER = 0.93;
    private double encoderOffset = 0;
//    private static int armPos = 0;

    Telemetry telemetry;
    private final MotorEx armMotor;

    public Pivot(Telemetry tl, HardwareMap hw) {
        armMotor = new MotorEx(hw, "clawM");

        //Reverse claw motor
        armMotor.setInverted(true);
        armMotor.resetEncoder();
//        this.armMotor.setZeroPowerBehavior(BRAKE); // TODO: FOR COMP
        armMotor.setDistancePerPulse(360 / CPR);

        armMotor.set(0);

        controller = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f, getAngle(), getAngle());
        controller.setTolerance(10);

        this.telemetry = tl;
        armAutomatic = false;
//        setOffset();
        //TODO: isAuto for Brake Mode
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE); //Remove for practice and stuff
//        armPos = ArmPos.RESET;

//        potentiometer = hw.get(AnalogInput.class, "Potentiometer");
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
        Util.logger(this, telemetry, Level.INFO, "Arm Encoder Pos: ", armMotor.getCurrentPosition());
        Util.logger(this, telemetry, Level.INFO, "Arm Pos: ", armPos);

    }


    private double getEncoderDistance() {
        return armMotor.getDistance() - encoderOffset;
    }
    public double getAngle() {
        return getEncoderDistance();
    }

    /****************************************************************************************/

    public void raiseClawManual() {
        armAutomatic = false;
        armMotor.set(UP_SPEED);

//        armAutomatic = true;
////        if((armMotor.getCurrentPosition()<275)){
//            controller.setSetPoint(armMotor.getCurrentPosition()+5);
////        }
////        else return;
    }
    public void lowerClawManual() {
        armAutomatic = false;
        armMotor.set(DOWN_SPEED);

//        armAutomatic = true;
//        if((armMotor.getCurrentPosition()>-275)){
//            controller.setSetPoint(armMotor.getCurrentPosition()-5);
//        }
//        else return;
    }

    public void stopArm() {
        armMotor.stopMotor();
        armAutomatic = false;
    }

    /****************************************************************************************/

    public void moveTeleOpAutoStart() {
        armAutomatic = true;
        setSetPoint(TELE_OP_START_POS);
        armPos = ArmPos.RESET;
        shouldSensorWork = true;
    }
    public void moveInitializationPosition() {
        armAutomatic = true;
        setSetPoint(INIT_POS - encoderOffset - 10);
        armPos = ArmPos.RESET;
        shouldSensorWork = true;
//        resetOffset();
    }
    public void moveIntakeF() {
        armAutomatic = true;
        setSetPoint(INTAKE_POS_FRONT);
        armPos = ArmPos.INTAKE_FRONT;
        shouldSensorWork = true;
    }
    public void moveIntakeB() {
        armAutomatic = true;
        setSetPoint(INTAKE_POS_BACK);
        armPos = ArmPos.INTAKE_BACK;
        shouldSensorWork = true;
    }
    public void moveGroundB(){
        armAutomatic = true;
        setSetPoint(GROUND_POS_BACK);
        armPos = ArmPos.GROUND_BACK;
        shouldSensorWork = false;
    }
    public void moveGroundF(){
        armAutomatic = true;
        setSetPoint(GROUND_POS_FRONT);
        armPos = ArmPos.GROUND_FRONT;
        shouldSensorWork = false;
    }
    public void moveF() {
        armAutomatic = true;
        setSetPoint(POS_FRONT);
        armPos = ArmPos.FRONT;
        shouldSensorWork = false;
    }
    public void moveB() {
        armAutomatic = true;
        setSetPoint(POS_BACK);
        armPos = ArmPos.BACK;
        shouldSensorWork = false;
    }
    public void moveHighF() {
        armAutomatic = true;
        setSetPoint(HIGH_POS_FRONT);
        armPos = ArmPos.HIGH_FRONT;
        shouldSensorWork = false;
    }
    public void moveHighB() {
        armAutomatic = true;
        setSetPoint(HIGH_POS_BACK);
        armPos = ArmPos.HIGH_BACK;
        shouldSensorWork = false;
    }



    public void moveHighBAuto(){
        armAutomatic = true;
        setSetPoint(HIGH_POS_AUTO_BACK);
        armPos = ArmPos.AUTO_HIGH_BACK;
        shouldSensorWork = false;
    }
    public void moveHighFAuto(){
        armAutomatic = true;
        setSetPoint(HIGH_POS_AUTO_FRONT);
        armPos = ArmPos.AUTO_HIGH_FRONT;
        shouldSensorWork = false;
    }
    public void moveIntakeFAuto() {
        armAutomatic = true;
        setSetPoint(INTAKE_POS_AUTO_FRONT);
        armPos = ArmPos.AUTO_INTAKE_FRONT;
        shouldSensorWork = true;
    }
    public void moveIntakeBAuto() {
        armAutomatic = true;
        setSetPoint(INTAKE_POS_AUTO_BACK);
        armPos = ArmPos.AUTO_INTAKE_BACK;
        shouldSensorWork = true;
    }
    public void moveBAuto() {
        armAutomatic = true;
        setSetPoint(POS_AUTO_BACK);
        armPos = ArmPos.AUTO_BACK;
        shouldSensorWork = false;
    }
    public void moveFAuto() {
        armAutomatic = true;
        setSetPoint(POS_AUTO_FRONT);
        armPos = ArmPos.AUTO_FRONT;
        shouldSensorWork = false;
    }
    public void moveBDrop() {
        armAutomatic = true;
        setSetPoint(DROP_BACK);
        armPos = ArmPos.DROP_BACK;
        shouldSensorWork = false;
    }
    public void moveFDrop() {
        armAutomatic = true;
        setSetPoint(DROP_FRONT);
        armPos = ArmPos.AUTO_DROP_FRONT;
        shouldSensorWork = false;
    }

    public void moveBDropAuto() {
        armAutomatic = true;
        setSetPoint(DROP_AUTO_BACK);
        armPos = ArmPos.DROP_BACK;
        shouldSensorWork = false;
    }
    public void moveFDropAuto() {
        armAutomatic = true;
        setSetPoint(DROP_AUTO_FRONT);
        armPos = ArmPos.AUTO_DROP_FRONT;
        shouldSensorWork = false;
    }

    public void dropArmTeleop(){
        switch (armPos){
            case FRONT:
            case HIGH_FRONT:
                setSetPoint(DROP_FRONT);
                break;
            case BACK:
            case HIGH_BACK:
                setSetPoint(DROP_BACK);
                break;
        }
    }

    public void dropArmAuto(){
        switch (armPos){
            case AUTO_HIGH_BACK:
            case AUTO_BACK:
                setSetPoint(DROP_AUTO_BACK);
                break;
            case AUTO_HIGH_FRONT:
            case AUTO_FRONT:
                setSetPoint(DROP_AUTO_FRONT);
                break;
            case AUTO_INTAKE_FRONT:
                setSetPoint(INTAKE_POS_AUTO_FRONT+25);
                shouldSensorWork = true;
                break;
            case AUTO_INTAKE_BACK:
                setSetPoint(INTAKE_POS_AUTO_BACK-25);
                shouldSensorWork = true;
                break;
        }
    }

    public void setSetPoint(double setPoint) {
        controller.setSetPoint(setPoint + encoderOffset);
    }

    public void encoderReset() {
        armMotor.resetEncoder();
        telemetry.addLine("ARM RESET");
    }

    /****************************************************************************************/

//
//    public void resetOffset() {
//        encoderOffset = 0;
//
//        armAutomatic = true;
//    }
//    public void clawEncoderReset() {
//        armPos = ArmPos.RESET;
//    }
//    public void resetEncoder() {
//        clawEncoderReset();
//    }

    //Check Documentation if confused
//    public double getPotentiometerAngle(){
//        double angle = potentiometer.getVoltage()*81.8;
//        return Range.scale(potentiometer.getVoltage(), 0, potentiometer.getMaxVoltage(), 0, 270);
//    }

    public void setPosition(double point){
        controller.setSetPoint(point);
    }
    public double getPosition(){
        return controller.getSetPoint();
    }

    public void pivotDown(){
        setPosition(getPosition()-10);
    }
    public void pivotUp(){
        setPosition(getPosition()+10);
    }
}

