package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slide extends SubsystemBase {
    private final Telemetry telemetry;
    private final MotorEx slideM1;
    private final MotorEx slideM2;

//    public boolean liftTime;
//    int liftError = 0, liftTargetPos = 0, setPos;

    public static PIDFCoefficients pidfUpCoefficients = new PIDFCoefficients(0.005, 0.00, 0,0);//.0075, 0., .003, 0)
//    public static PIDFCoefficients pidfDownCoefficients = new PIDFCoefficients(0.01, 0.00, 0, 0);

    private PIDFController upController;//, downController;
    private boolean slideAutomatic;

    public static double CPR = 751.8;
    public static double UP_SPEED = -0.8;
    public static double DOWN_SPEED = 0.8;

    private double encoderOffset = 0;
    private double encoderOffset2 = 0;

    public static int RESTING_POS = 5;
    public static int GROUND_POS = -30;
    public static int LOW_POS = -663;
    public static int MID_POS = -1149;
    public static int HIGH_POS = -1240;

    public static int AUTO_MID_POS = -1015;
    public static int AUTO_HIGH_POS = -1240;


    //Auto Slide Positions
    public static int CONE_STACK_POS = -236;
    public static int CONE_5_POS = -135;
    public static int CONE_4_POS = -126;
    public static int CONE_3_POS = -87;
    public static int CONE_2_POS = -68;
    public static int CONE_1_POS = -10;
    double output = 0;

    public static boolean lowBool = false;
//    private static double POWER = 1.4;
//    LOW_POWER =0.7;

//    private static int liftPos = 0;

    public enum LiftPos{
        REST,
        GROUND, LOW, MID, HIGH,
        AUTO_MID, AUTO_HIGH,

        CONE_STACK, FIVE, FOUR, THREE, TWO, ONE
    }
    LiftPos liftPos = LiftPos.REST;


    public Slide( Telemetry tl, HardwareMap hw) {
//        this.slideM1 = slideM1;
//        this.slideM2 = slideM2;

        slideM1 = new MotorEx(hw, "lift");
        slideM2 = new MotorEx(hw, "lift2");

        //Reverse lift motor
        slideM1.setInverted(true);
        //this.slideMotor2.setInverted(true);

        slideM1.resetEncoder();
        slideM2.resetEncoder();

        slideM1.setDistancePerPulse(360 / CPR);
        slideM2.setDistancePerPulse(360 / CPR);

        upController = new PIDFController(pidfUpCoefficients.p, pidfUpCoefficients.i, pidfUpCoefficients.d, pidfUpCoefficients.f, getAngle(), getAngle());
        upController.setTolerance(10);

        this.telemetry = tl;
        slideAutomatic = false;
        liftPos = LiftPos.REST;
        setOffset();
    }

    @Override
    public void periodic() {
        if (slideAutomatic) {
            upController.setF(pidfUpCoefficients.f * Math.cos(Math.toRadians(upController.getSetPoint())));

            output = upController.calculate(getAngle());
//            if (output >= 1) output = 1;
//            if (output <= -1) output = -1;

            slideM1.set(output );
            slideM2.set(output);

//            if (lowBool) {
//                slideM1.set(output * LOW_POWER);
//                slideM2.set(output * LOW_POWER);
//            }
//            else {
//                slideM1.set(output * POWER);
//                slideM2.set(output * POWER);
//            }
        }
        telemetry.addLine("Slide - ");
        telemetry.addData("     Lift Motor Output:", output);

        telemetry.addData("     Lift1 Encoder: ", slideM1.getCurrentPosition());
        telemetry.addData("     Lift2 Encoder: ", slideM2.getCurrentPosition());
        telemetry.addData("     List Pos:", liftPos);
    }

    private double getEncoderDistance() {
        return slideM1.getDistance() - encoderOffset;
    }

    private double getEncoderDistance2(){
        return slideM2.getDistance() - encoderOffset2;
    }

    public void upSlideManual(){
        slideAutomatic = false;
        slideM1.set(UP_SPEED);
        slideM2.set(UP_SPEED);
//        slideAutomatic = true;
////        if((HIGH_POS<slideM1.getCurrentPosition())){
//            upController.setSetPoint(slideM1.getCurrentPosition()-20);
////        }
////        else return;

    }
    public void downSlideManual() {
        slideAutomatic = false;
        slideM1.set(DOWN_SPEED);
        slideM2.set(DOWN_SPEED);
//        slideAutomatic = true;
////        if((-15>slideM1.getCurrentPosition())){
//            upController.setSetPoint(slideM1.getCurrentPosition()+20);
////        }
////        else return;
    }

    public void setPower(double power) {
        slideM1.set(power);
        slideM2.set(power);
    }



    public void stopSlide() {
        slideM1.stopMotor();
        upController.setSetPoint(getAngle());
        slideM2.stopMotor();
        slideAutomatic = false;
    }

//    public void setAutomatic(boolean auto) {
//        this.automatic = auto;
//    }

    public void resetEncoder() {
//        liftEncoderReset();
    }

    public double getAngle() {
        return getEncoderDistance();
    }

    public double getAngle2(){
        return getEncoderDistance2();
    }

    /****************************************************************************************/


    public void slideResting() {
        slideAutomatic = true;
        lowBool = true;
        upController.setSetPoint(RESTING_POS);
        liftPos = LiftPos.REST;
    }

    public void encoderReset() {
        slideM1.resetEncoder();
        slideM2.resetEncoder();
        telemetry.addLine("SLIDE RESET");
    }

    public void slideGround() {
        slideAutomatic = true;
        lowBool = true;
        upController.setSetPoint(GROUND_POS);
        liftPos = LiftPos.GROUND;
    }
    public void slideLow() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(LOW_POS);
        liftPos = LiftPos.LOW;
    }
    public void slideMid() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(MID_POS);
        liftPos = LiftPos.MID;
    }
    public void slideHigh() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(HIGH_POS);
        liftPos = LiftPos.HIGH;
    }


    public void slideCone5() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(CONE_5_POS);
        liftPos = LiftPos.FIVE;
    }
    public void slideCone4() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(CONE_4_POS);
        liftPos = LiftPos.FOUR;
    }
    public void slideCone3() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(CONE_3_POS);
        liftPos = LiftPos.THREE;
    }
    public void slideCone2() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(CONE_2_POS);
        liftPos = LiftPos.TWO;
    }
    public void slideCone1() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(CONE_1_POS);
        liftPos = LiftPos.ONE;
    }

//    public void autoPickSlideUp() {
//        slideAutomatic = true;
//        lowBool = false;
//        upController.setSetPoint(slideM1.getCurrentPosition()-200);
////        liftPosition = 10;
//    }
//    public void autoDropSlideUp() {
//        slideAutomatic = true;
//        lowBool = false;
//        upController.setSetPoint(slideM1.getCurrentPosition()+200);
////        liftPosition = 11;
//    }

    public void slidePickUp(){
        slideAutomatic = true;
        upController.setSetPoint(slideM1.getCurrentPosition()-30);
    }
    public void slideAutoMid(){
        slideAutomatic = true;
        upController.setSetPoint(AUTO_MID_POS);
        liftPos = LiftPos.AUTO_MID;
    }
    public void slideConeStack() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(CONE_STACK_POS);
        liftPos = LiftPos.CONE_STACK;
    }
    public void slideAutoHigh() {
        slideAutomatic = true;
        lowBool = false;
        upController.setSetPoint(AUTO_HIGH_POS);
        liftPos = LiftPos.AUTO_HIGH;
    }


//    public void setLift(double angle) {
//        slideAutomatic = true;
//        upController.setSetPoint(angle);
//    }
//    public boolean atTargetAngle() {
//        return upController.atSetPoint();
//    }


    public void setOffset() {
        resetEncoder();
        upController.setSetPoint(getAngle());
    }
    public boolean isSlideAutomatic(){
        return slideAutomatic;
    }

    public void dropSlide(){
        switch (liftPos){
            case LOW:
                upController.setSetPoint(LOW_POS+350);
                break;
            case MID:
                upController.setSetPoint(MID_POS+500);
                break;
            case HIGH:
                upController.setSetPoint(HIGH_POS+400);
                break;
            case AUTO_MID:
                upController.setSetPoint(AUTO_MID_POS+500);
                break;
            case AUTO_HIGH:
                upController.setSetPoint(AUTO_HIGH_POS+120);
                break;
        }
    }

    public void voltage() {
//        if (voltreading > pixyMax || voltreading < pixyMin) {
//            telemetry.addData("Out of Range", "");
//            telemetry.update();
//        } else if (voltreading > pixyCenter && xPosition > turretLow && xPosition < turretHigh) {
//            if (voltreading > (pixyCenter + 1.7)) {
//                xPosition = xPosition + 200;
//            } else if (voltreading > (pixyCenter + 1.28)) {
//                xPosition = xPosition + 75;
//            } else if (voltreading > (pixyCenter + .885)) {
//                xPosition = xPosition + 40;
//            } else if (voltreading > (pixyCenter + .49)) {
//                xPosition = xPosition + 15;
//            } else if (voltreading > (pixyCenter + .1)) {
//                xPosition = xPosition + 7;
//            } else if (voltreading > (pixyCenter + .05)) {
//                xPosition = xPosition + 1;
//            } else if (voltreading > (pixyCenter + deadband)) {
//                telemetry.addData("Pixy Centered",":)");
//            }
//        } else if (voltreading < pixyCenter) {
//            if (voltreading < (pixyCenter - 1.7)) {
//                xPosition = xPosition - 200;
//            } else if (voltreading < (pixyCenter - 1.28)) {
//                xPosition = xPosition - 75;
//            } else if (voltreading < (pixyCenter - .885)) {
//                xPosition = xPosition - 40;
//            } else if (voltreading < (pixyCenter - .49)) {
//                xPosition = xPosition - 15;
//            } else if (voltreading < (pixyCenter - .1)) {
//                xPosition = xPosition - 7;
//            } else if (voltreading < (pixyCenter - .05)) {
//                xPosition = xPosition - 1;
//            } else if (voltreading < (pixyCenter - deadband)) {
//                telemetry.addData(":) ","Pixy Centered");
//            }
//        }
    }

}