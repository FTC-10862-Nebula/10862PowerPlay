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

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.00, 0.000, 0, 0);
    //I = 0.0008
    private PIDFController controller;
    private boolean automatic;

    public static double CPR = 384.5;
    public static double UP_SPEED = -0.2;
    public static double DOWN_SPEED = 0.2;

    private double encoderOffset = 0;

    public static int INTAKE_POS_FRONT = -300;
    public static int GROUND_POS_FRONT = 300;
    public static int LOW_POS_FRONT = 280;
    public static int MID_POS_FRONT = 350;
    public static int HIGH_POS_FRONT = 350;

    public static int INIT_POS = 0;

    public static int INTAKE_POS_BACK = -300;
    public static int GROUND_POS_BACK = -300;
    public static int LOW_POS_BACK = -280;
    public static int MID_POS_BACK = -350;
    public static int HIGH_POS_BACK = -350;


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

        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        if (automatic) {
            controller.setF(pidfCoefficients.f * Math.cos(Math.toRadians(controller.getSetPoint())));

            double output = controller.calculate(getAngle());

            clawMotor.set(output);
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

    public void moveClawIntakeFront() {
        automatic = true;
        controller.setSetPoint(INTAKE_POS_FRONT);
        clawPos = 0;
    }
    public void moveClawGroundFront() {
        automatic = true;
        controller.setSetPoint(GROUND_POS_FRONT);
        clawPos = 1;
    }
    public void moveClawLowFront() {
        automatic = true;
        controller.setSetPoint(LOW_POS_FRONT);
        clawPos = 2;
    }
    public void moveClawMidFront() {
        automatic = true;
        controller.setSetPoint(MID_POS_FRONT);
        clawPos = 3;
    }
    public void moveClawHighFront() {
        automatic = true;
        controller.setSetPoint(HIGH_POS_FRONT);
        clawPos = 4;
    }

    public void moveClawIntakeBack() {
        automatic = true;
        controller.setSetPoint(INTAKE_POS_FRONT);
        clawPos = 5;
    }
    public void moveClawGroundBack() {
        automatic = true;
        controller.setSetPoint(GROUND_POS_FRONT);
        clawPos = 6;
    }
    public void moveClawLowBack() {
        automatic = true;
        controller.setSetPoint(LOW_POS_FRONT);
        clawPos = 7;
    }
    public void moveClawMidBack() {
        automatic = true;
        controller.setSetPoint(MID_POS_FRONT);
        clawPos = 8;
    }
    public void moveClawHighBack() {
        automatic = true;
        controller.setSetPoint(HIGH_POS_FRONT);
        clawPos = 9;
    }
    public void encoderReset() {
        clawMotor.resetEncoder();
    }

    /****************************************************************************************/


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
            moveClawIntakeFront();
        } else if(clawPos == 1) {
            moveClawGroundFront();
        } else if(clawPos == 2) {
            moveClawLowFront();
        } else if(clawPos == 3) {
            moveClawMidFront();
        } else if(clawPos == 4) {
            moveClawHighFront();
        } else if(clawPos == 5) {
            moveClawIntakeBack();
        } else if(clawPos == 6) {
            moveClawGroundBack();
        } else if(clawPos == 7) {
            moveClawLowBack();
        } else if(clawPos == 8) {
            moveClawMidBack();
        } else if(clawPos == 9) {
            moveClawHighBack();
        }
    }
}
