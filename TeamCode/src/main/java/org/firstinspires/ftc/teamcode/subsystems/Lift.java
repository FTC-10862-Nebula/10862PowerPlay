package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift extends Subsystem {
    public static class Constants {
        public static Hardware hardware;
        public static Controller controller;
        public static Position position;
        public static Speed speed;

        public static class Hardware {
            public static DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
            public static DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
            public static double
                    RPM = 435,
                    CPR = 751.8;

        }
        public static class Controller {
            public static double
                    P = 2,
                    I = 0,
                    D = 0,
                    F = 0;
            public static int TOLERANCE = 8;
        }
        public static class Position {
            public static int
                    HIGH = 1240,
                    MID = 735,
                    LOW = 315,
                    GROUND_JUNCTION = 30,
                    INITIAL = -5,
                    MAX_POSITION = 3000,
                    MIN_POSITION = -70,
                    AUTO_5CONE = 312,
                    AUTO_4CONE = 280,
                    AUTO_3CONE = 240,
                    AUTO_2CONE = 210,
                    AUTO_1CONE = 100;
        }
        public static class Speed {
            public static double NORMAL        = 10;   // ill use this but im testing something
            public static int MANUAL_MOVE_SPEED = 6;

        }
    }
    private final DcMotorEx SlideM1;
    private final DcMotorEx SlideM2;
    private int motorPosition;
    private boolean isMovingManually;




    public Lift(@NonNull OpMode opMode) {
        super(opMode);
        SlideM1 = opMode.hardwareMap.get(DcMotorEx.class, "SlideM1");
        SlideM2 = opMode.hardwareMap.get(DcMotorEx.class, "SlideM2");

        SlideM1.setDirection(Constants.Hardware.RIGHT_DIRECTION);
        SlideM2.setDirection(Constants.Hardware.LEFT_DIRECTION);

        // TODO: Find out what these constants are by default
//        leftLift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(Constants.Controller.P, Constants.Controller.I, Constants.Controller.D, Constants.Controller.F));
//        leftLift.setTargetPositionTolerance(Constants.Controller.TOLERANCE);

        SlideM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SlideM1.setTargetPosition(0);
        SlideM2.setTargetPosition(0);

        SlideM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }

    @Override
    public void periodic() {

    }

    @Override
    protected void manualControl() {
        if (opMode.gamepad2.dpad_down) moveInitial();
        else if(opMode.gamepad2.dpad_left) moveLow();
        else if(opMode.gamepad2.dpad_right) moveMid();
        else if (opMode.gamepad2.dpad_up) moveHigh();

        if (opMode.gamepad2.right_stick_y < -0.2 || opMode.gamepad2.right_stick_y > 0.2) {
            moveMotors((int)(motorPosition + Constants.Speed.MANUAL_MOVE_SPEED * -opMode.gamepad2.right_stick_y));
            isMovingManually = true;
        } else {
            isMovingManually = false;
        }

        opMode.telemetry.addData("Slide position", motorPosition);
        opMode.telemetry.addData("Slide P", SlideM1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p);
        opMode.telemetry.addData("Slide I", SlideM1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i);
        opMode.telemetry.addData("Slide D", SlideM1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d);
        opMode.telemetry.addData("Slide F", SlideM1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).f);
        opMode.telemetry.addData("Slide Tolerance", SlideM1.getTargetPositionTolerance());
    }

    public void moveMotors(int position) {
        if (position > Constants.Position.MAX_POSITION || position < Constants.Position.MIN_POSITION) return;

        this.motorPosition = position;

        SlideM1.setTargetPosition(position);
        SlideM2.setTargetPosition(position);

        SlideM1.setPower(1);
        SlideM2.setPower(1);
    }

    public void moveInitial() {
        if (isMovingManually) {
            Constants.Position.INITIAL = motorPosition;
        }
        moveMotors(Constants.Position.INITIAL);
    }

    public void moveHigh() {
        if (isMovingManually) {
            Constants.Position.HIGH = motorPosition;
        }
        moveMotors(Constants.Position.HIGH);
    }

    public void moveMid() {
        if (isMovingManually) {
            Constants.Position.MID = motorPosition;
        }
        moveMotors(Constants.Position.MID);
    }

    public void moveLow() {
        if (isMovingManually) {
            Constants.Position.LOW = motorPosition;
        }
        moveMotors(Constants.Position.LOW);
    }


    public void moveGroundJunction() {
        moveMotors(Constants.Position.GROUND_JUNCTION);
    }

    public void moveCone5() {
        moveMotors(Constants.Position.AUTO_5CONE);
    }

    public void moveCone4() {
        moveMotors(Constants.Position.AUTO_4CONE);
    }

    public void moveCone3() {
        moveMotors(Constants.Position.AUTO_3CONE);
    }

    public void moveCone2() {
        moveMotors(Constants.Position.AUTO_2CONE);
    }


    public boolean isDown() {
        return motorPosition <= Constants.Position.INITIAL;
    }

    public boolean isUp() {return motorPosition > Constants.Position.INITIAL;}
}