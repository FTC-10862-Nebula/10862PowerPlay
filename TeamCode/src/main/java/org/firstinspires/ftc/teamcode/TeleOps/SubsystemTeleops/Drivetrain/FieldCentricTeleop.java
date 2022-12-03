/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode.TeleOps.SubsystemTeleops.Drivetrain;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
@Disabled

/**
 * feel free to change the name or group of your class to better fit your robot
 */
@TeleOp (name = "FieldCentricDriveTest", group = "Test")
public class FieldCentricTeleop extends LinearOpMode {

    /**
     * make sure to change these motors to your team's preference and configuration
     */
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor leftRear;

    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {

        /**
         * you can change the variable names to make more sense
         */
        double driveTurn;
        //double driveVertical;
        //double driveHorizontal;

        double gpXCord,
                gpYCord,
                gamepadHypot,
                gamepadDegree,
                robotDegree,
                movementDegree,
                gpXControl,
                gpYControl;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //might need to change the motors being reversed
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(REVERSE);
        leftRear.setDirection(FORWARD);
        rightFront.setDirection(REVERSE);
        rightRear.setDirection(FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        /**
         * make sure you've configured your imu properly and with the correct device name
         */
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        while (opModeIsActive()) {
            gpYCord = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver
            gpXCord = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
            driveTurn = -gamepad1.right_stick_x;
            //driveVertical = -gamepad1.righft_stick_y;
            //driveHorizontal = gamepad1.right_stick_x;

            gamepadHypot = Range.clip(Math.hypot(gpXCord, gpYCord), 0, 1);
            //finds just how much power to give the robot based on how much x and y given by gamepad
            //range.clip helps us keep our power within positive 1
            // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
            gamepadDegree = Math.atan2(gpYCord, gpXCord);
            //the inverse tangent of opposite/adjacent gives us our gamepad degree
            robotDegree = getAngle();
            //gives us the angle our robot is at
            movementDegree = gamepadDegree - robotDegree;
            //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
            gpXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
            //by finding the adjacent side, we can get our needed x value to power our motors
            gpYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
            //by finding the opposite side, we can get our needed y value to power our motors


            //by mulitplying the gpYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
            //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
            //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving

            double frontLPower=( -gpYControl * Math.abs(gpYControl) - gpXControl * Math.abs(gpXControl) + driveTurn);
            double backLPower=(gpYControl * Math.abs(gpYControl) - gpXControl * Math.abs(gpXControl) + driveTurn);
            double frontRPower=(-gpYControl * Math.abs(gpYControl) - gpXControl * Math.abs(gpXControl) - driveTurn);
            double backRPower=(gpYControl * Math.abs(gpYControl) - gpXControl * Math.abs(gpXControl) - driveTurn);

            rightFront.setPower(frontRPower);
            rightRear.setPower(backRPower);
            leftFront.setPower(frontLPower);
            leftRear.setPower(backLPower);

            /*frontRight.setPower(driveVertical - driveHorizontal + driveTurn);
            backRight.setPower(driveVertical + driveHorizontal + driveTurn);
            frontLeft.setPower(driveVertical + driveHorizontal - driveTurn);
            backLeft.setPower(driveVertical - driveHorizontal - driveTurn);*/
        }
        telemetry.update();
    }

    void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
    }

    //allows us to quickly get our z angle
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}