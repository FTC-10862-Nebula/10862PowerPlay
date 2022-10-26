
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.driveTrain.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Date;
import java.util.List;
import java.util.logging.Level;


public class Drivetrain extends SubsystemBase {

    private final SampleMecanumDrive drive;
    private Telemetry telemetry;
    private BNO055IMU imu;

    public Drivetrain(SampleMecanumDrive drive, Telemetry tl) {
        this.drive = drive;
        this.telemetry = tl;
        //Add imu?
    }

    public void init() {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setMotorPowers(0, 0, 0, 0);
        drive.setPoseEstimate(new Pose2d());

        //init the imu
    }
    @Override
    public void periodic() {
        update();
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        drive.update();
    }

    public void mecDrive(double leftF, double leftR, double rightR, double rightF) {
        drive.setMotorPowers(leftF, leftR, rightR, rightF);
    }

//    public void arcadeDrive(double forward, double rotate) {
//        double maxInput = Math.copySign(Math.max(Math.abs(forward), Math.abs(rotate)), forward);
//        forward = clipRange(forward);
//        rotate = clipRange(rotate);
//
//
//        double[] wheelSpeeds = new double[2];
//        wheelSpeeds[0] = forward + rotate;
//        wheelSpeeds[1] = forward - rotate;
//
//        normalize(wheelSpeeds);
//
//        drive.setMotorPowers(wheelSpeeds[0], wheelSpeeds[1]);
//    }

    public void mecDrive(double y, double x, double rx) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double powers[] = new double[4];
//        powers [1] = (y + x + rx) / denominator;    //fLPower
//        powers [2] = (y - x + rx) / denominator;    //bLPower
//        powers [3] = (y - x - rx) / denominator;    //fRPower
//        powers [4] = (y + x - rx) / denominator;    //bRPower
//        Orginal Comp1

//        double fLPower = (y + x + rx) / denominator;
//        double backLPower = (y - x + rx) / denominator;
//        double frontRPower = (y + x - rx) / denominator;
//        double backRPower = (y - x - rx) / denominator;
        //Strafes (up/down) forward (right/left), turns opposite

         powers [1] =    (-y - x + rx) / denominator;
         powers [2] =     (y - x + rx) / denominator;
         powers [3] =   (-y - x - rx) / denominator;
         powers [4] =     (y - x - rx) / denominator;
        //Everthing but turning works- Test


//        double frontLPower = (-y - x - rx) / denominator;
//        double frontRPower = (y - x - rx) / denominator;
//        double backLPower = (-y + x - rx) / denominator;
//        double backRPower = (y + x - rx) / denominator;
        drive.setMotorPowers(powers[1], powers[2], powers[3], powers[4]);
    }

    public void  fieldCentric(double driveTurn, double gamepadXCord, double gamepadYCord){
        double gamepadHypot = Range.clip(Math.hypot(gamepadXCord, gamepadYCord), 0, 1);
        //finds just how much power to give the robot based on how much x and y given by gamepad
        //range.clip helps us keep our power within positive 1
        // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
        double gamepadDegree = Math.atan2(gamepadYCord, gamepadXCord);
        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        double robotDegree = getAngle();
        //gives us the angle our robot is at
        double movementDegree = gamepadDegree - robotDegree;

        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
        double gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the adjacent side, we can get our needed x value to power our motors
        double gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
        //by finding the opposite side, we can get our needed y value to power our motors

        //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values,
        //we can guarantee that our motor powers will not exceed 1 without any driveTurn
        //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
        //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving

        double frontRPower=(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
        double backRPower=(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
        double frontLPower=(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
        double backLPower=(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);

        drive.setMotorPowers(frontLPower, backLPower, frontRPower, backRPower);
    }



    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        drive.setWeightedDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }
    public double getHeading() {
        return Math.toDegrees(drive.getExternalHeading());
    }
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //works
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void followTrajectoryBlock(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public void turnTo(double radians) {
        drive.turnAsync(radians);
    }

    public void turnBlock(double radians) {
        drive.turn(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        mecDrive(0, 0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Returns minimum range value if the given value is less than
     * the set minimum. If the value is greater than the set maximum,
     * then the method returns the maximum value.
     *
     * @param value The value to clip.
     */
    public double clipRange(double value) {
        return value <= -1 ? -1
                : value >= 1 ? 1
                : value;
    }

    /**
     * Normalize the wheel speeds
     */
    protected void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    /**
     * Normalize the wheel speeds
     */
    protected void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if(maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }
    }
}