
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


public class Drivetrain extends SubsystemBase {

    private final SampleMecanumDrive drive;
    private Telemetry telemetry;
    private BNO055IMU imu;
    private final int LFVal = 0,
                    LRVal = 1,
                    RFVal = 2,
                    RRVal = 3;

    double powers[] = new double[4];



    public Drivetrain(SampleMecanumDrive drive, Telemetry tl, HardwareMap hardwareMap) {
        this.drive = drive;
        this.telemetry = tl;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.close();
    }

    public void init() {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setMotorPowers(0, 0, 0, 0);
        drive.setPoseEstimate(new Pose2d());
//        imu = hardwareMap.get(BNO055IMU.class, "imu");

    }

    @Override
    public void periodic() {
        update();
    }

    public void resetImu(){
//        imu.close();
//        imu.initialize("imu");
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

//        powers [FLVal] = (y + x + rx) / denominator;    //fLPower
//        powers [FRVal] = (y - x + rx) / denominator;    //bLPower
//        powers [RFVal] = (y - x - rx) / denominator;    //fRPower
//        powers [RRVal] = (y + x - rx) / denominator;    //bRPower
//        Orginal Comp1

//        powers [FLVal] = (y + x + rx) / denominator;
//        powers [FRVal] = (y - x + rx) / denominator;
//        powers [RFVal] = (y + x - rx) / denominator;
//        powers [RRVal] = (y - x - rx) / denominator;
        //Strafes (up/down) forward (right/left), turns opposite

         powers [LFVal] =    (-y - x + rx) / denominator;
         powers [LRVal] =     (y - x + rx) / denominator;
         powers [RFVal] =   (-y - x - rx) / denominator;
         powers [RRVal] =     (y - x - rx) / denominator;
        //Everthing but turning works- Test


//        double frontLPower = (-y - x - rx) / denominator;
//        double frontRPower = (y - x - rx) / denominator;
//        double backLPower = (-y + x - rx) / denominator;
//        double backRPower = (y + x - rx) / denominator;
        drive.setMotorPowers(powers[LFVal], powers[LRVal], powers[RFVal], powers[RRVal]);
    }

    public void  fieldCentric(double y, double x, double rx){
        double theta = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(theta) - y * Math.sin(theta);
        double rotY = x * Math.sin(theta) + y * Math.cos(theta);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        powers [LFVal] = (rotY + rotX + rx) / denominator;
//        powers [LRVal] = (rotY - rotX + rx) / denominator;
//        powers [RFVal] = (rotY - rotX - rx) / denominator;
//        powers [RRVal] = (rotY + rotX - rx) / denominator;
//not work


//        powers [LFVal] = Math.sin(theta + Math.PI / 4);
//        powers [LRVal] = Math.sin(theta - Math.PI / 4);
//        powers [RFVal] = Math.sin(theta - Math.PI / 4);
//        powers [RRVal] = Math.sin(theta + Math.PI / 4);
        //Test 1 - suddenly turns to one direction without any input
//        powers [LFVal] = (rotY + rotX - rx) / denominator;
//        powers [LRVal] = (rotY + rotX - rx) / denominator;
//        powers [RFVal] = (rotY - rotX + rx) / denominator;
//        powers [RRVal] = (rotY - rotX + rx) / denominator;
        //Test2 - stafes when front/back; tueninf and strafing is badd

//        powers [LFVal] = (rotY + rotX + rx) / denominator;
//        powers [LRVal] = (rotY + rotX - rx) / denominator;
//        powers [RFVal] = (rotY - rotX - rx) / denominator;
//        powers [RRVal] = (rotY - rotX + rx) / denominator;
        //Test3 - everything but back and forth works

//        powers [LFVal] = (rotY + rotX - rx) / denominator;
//        powers [LRVal] = (rotY + rotX + rx) / denominator;
//        powers [RFVal] = (rotY - rotX + rx) / denominator;
//        powers [RRVal] = (rotY - rotX - rx) / denominator;
        //Test4 - turing is opp; y control makes bot strafe;

//        powers [LFVal] = (rotY + rotX + rx) / denominator;
//        powers [LRVal] = (rotY - rotX + rx) / denominator;
//        powers [RFVal] = (rotY - rotX - rx) / denominator;
//        powers [RRVal] = (rotY + rotX - rx) / denominator;

//        powers [LFVal] = Math.sin(theta + Math.PI / 4)+rx;
//        powers [LRVal] = Math.sin(theta - Math.PI / 4)-rx;
//        powers [RFVal] = Math.sin(theta - Math.PI / 4)+rx;
//        powers [RRVal] = Math.sin(theta + Math.PI / 4)-rx;
        //Tes just no - turns wih no inout

//        powers [LFVal] = (rotY + rotX + rx) / denominator;
//        powers [LRVal] = (rotY + rotX +   rx) / denominator;
//        powers [RFVal] = (rotY + rotX + rx) / denominator;
//        powers [RRVal] = (rotY + rotX + rx) / denominator;
//just strafes
        powers [LFVal] = (-rotY -rotX- rx) / denominator;
        powers [LRVal] = (-rotY + rotX -   rx) / denominator;
        powers [RFVal] = (-rotY - rotX + rx) / denominator;
        powers [RRVal] = (-rotY + rotX + rx) / denominator;
        //WORKS!!!!!!!!!!!!!!!!

//        powers [LFVal] = (-rotY +rotX+ rx) / denominator;
//        powers [LRVal] = (-rotY - rotX +   rx) / denominator;
//        powers [RFVal] = (-rotY + rotX - rx) / denominator;
//        powers [RRVal] = (-rotY - rotX - rx) / denominator;










//        powers [LFVal] = (y + x + rx) / denominator;    //fLPower
//        powers [LRVal] = (y - x + rx) / denominator;    //bLPower
//        powers [RFVal] = (y - x - rx) / denominator;    //fRPower
//        powers [RRVal] = (y + x - rx) / denominator;    //bRPower
////        Orginal Comp1 for noral mec drive

//        double gpHypot = Range.clip(Math.hypot(gpXCord, gpYCord), 0, 1);
//        //finds just how much power to give the robot based on how much x and y given by gamepad
//        //range.clip helps us keep our power within positive 1
//        // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
//        double gpDegree = Math.atan2(gpYCord, gpXCord);
//        //the inverse tangent of opposite/adjacent gives us our gamepad degree
//        double botDegree = this.getAngle();
//        //gives us the angle our robot is at
//        double movementDegree = gpDegree - botDegree;
//
//        //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
//        double gpXCtrl = Math.cos(Math.toRadians(movementDegree)) * gpHypot;
//        //by finding the adjacent side, we can get our needed x value to power our motors
//        double gpYCtrl = Math.sin(Math.toRadians(movementDegree)) * gpHypot;
//        //by finding the opposite side, we can get our needed y value to power our motors

        //by mulitplying the gpYCtrl and gpXCtrl by their respective absolute values,
        //we can guarantee that our motor powers will not exceed 1 without any driveTurn
        //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
        //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving

        //Strafing ____ straight front and back __ turn
//        powers [FLVal]=//Math.sin(botDegree + Math.PI / 4);       //(gpYCtrl * Math.abs(gpYCtrl) + gpXCtrl * Math.abs(gpXCtrl) - driveTurn);
//        powers [FRVal]=//Math.sin(botDegree - Math.PI / 4);       //(gpYCtrl * Math.abs(gpYCtrl) - gpXCtrl * Math.abs(gpXCtrl) - driveTurn);
//        powers [RFVal2]=//Math.sin(botDegree - Math.PI / 4);           //(gpYCtrl * Math.abs(gpYCtrl) - gpXCtrl * Math.abs(gpXCtrl) + driveTurn);
//        powers [3]=;//Math.sin(botDegree + Math.PI / 4);          //(gpYCtrl * Math.abs(gpYCtrl) + gpXCtrl * Math.abs(gpXCtrl) + driveTurn);

//   lF     Math.sin(theta + Math.PI / 4);
//       lR wheelSpeeds[2] = Math.sin(theta - Math.PI / 4);
//     rF   wheelSpeeds[1] = Math.sin(theta - Math.PI / 4);

//     rR   wheelSpeeds[3] = Math.sin(theta + Math.PI / 4);
        drive.setMotorPowers(powers[LFVal], powers[LRVal], powers[RFVal], powers[3]);
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