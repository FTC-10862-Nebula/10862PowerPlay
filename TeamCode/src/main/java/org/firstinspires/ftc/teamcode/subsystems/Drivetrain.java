
package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;

import java.util.List;


public class Drivetrain extends SubsystemBase {

    private final SampleMecanumDrive drive;
    private Telemetry telemetry;
    private BNO055IMU imu;
    private final int LFVal = 0,
                    LRVal = 1,
                    RFVal = 2,
                    RRVal = 3;

    double[] powers = new double[4];



    public Drivetrain(SampleMecanumDrive drive, Telemetry tl, HardwareMap hardwareMap) {
        this.drive = drive;
        this.telemetry = tl;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public void init() {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setMotorPowers(0, 0, 0, 0);
        drive.setPoseEstimate(new Pose2d());
//        imu = hardwareMap.get(BNO055IMU.class, "imu");

    }
    public void closeImu(){
        imu.close();
    }
    public Command inIMU(HardwareMap hM){
        imu = hM.get(BNO055IMU.class, "imu");
        return null;
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

    public void setPowers(double leftF, double leftR, double rightR, double rightF) {
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

//        Orginal Comp1
        powers [LFVal] = (y + x + rx) / denominator;    //fLPower
        powers [LRVal] = (y - x + rx) / denominator;    //bLPower
        powers [RFVal] = (y - x - rx) / denominator;    //fRPower
        powers [RRVal] = (y + x - rx) / denominator;    //bRPower
//        powers [LFVal] = (y + x + rx) / denominator;    //fLPower
//        powers [LRVal] = (y - x + rx) / denominator;    //bLPower
//        powers [RFVal] = (y - x - rx) / denominator;    //fRPower
//        powers [RRVal] = (y + x - rx) / denominator;    //bRPower
////        Orginal Comp1 for noral mec drive
//
////        powers [FLVal] = (y + x + rx) / denominator;
////        powers [FRVal] = (y - x + rx) / denominator;
////        powers [RFVal] = (y + x - rx) / denominator;
////        powers [RRVal] = (y - x - rx) / denominator;
//        //Strafes (up/down) forward (right/left), turns opposite
//
//         powers [LFVal] =    (y + x + rx) / denominator;
//         powers [LRVal] =     (y - x + rx) / denominator;
//         powers [RFVal] =   (y - x - rx) / denominator;
//         powers [RRVal] =     (y + x - rx) / denominator;
//        //Everthing but turning works- Test
//
//
////        double frontLPower = (-y - x - rx) / denominator;
////        double frontRPower = (y - x - rx) / denominator;
////        double backLPower = (-y + x - rx) / denominator;
////        double backRPower = (y + x - rx) / denominator;
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
//
        powers [LFVal] = (-rotY -rotX- rx) / denominator;
        powers [LRVal] = (-rotY + rotX -   rx) / denominator;
        powers [RFVal] = (-rotY - rotX + rx) / denominator;
        powers [RRVal] = (-rotY + rotX + rx) / denominator;
        //WORKS!!!!!!!!!!!!!!!!


        drive.setMotorPowers(powers[LFVal], powers[LRVal], powers[RFVal], powers[3]);
//
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
        setPowers(0, 0, 0, 0);
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


//    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
//    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
//
//    //Test for trajectories and multithreading
//    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
//        return new TrajectorySequenceBuilder(
//                startPose,
//                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
//                MAX_ANG_VEL, MAX_ANG_ACCEL,
//                opMode
//
//        );
//    }
}