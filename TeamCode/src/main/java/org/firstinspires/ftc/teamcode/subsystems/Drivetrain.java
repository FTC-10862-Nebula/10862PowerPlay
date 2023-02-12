
package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;


public class Drivetrain extends SubsystemBase {

    private final MecanumDrive drive;
    private Telemetry telemetry;
    private BNO055IMU imu;
    private final int LFVal = 0,
            LRVal = 1,
            RFVal = 2,
            RRVal = 3;
//    private final int AUTOFIXLEFTANGLE = 90,
//    AUTOFIXUPANGLE = 270,
//    AUTOFIXRIGHTANGLE = -90;

    private final int AUTOFIXLEFTANGLE = -90,
            AUTOFIXUPANGLE = 0,
            AUTOFIXRIGHTANGLE = 90;

    double[] powers = new double[4];



    public Drivetrain(MecanumDrive drive, Telemetry tl, HardwareMap hardwareMap) {
        this.drive = drive;
        this.telemetry = tl;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public void init() {
        new Pose2d(0,0,0);
        drive.setMotorPowers(0, 0, 0, 0);
        setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        PoseStorage.currentPose = (new Pose2d(0, 0, Math.toRadians(0)));
    }

    //TODO: TEST!
    public void reInitializeIMU(){
//            imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());
    }


    public void setPowers(double leftF, double leftR, double rightR, double rightF) {
        drive.setMotorPowers(leftF, leftR, rightR, rightF);
    }

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
//        Original Comp1 for normal mec drive

//         powers [LFVal] =    (y + x + rx) / denominator;
//         powers [LRVal] =     (y - x + rx) / denominator;
//         powers [RFVal] =   (y - x - rx) / denominator;
//         powers [RRVal] =     (y + x - rx) / denominator;
//        //Everything but turning works- Test
//
//        double frontLPower = (-y - x - rx) / denominator;
//        double frontRPower = (y - x - rx) / denominator;
//        double backLPower = (-y + x - rx) / denominator;
//        double backRPower = (y + x - rx) / denominator;
        drive.setMotorPowers(powers[LFVal], powers[LRVal], powers[RFVal], powers[RRVal]);
    }

    public void  fieldCentric(double y, double x, double rx, int choice){
        double theta = 0;
        switch(choice){
            case 1:
                theta = -imu.getAngularOrientation().firstAngle+(AUTOFIXLEFTANGLE);
                break;
            case 2:
                theta = -imu.getAngularOrientation().firstAngle+(AUTOFIXUPANGLE);
                break;
            case 3:
                theta = -imu.getAngularOrientation().firstAngle+(AUTOFIXRIGHTANGLE);
                break;
        }
        double rotX = x * Math.cos(theta) - y * Math.sin(theta);
        double rotY = x * Math.sin(theta) + y * Math.cos(theta);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // ^^^^^^ Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        powers [LFVal] = (rotY + rotX - rx) / denominator;
        powers [LRVal] = (rotY - rotX - rx) / denominator;
        powers [RFVal] = (rotY + rotX + rx) / denominator;
        powers [RRVal] = (rotY - rotX + rx) / denominator;


        if(Math.abs(powers[LFVal])<0.25&Math.abs(powers[LRVal])<0.25&Math.abs(powers[RFVal])<0.25&Math.abs(powers[RRVal])<0.25){
            for (int i = 0; i <= 3; i++) {
                powers[i] = squareInput(powers[i]);
            }//TODO:Test
        }
        drive.setMotorPowers(powers[LFVal], powers[LRVal], powers[RFVal], powers[RRVal]);
    }

    private double squareInput(double input) {
        return input * Math.abs(input);
    }
    public double getHeading() {
        return Math.toDegrees(drive.getExternalHeading());
    }
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //works
    }

//    public void stop() {
//        setPowers(0, 0, 0, 0);
//    }

//    private double clamp(double val, double min, double max) {
//        return Math.max(min, Math.min(max, val));
//    }

    /**
     * Returns minimum range value if the given value is less than
     * the set minimum. If the value is greater than the set maximum,
     * then the method returns the maximum value.
     *
     * value - The value to clip.
     */
//    public double clipRange(double value) {
//        return value <= -1 ? -1
//                : value >= 1 ? 1
//                : value;
//    }

    /*protected void normalize(double[] wheelSpeeds, double magnitude) {
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

//     Normalize the wheel speeds

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
    }*/

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




    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        drive.setWeightedDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
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

    public void followTrajectoryAsync(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }
    public void turnAsync(double angle) {
        drive.turnAsync(angle);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose);
    }


    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequence(trajectorySequence);
    }

    public int getRightAngle(){
        return drive.getRightAngle();
    }
    public int getLeftAngle(){
        return drive.getLeftAngle();

    }

}