/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmode.auto.old.left;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.old.auto.LeftSpline;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
public class LeftSplineAuto extends MatchOpMode
{
    private int tagNum = 0;

    // Subsystems
    private Pivot pivot;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;
    private TurnServo turnServo;
//    private SensorColor sensorColor;

    private TrajectorySequence high;

    @Override
    public void robotInit() {
        claw = new Claw(telemetry, hardwareMap);
        pivot = new Pivot( telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide( telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        tagVision = new TagVision(hardwareMap, telemetry);;

//        high = LeftRegionalsPath.Constants.preLoad.toHigh.build(LeftRegionalsPath.Constants.PreLoad.startPose.getPose());
        while (!isStarted() && !isStopRequested())
        {
            tagVision.updateTagOfInterest();
            tagVision.tagToTelemetry();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
        tagNum = tagVision.getTag();

        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: { //Left
                autonGroup = new SequentialCommandGroup(
                        new LeftSpline(drivetrain, slide, pivot, claw, turnServo)
                );
            }
            case 2: { //Mid
                autonGroup = new SequentialCommandGroup(
                        new LeftSpline(drivetrain, slide, pivot, claw, turnServo)
//                        new DriveForwardCommand(drivetrain, 12)
                );

            }
            case 3: { //High
                autonGroup =new SequentialCommandGroup(
                        new LeftSpline(drivetrain, slide, pivot, claw, turnServo)
//                        new DriveForwardCommand(drivetrain, 32)
                );
            }
            default: {
                autonGroup = new SequentialCommandGroup(
                        new LeftSpline(drivetrain, slide, pivot, claw, turnServo)
//                        new DriveForwardCommand(drivetrain, 32)
                );
            }
        }
        schedule(autonGroup);
    }
}