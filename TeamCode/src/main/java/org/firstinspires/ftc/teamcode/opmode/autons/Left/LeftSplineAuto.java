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

package org.firstinspires.ftc.teamcode.opmode.autons.Left;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.Auto.LeftSpline;
import org.firstinspires.ftc.teamcode.subsystems.Drive.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Misc.TagVision;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

//@Disabled
@Autonomous
public class LeftSplineAuto extends MatchOpMode
{
    private int tagNum = 0;

    // Subsystems
    private Arm arm;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;
    private TurnServo turnServo;
//    private SensorColor sensorColor;


    @Override
    public void robotInit() {
        claw = new Claw(telemetry, hardwareMap);
        arm = new Arm( telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide( telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
//        sensorColor = new SensorColor(hardwareMap, telemetry);
        tagVision = new TagVision(hardwareMap, telemetry);
//        sensorColor = new SensorColor(hardwareMap, telemetry);
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
                        new LeftSpline(drivetrain, slide, arm, claw, turnServo)
                );
            }
            case 2: { //Mid
                autonGroup = new SequentialCommandGroup(
                        new LeftSpline(drivetrain, slide, arm, claw, turnServo)
//                        new DriveForwardCommand(drivetrain, 12)
                );

            }
            case 3: { //High
                autonGroup =new SequentialCommandGroup(
                        new LeftSpline(drivetrain, slide, arm, claw, turnServo)
//                        new DriveForwardCommand(drivetrain, 32)
                );
            }
            default: {
                autonGroup = new SequentialCommandGroup(
                        new LeftSpline(drivetrain, slide, arm, claw, turnServo)
//                        new DriveForwardCommand(drivetrain, 32)
                );
            }
        }
        schedule(autonGroup);
    }
}