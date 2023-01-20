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

package org.firstinspires.ftc.teamcode.autons.Autons.Left;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Misc.Vision.TagVision;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Disabled
@Autonomous
public class LeftMidAuton extends MatchOpMode
{
    private int tagNum = 0;

    // Subsystems
    private Arm arm;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;
    private TurnServo turnServo;


    @Override
    public void robotInit() {
        claw = new Claw(telemetry, hardwareMap);
        arm = new Arm( telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide( telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);

        tagVision = new TagVision(hardwareMap, telemetry);
//        tagVision.init(hardwareMap);
        while (!isStarted() && !isStopRequested())
        {
            tagVision.updateTagOfInterest();
            tagVision.tagToTelemetry();
            telemetry.update();
//            new InstantCommand(tagVision::getsend, tagVision);

        }
        this.matchStart();
    }

    public void matchStart() {
        tagNum = tagVision.getTag();

        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: { //Left
                autonGroup = new SequentialCommandGroup(
//                        new LeftHighAutonCommand(drivetrain, slide, clawMotors, claw),
//                        new DriveForwardCommand(drivetrain, 28)
                );
            }
            case 2: { //Mid
                autonGroup = new SequentialCommandGroup(
//                        new LeftHighAutonCommand(drivetrain, slide, clawMotors, claw),
//                        new DriveForwardCommand(drivetrain, 20)
                );

            }
            case 3: { //High
                autonGroup =new SequentialCommandGroup(
//                        new LeftHighAutonCommand(drivetrain, slide, clawMotors, claw),
//                        new DriveForwardCommand(drivetrain, 19)
                );
            }
            default: {
                autonGroup = new SequentialCommandGroup(
//                        new LeftHighAutonCommand(drivetrain, slide, clawMotors, claw),
//                        new DriveForwardCommand(drivetrain, 20)
                );
            }
        }
        schedule(autonGroup);
    }
}