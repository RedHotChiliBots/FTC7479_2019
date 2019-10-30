/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Library.Library;

import java.util.Arrays;
import java.util.List;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Auton SkyStone", group="Auton")
//@Disabled
public class Auton_SkyStone extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot           = new Hardware(); // use the class created to define a Pushbot's hardware
    VuforiaSkyStoneWebcam vu = new VuforiaSkyStoneWebcam();
    Library lib = new Library();

    private Hardware.COLOR      allianceColor = Hardware.COLOR.OTHER;
    private boolean skystone = false;
    private boolean foundation = false;
    private boolean park = false;

    private List<Double> speed = Arrays.asList(0.0, 0.0);
    private VuforiaTrackable rearWallTgt = null;
    private VuforiaTrackable frontWallTgt = null;
    private VuforiaTrackable parkWallTgt = null;
    private double turnSpeed = 0.0;

    private int stateCnt = 0;
    private ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {
        initOpmode();
        waitForStart();
        startOpmode();
        while (opModeIsActive())
        {
            loopOpmode();
            telemetry.update();
        }
        stopOpMode();
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void initOpmode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        telemetry.addData("Hardware", "Init");

        vu.init(robot.parameters);
        vu.targetsSkyStone.activate();
        telemetry.addData("Vuforia", "Init");

        allianceColor = FtcRobotControllerActivity.isRed() ? Hardware.COLOR.RED :
                (FtcRobotControllerActivity.isBlue() ? Hardware.COLOR.BLUE :
                        Hardware.COLOR.OTHER);
        skystone = FtcRobotControllerActivity.isSkyStone();
        foundation = FtcRobotControllerActivity.isFoundation();
        park = FtcRobotControllerActivity.isPark();

        if (allianceColor == Hardware.COLOR.RED) {
            frontWallTgt = vu.front1;
            rearWallTgt = vu.rear1;
            turnSpeed = 0.5;

        } else {
            frontWallTgt = vu.front2;
            rearWallTgt = vu.rear2;
            turnSpeed = -0.5;
        }
        parkWallTgt = rearWallTgt;

        telemetry.addData("Alliance", allianceColor);
        telemetry.addData("SkyStone", skystone);
        telemetry.addData("Foundation", foundation);
        telemetry.addData("Park", park);
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
//    @Override
//    public void init_loop() {
//     }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void startOpmode() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loopOpmode() {

        switch (stateCnt) {
            /**********************************************************/
            /* Deliver SkyStone if enabled                            */
            /**********************************************************/
            case 0: // Initialize
                if (skystone) {
                    stateCnt++;
                } else {
                    stateCnt = 50;
                }
                break;

            case 1: // Drive to Stone Image
                parkWallTgt = frontWallTgt;
                trackTarget(vu.stoneTarget, 3.0);
                if (robot.getTrackState() == Hardware.TRACK.STOPPED) {
                    stateCnt++;
                }
                break;

            case 2: // Control SkyStone
                robot.setStoneSpeed(robot.LEFT_IN, robot.RIGHT_IN);
                timer.reset();
                stateCnt++;
                break;

            case 3: // Wait 1 sec for Servo to drop
                if (timer.time() >= 1000) {
                    stateCnt++;
                }
                break;

            case 4: // Turn to Rear Wall Image
                robot.setDriveSpeed(turnSpeed, -turnSpeed);
                trackTarget(rearWallTgt, 36.0);
                if (robot.getTrackState() == Hardware.TRACK.TRACKING) {
                    stateCnt++;
                }
                break;

            case 5: // Track Rear Wall Image
                trackTarget(rearWallTgt, 36.0);
                if (robot.getTrackState() == Hardware.TRACK.STOPPED) {
                    stateCnt++;
                }
                break;

            case 6: // Release SkyStone
                robot.setStoneSpeed(robot.LEFT_OUT, robot.RIGHT_OUT);
                timer.reset();
                stateCnt++;
                break;

            case 7: // Wait 1 sec for Servo to raise
                if (timer.time() >= 1000) {
                    stateCnt++;
                }
                break;

            case 8:
                stateCnt = 50;
                break;

            /**********************************************************/
            /* Move Foundation if enabled                             */
            /**********************************************************/

            case 50:
                if (foundation) {
                    stateCnt++;
                } else {
                    stateCnt = 100;
                }
                break;

            case 51:
                stateCnt = 100;
                break;

            /**********************************************************/
            /* Park if enabled                                        */
            /**********************************************************/
            case 100:
                if (park) {
                    stateCnt++;
                } else {
                    stateCnt = 150;
                }
                break;

            case 101: // Turn to Front Wall Image
                robot.setDriveSpeed(turnSpeed, -turnSpeed);
                trackTarget(parkWallTgt, 72.0);
                if (robot.getTrackState() == Hardware.TRACK.TRACKING) {
                    stateCnt++;
                }
                break;

            case 102: // Track Front Wall Image
                trackTarget(parkWallTgt, 72.0);
                if (robot.getTrackState() == Hardware.TRACK.STOPPED) {
                    stateCnt++;
                }
                break;

            case 103:
                stateCnt = 150;
                break;

            case 150:    // Parked
                stateCnt++;
                break;

            default:
                stateCnt++;
        }

            // Send telemetry message to signify robot running;
        telemetry.addData("State",  "%3d", stateCnt);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stopOpMode() {
        // Disable Tracking when we are done;
        vu.targetsSkyStone.deactivate();
    }

    private void trackTarget(VuforiaTrackable target, double dist) {

        if (((VuforiaTrackableDefaultListener)target.getListener()).isVisible()) {
            vu.setVisible(true);

            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that call was made, or if the trackable is not currently visible.
            vu.setTransform(((VuforiaTrackableDefaultListener)target.getListener()).getUpdatedRobotLocation());

            if (vu.getPosLOS() > dist) {
                // calculate course adjustment if
                // we are off coarse by 2 inches or 2 degrees
                speed = lib.calcCorrection(vu.getPosOffset(), vu.getPosAngle(), vu.getYaw());
                robot.setTrackState(Hardware.TRACK.TRACKING);

            } else {
                speed = Arrays.asList(0.0, 0.0);
                robot.setTrackState(Hardware.TRACK.STOPPED);
            }
/*
            robot.setDriveSpeed(speed.get(0), speed.get(1));
*/
            telemetry.addData("Visible Target", target.getName());
            telemetry.addData("Pos (in)", "{Dist, Offset, Height} = %.1f, %.1f, %.1f", vu.getPosDist(), vu.getPosOffset(), vu.getPosHeight());
            telemetry.addData("Pos (in,deg)", "{LOS, Angle} = %.1f, %.1f", vu.getPosLOS(), vu.getPosAngle());
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Yaw} = %.0f, %.0f, %.0f", vu.getRoll(), vu.getPitch(), vu.getYaw());
            telemetry.addData("Speed",  "{left, right} = %4.2f %4.2f", robot.getDriveSpeed());

        } else {
            vu.setVisible(false);
            vu.setTransform(null);
            robot.setTrackState(Hardware.TRACK.UNKNOWN);
            telemetry.addData("Visible Target", "none");
        }
    }
}
