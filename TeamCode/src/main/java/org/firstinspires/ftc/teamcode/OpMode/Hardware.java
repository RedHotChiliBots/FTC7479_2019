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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware {

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQvZgMD/////AAABmZquHHM/akuGkmTcIcssi+gINVzua6tbuI6iq9wY3ypvUkndXoRQncprZtLgjoNzaAZx4jTucekE90oZj0G/CqgXL1uzhrV4+knSziKUwgFVy3SVvGzw0+/ZqHVFwAFe6wsty2B2Mxg+uIoAFq7tB5WRB6GMx1j47m9q7+hkx3+KOKasSiO/T8Fd/nehQkRVBwB1XJNEo28R0yicJfdGkhxgJOK/CGTkN49MooMjaSx1PFpgx2Bx8wxJwNMcOxzh3zYeiwddMZvsycSf3h2WTDHBHeFkW+f00i0071LJRaawELtRmIxP/pmV2Squ/1daGYjLGKveSPH5tBIHiQvGwdAnv3QrRZnhEf6ztG9eELEs";

    VuforiaLocalizer.Parameters parameters;
//    private WebcamName webcamName = null;


    /* Public OpMode members. */
    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftRearDrive    = null;
    private DcMotor rightRearDrive   = null;

    private DcMotor boomMotor   = null;

    private Servo foundationServo  = null;
    private Servo stoneServo  = null;
    private Servo turretServo  = null;
    private Servo wristServo  = null;
    private Servo clawServo = null;

    private final double FDTN_UP  = 0.5;
    private final double FDTN_DN  = 0.25;
    private final double FDTN_STOW  = FDTN_UP;

    private final double STONE_UP  = 0.65;
    private final double STONE_DN  = 0.0;
    private final double STONE_STOW  = STONE_UP;

    private final double CLAW_OPEN  = 0.75;
    private final double CLAW_CLOSED  = 0.0;
    private final double CLAW_STOW  = CLAW_CLOSED;

    private final double TURRET_MAX  = 0.75;
    private final double TURRET_MIN  = 0.0;
    private final double TURRET_STOW  = TURRET_MIN;

    private final double WRIST_MAX  = 0.0;
    private final double WRIST_MIN  = 0.75;
    private final double WRIST_STOW  = WRIST_MAX;

    private enum DDIR {FORWARD, REVERSE}
    private enum DHALF {FULL, HALF}
    public enum POS1 {OPEN, CLOSED, STOW}
    public enum POS2 {UP, DOWN, STOW}
    public enum COLOR {RED,BLUE,OTHER}
    public enum TRACK {TRACKING,STOPPED,UNKNOWN}

    private Map<POS1, Double> clawPos = new HashMap<>();
    private Map<POS2, Double> stonePos = new HashMap<>();
    private Map<POS2, Double> foundationPos = new HashMap<>();

    private DDIR driveDir = DDIR.FORWARD;
    private DHALF driveHalfSpeed = DHALF.FULL;

    private double turretPosition = 0.0;
    private double wristPosition = 0.0;
    private POS1 clawPosition = null;
    private POS2 fntnPosition = null;
    private POS2 stonePosition = null;
    private TRACK trackState = null;
    private double leftDrive = 0.0;
    private double rightDrive = 0.0;
    private double boomSpeed = 0.0;

    /* local OpMode members. */
    private HardwareMap hwMap  =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //**************************************************************/
        //********** Define and Initialize Vuforia & Camera ************/
        //**************************************************************/

//        webcamName = ahwMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

         //* We also indicate which camera on the RC we wish to use.
//        parameters.cameraName = webcamName;

        setTrackState(TRACK.UNKNOWN);

        //**********************************************************/
        //********** Initialize Constant Values         ************/
        //**********************************************************/
        clawPos.put(POS1.OPEN, CLAW_OPEN);
        clawPos.put(POS1.CLOSED, CLAW_CLOSED);
        clawPos.put(POS1.STOW, CLAW_STOW);

        stonePos.put(POS2.UP, STONE_UP);
        stonePos.put(POS2.DOWN, STONE_DN);
        stonePos.put(POS2.STOW, STONE_STOW);

        foundationPos.put(POS2.UP, FDTN_UP);
        foundationPos.put(POS2.DOWN, FDTN_DN);
        foundationPos.put(POS2.STOW, FDTN_STOW);

        //**********************************************************/
        //********** Define and Initialize Drive Motors ************/
        //**********************************************************/
        // Define each drive motor
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hwMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive");

        // Initialize drive motors to correct rotation
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Initialize all drive motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        // Initialize all drive motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define each drive motor
        boomMotor = hwMap.get(DcMotor.class, "boomMotor");

        // Initialize  motor to correct rotation
        boomMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        // Initialize all motor to zero power
        boomMotor.setPower(0);

        // Initialize motor to run with encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        boomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //**********************************************************/
        //************* Define and Initialize Servos ***************/
        //**********************************************************/

        turretServo = hwMap.get(Servo.class, "turretServo");

        setTurret(TURRET_STOW);

        wristServo = hwMap.get(Servo.class, "wristServo");

        setWrist(WRIST_STOW);

        clawServo = hwMap.get(Servo.class, "clawServo");

        setClaw(POS1.STOW);

        foundationServo = hwMap.get(Servo.class, "foundationServo");

        setFoundation(POS2.STOW);

        stoneServo = hwMap.get(Servo.class, "stoneServo");

        setStone(POS2.STOW);
    }


    void setTurret(double pos) {
        turretPosition = pos;
        turretServo.setPosition(pos);
    }

    double getTurret() {
        return turretPosition;
    }

    void setWrist(double pos) {
        wristPosition = pos;
        wristServo.setPosition(pos);
    }

    double getWrist() {
        return wristPosition;
    }

    void setClaw(POS1 pos) {
        clawPosition = pos;
        clawServo.setPosition(clawPos.get(pos));
    }

    POS1 getClaw() {
        return clawPosition;
    }

    void setFoundation(POS2 pos) {
        fntnPosition = pos;
        foundationServo.setPosition(foundationPos.get(pos));
    }

    POS2 getFoundation() {
        return fntnPosition;
    }

    void setStone(POS2 pos) {
        stonePosition = pos;
        stoneServo.setPosition(stonePos.get(pos));
    }

    POS2 getStone() {
        return stonePosition;
    }

    void setDriveSpeed(double l, double r) {
        if (driveHalfSpeed == DHALF.HALF) {
            l /= 2.0;
            r /= 2.0;
        }
        leftDrive = l;
        rightDrive = r;
        if (driveDir == DDIR.FORWARD) {
            leftFrontDrive.setPower(l);
            rightFrontDrive.setPower(r);
            leftRearDrive.setPower(l);
            rightRearDrive.setPower(r);
        } else {
            leftFrontDrive.setPower(-r);
            rightFrontDrive.setPower(-l);
            leftRearDrive.setPower(-r);
            rightRearDrive.setPower(-l);
        }
    }

    List<Double> getDriveSpeed() {
        return Arrays.asList(leftDrive, rightDrive);
    }

    void setDriveMecanum(double lx, double ly, double rx) {
        double magnitude = Math.hypot(lx, ly);
        double robotAngle = Math.atan2(ly, lx) + Math.PI / 4;
        double fld = magnitude * Math.cos(robotAngle) + rx;
        double frd = magnitude * Math.sin(robotAngle) - rx;
        double bld = magnitude * Math.sin(robotAngle) + rx;
        double brd = magnitude * Math.cos(robotAngle) - rx;
        leftFrontDrive.setPower(fld);
        rightFrontDrive.setPower(frd);
        leftRearDrive.setPower(bld);
        rightRearDrive.setPower(brd);
    }

    void setDriveHalfSpeed(DHALF h) {
        driveHalfSpeed = h;
    }

    void toggleHalfSpeed() {
        if (driveHalfSpeed == DHALF.FULL) {
            driveHalfSpeed = DHALF.HALF;
        } else {
            driveHalfSpeed = DHALF.FULL;
        }
    }

    DHALF getDriveHalfSpeed() {
        return driveHalfSpeed;
    }

    void setDriveDir(DDIR d) {
        driveDir = d;
    }

    void toggleDriveDir() {
        if (driveDir == DDIR.FORWARD) {
            driveDir = DDIR.REVERSE;
        } else {
            driveDir = DDIR.FORWARD;
        }
    }

    DDIR getDriveDir() {
        return driveDir;
    }

    void setBoomSpeed(double s) {
        boomSpeed = s * 0.5;
        boomMotor.setPower(boomSpeed);
    }

    double getBoomSpeed() {
        return boomSpeed;
    }

    void setTrackState(TRACK s) {
        trackState = s;
    }

    TRACK getTrackState() {
        return trackState;
    }
}

