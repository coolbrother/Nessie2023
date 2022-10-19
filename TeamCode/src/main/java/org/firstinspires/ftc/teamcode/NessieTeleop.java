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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="NessieTeleop")
//@Disabled
public class NessieTeleop extends LinearOpMode {
    
    enum SlidePackDirection {
        UP,
        DOWN
    }

    enum PoleHeight {
        HIGH,
        MEDIUM,
        LOW,
        GROUND
    }

    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor Flywheel;
    private CRServo GrabberL;
    private Servo GrabberR;
    private CRServo Spinner;
    private DcMotor VerticalSlidePack;
    private double drive;
    private double turn;
    private final double DriveSpeed = 0.9;
    private final double SlidePackSpeed = 0.5;
//     private final double GrabberLGrabPosition = 0.2;
//     private final double GrabberLReleasePosition = 0.4;
    private final double GrabberRGrabPosition = 0.23;
    private final double GrabberRReleasePosition = 0.0;
    private final double SpinnerForwardPosition = 0.91;
    private final double SpinnerBackwardPosition = 0.25;
    private PoleHeight CurrentPoleHeight = PoleHeight.GROUND;

    @Override
    public void runOpMode () {
        // Get motors, the expansion hub doesn't always properly connect which is why
        // the flywheel is commented out
        FLMotor = hardwareMap.dcMotor.get("1");
        FRMotor = hardwareMap.dcMotor.get("0");
        BLMotor = hardwareMap.dcMotor.get("2");
        BRMotor = hardwareMap.dcMotor.get("3");
//         Flywheel = hardwareMap.dcMotor.get("Fly");
//         GrabberL = hardwareMap.crservo.get("GL");
        GrabberR = hardwareMap.servo.get("GR");
        Spinner = hardwareMap.crservo.get("SP");
//        GrabberL = hardwareMap.crservo.get("GL");
//        GrabberR = hardwareMap.crservo.get("GR");
//        DcMotor HorizontalSlidePack = hardwareMap.dcMotor.get("HorizontalSlidePack");
        VerticalSlidePack = hardwareMap.dcMotor.get("VSP");
        // VerticalSlidePack.setTargetPosition(0);
        // VerticalSlidePack.setPower(0.3);
        // VerticalSlidePack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        DcMotor EaterMotor = hardwareMap.dcMotor.get("Eater");

        // Set Directions
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
//         Flywheel.setDirection(DcMotor.Direction.FORWARD);
//         GrabberL.setDirection(CRServo.Direction.FORWARD);
        GrabberR.setDirection(Servo.Direction.FORWARD);
        Spinner.setDirection(CRServo.Direction.FORWARD);
//        HorizontalSlidePack.setDirection(DcMotor.Direction.FORWARD);
        VerticalSlidePack.setDirection(DcMotor.Direction.REVERSE);
//        EaterMotor.setDirection(DcMotor.Direction.FORWARD);

        // wait for the coach to press start
        waitForStart();
        telemetry.addData("Status","TeleOp");
        telemetry.update();


        while(opModeIsActive()) {

            //Driver 1
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            //TANK DRIVE
            // double LeftDrive = -gamepad1.left_stick_y;// * DriveSpeed;
            // double RightDrive = -gamepad1.right_stick_y;// * DriveSpeed;
            
            double LeftDrive = Range.clip(drive + turn, -1.0, 1.0);
            double RightDrive = Range.clip(drive - turn, -1.0, 1.0);

            telemetry.addData("left_stick_y", LeftDrive);
            telemetry.addData("right_stick_y", RightDrive);
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("y", gamepad1.left_stick_y);

            if (LeftDrive > 0.9) {
                LeftDrive = DriveSpeed;
            } else if (LeftDrive > 0) {
                LeftDrive = 0.3 * DriveSpeed;
            } else if (LeftDrive < -0.9) {
                LeftDrive = -DriveSpeed;
            } else if (LeftDrive < 0) {
                LeftDrive = 0.3 * -DriveSpeed;
            } else {
                LeftDrive = 0;
            }

            if (RightDrive > 0.9) {
                RightDrive = DriveSpeed;
            } else if (RightDrive > 0) {
                RightDrive = 0.3 * DriveSpeed;
            } else if (RightDrive < -0.9) {
                RightDrive = -DriveSpeed;
            } else if (RightDrive < 0) {
                RightDrive = 0.3 * -DriveSpeed;
            } else {
                RightDrive = 0;
            }
            
            double LeftStrafe = gamepad1.left_trigger;
            double RightStrafe = gamepad1.right_trigger;

            // THE CLAW
            double VerticalSlidePackForward = -gamepad2.left_stick_y * SlidePackSpeed;
//            double VerticalSlidePackBackward = gamepad2.dpad_down ? -1 : 0;
            boolean GrabberIn = gamepad2.a;
            boolean GrabberOut = gamepad2.b;
            double SpinnerForward = gamepad2.right_stick_y;
            boolean GroundPoleHeight = gamepad2.dpad_down;
            boolean LowPoleHeight = gamepad2.dpad_left;
            boolean MediumPoleHeight = gamepad2.dpad_right;
            boolean HighPoleHeight = gamepad2.dpad_up;
            
            if (HighPoleHeight)
                VerticalSlidePack.setTargetPosition(20);

            if (GrabberIn || GrabberOut) {
//                 GrabberL.getController().setServoPosition(GrabberL.getPortNumber(), GrabberIn ? GrabberLGrabPosition : GrabberLReleasePosition);
                GrabberR.setPosition(GrabberIn ? GrabberRGrabPosition : GrabberRReleasePosition);
            }
            
            if (SpinnerForward != 0) {
                Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerForward > 0 ? SpinnerForwardPosition : SpinnerBackwardPosition);
            }
            
            // if (VerticalSlidePack.getCurrentPosition() > VSP_MAX_POSITION
                // || VerticalSlidePack.getCurrentPosition() < VSP_MIN_POSITION) {
                VerticalSlidePack.setPower(VerticalSlidePackForward);
            // }
            if (GroundPoleHeight)
                moveSlidePackToPosition(CurrentPoleHeight, PoleHeight.GROUND);
            else if (LowPoleHeight)
                moveSlidePackToPosition(CurrentPoleHeight, PoleHeight.LOW);
            else if (MediumPoleHeight)
                moveSlidePackToPosition(CurrentPoleHeight, PoleHeight.MEDIUM);
            else if (HighPoleHeight)
                moveSlidePackToPosition(CurrentPoleHeight, PoleHeight.HIGH);

            if (LeftStrafe == 0 && RightStrafe == 0) {
                FLMotor.setPower(LeftDrive);
                BLMotor.setPower(LeftDrive);
                FRMotor.setPower(RightDrive);
                BRMotor.setPower(RightDrive);
            } else if (LeftStrafe > 0) {
                if (LeftStrafe > 0.9) {
                    FLMotor.setPower(-1);
                    BLMotor.setPower(1);
                    FRMotor.setPower(1);
                    BRMotor.setPower(-1);
                } else {
                    FLMotor.setPower(-0.3);
                    BLMotor.setPower(0.3);
                    FRMotor.setPower(0.3);
                    BRMotor.setPower(-0.3);
                }
            } else if (RightStrafe > 0) {
                if (RightStrafe > 0.9) {
                    FLMotor.setPower(1);
                    BLMotor.setPower(-1);
                    FRMotor.setPower(-1);
                    BRMotor.setPower(1);
                } else {
                    FLMotor.setPower(0.3);
                    BLMotor.setPower(-0.3);
                    FRMotor.setPower(-0.3);
                    BRMotor.setPower(0.3);
                }
            }


            // add random telemetry stuff (shows up on driver station app) cuz why not
            telemetry.addData("LeftDrive", LeftDrive);
            telemetry.addData("RightDrive", RightDrive);
//             telemetry.addData("Flywheel", FlywheelCounterClockwise + FlywheelClockwise);
//            telemetry.addData("HorizontalSlidePack", -HorizontalSlidePackBackward + HorizontalSlidePackForward);
            telemetry.addData("VerticalSlidePack", VerticalSlidePackForward);
            telemetry.addData("VerticalSlidePackPosition", VerticalSlidePack.getCurrentPosition());
            telemetry.addData("GrabberIn", GrabberIn);
            telemetry.addData("GrabberOut", GrabberOut);
            telemetry.addData("GroundPoleHeight", GroundPoleHeight);
            telemetry.addData("LowPoleHeight", LowPoleHeight);
            telemetry.addData("MediumPoleHeight", MediumPoleHeight);
            telemetry.addData("HighPoleHeight", HighPoleHeight);
            // telemetry.addData("GrabberLPosition", GrabberL.getController().getServoPosition(GrabberL.getPortNumber()));
            telemetry.addData("GrabberRPosition", GrabberR.getController().getServoPosition(GrabberR.getPortNumber()));
            telemetry.addData("SpinnerPosition", Spinner.getController().getServoPosition(Spinner.getPortNumber()));
            telemetry.update();
        }
    }
    
    private void moveSlidePackToPosition(PoleHeight curPoleHeight, PoleHeight targetPoleHeight) {
        int timeToMove = getMoveTimeOfSlidePack(curPoleHeight, targetPoleHeight);
        // if (timeToMove >= 0)
        // moveSlidePack(SlidePackDirection.UP, -getDrivePower(SlidePackPower), timeToMove);
        // else
        // moveSlidePack(SlidePackDirection.DOWN, -getDrivePower(SlidePackPower), timeToMove);
        telemetry.addData("Moving To", targetPoleHeight);
        telemetry.update();
    }
    
    private int getMoveTimeOfSlidePack(PoleHeight curPoleHeight, PoleHeight targetPoleHeight) {
        return convertPoleHeightToMs(targetPoleHeight) - convertPoleHeightToMs(curPoleHeight);
    }

    private int convertPoleHeightToMs(PoleHeight ph) {
        switch (ph) {
            case HIGH:
                return 400;
            case MEDIUM:
                return 300;
            case LOW:
                return 200;
            case GROUND:
                return 100;
            default:
                return 0;
        }
    }
}
