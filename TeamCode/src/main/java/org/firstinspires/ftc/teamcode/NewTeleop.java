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


@TeleOp(name="NewHashIsABadSpelerTeleop")
//@Disabled
public class NewTeleop extends LinearOpMode {

    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor Flywheel;
    private CRServo GrabberL;
    private CRServo GrabberR;
    private DcMotor VerticalSlidePack;
    private double drive;
    private double turn;
    private final double DriveSpeed = 0.75;
    private final double GrabberOutPosition = 0.3;

    @Override
    public void runOpMode () {
        // Get motors, the expansion hub doesn't always properly connect which is why
        // the flywheel is commented out
        FLMotor = hardwareMap.dcMotor.get("1");
        FRMotor = hardwareMap.dcMotor.get("0");
        BLMotor = hardwareMap.dcMotor.get("2");
        BRMotor = hardwareMap.dcMotor.get("3");
        Flywheel = hardwareMap.dcMotor.get("Fly");
//        GrabberL = hardwareMap.crservo.get("GrabL");
//        GrabberR = hardwareMap.crservo.get("GrabR");
//        DcMotor HorizontalSlidePack = hardwareMap.dcMotor.get("HorizontalSlidePack");
//        VerticalSlidePack = hardwareMap.dcMotor.get("VSP");
//        DcMotor EaterMotor = hardwareMap.dcMotor.get("Eater");

        // Set Directions
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        Flywheel.setDirection(DcMotor.Direction.FORWARD);
//        GrabberL.setDirection(DcMotorSimple.Direction.FORWARD);
//        GrabberR.setDirection(DcMotorSimple.Direction.REVERSE);
//        HorizontalSlidePack.setDirection(DcMotor.Direction.FORWARD);
//        VerticalSlidePack.setDirection(DcMotor.Direction.FORWARD);
//        EaterMotor.setDirection(DcMotor.Direction.FORWARD);

        // wait for the coach to press start
        waitForStart();
        telemetry.addData("Status","TeleOp");
        telemetry.update();


        while(opModeIsActive()) {

            //Driver 1
//            drive = -gamepad1.left_stick_y;
//            turn = gamepad1.right_stick_x;

            //TANK DRIVE
            double LeftDrive = gamepad1.left_stick_y * DriveSpeed;
            double RightDrive = gamepad1.right_stick_y * DriveSpeed;

            // FLYWHEEL DIRECTION
//            boolean FlyWheelR = gamepad1.right_bumper;
//            boolean FlyWheelL = gamepad1.left_bumper;
//            double LeftDrive = Range.clip(drive + turn, -1.0, 1.0) * DriveSpeed;
//            double RightDrive = Range.clip(drive - turn, -1.0, 1.0) * DriveSpeed;

            //WHY ARE THERE TWO FLYWHEELS also strafe
            double FlywheelClockwise = gamepad1.left_bumper ? -1 : 0;
            double FlywheelCounterClockwise = gamepad1.right_bumper ? 1 : 0;
            double LeftStrafe = gamepad1.left_trigger > 0 ? 1 : 0;
            double RightStrafe = gamepad1.right_trigger > 0 ? 1 : 0;
//            double HorizontalSlidePackForward = gamepad2.right_bumper ? 1 : 0;
//            double HorizontalSlidePackBackward = gamepad2.right_trigger > 0 ? 1 : gamepad2.right_trigger < 0 ? -1 : 0;

            //FOR DA CLAW
            double VerticalSlidePackForward = gamepad2.dpad_up ? 1 : 0;
            double VerticalSlidePackBackward = gamepad2.dpad_down ? -1 : 0;
            double GrabberIn = gamepad2.circle ? 1 : 0;
            double GrabberOut = gamepad2.cross ? 1 : 0;

            if (FlywheelClockwise != 0 || FlywheelCounterClockwise != 0) {
                Flywheel.setPower(0.5 * (FlywheelCounterClockwise + FlywheelClockwise));
            } else {
                Flywheel.setPower(0);
            }

            if (GrabberIn > 0 || GrabberOut > 0) {
//                GrabberL.getController().setServoPosition(GrabberL.getPortNumber(), GrabberIn > 0 ? 0 : GrabberOutPosition);
//                GrabberR.getController().setServoPosition(GrabberR.getPortNumber(), GrabberIn > 0 ? 0 : GrabberOutPosition);
            }

//            if (HorizontalSlidePackBackward != 0 || HorizontalSlidePackForward != 0) {
//                HorizontalSlidePack.setPower(- HorizontalSlidePackBackward + HorizontalSlidePackForward);
//            }
//
            if (VerticalSlidePackBackward != 0 || VerticalSlidePackForward != 0) {
//                VerticalSlidePack.setPower(- VerticalSlidePackBackward + VerticalSlidePackForward);
            }
//
//            if (EaterBackward != 0 || EaterForward != 0) {
//                EaterMotor.setPower(- EaterBackward + EaterForward);
//            }

//            LeftDrive = DriveSpeed * (gamepad1.left_stick_y);
//            RightDrive = DriveSpeed * (gamepad1.right_stick_y);
             if (LeftStrafe == 0 && RightStrafe == 0) {
                 FLMotor.setPower(LeftDrive);
                 BLMotor.setPower(LeftDrive);
                 FRMotor.setPower(RightDrive);
                 BRMotor.setPower(RightDrive);
             } else if (LeftStrafe > 0) {
                 FLMotor.setPower(-1);
                 BLMotor.setPower(1);
                 FRMotor.setPower(-1);
                 BRMotor.setPower(1);
             } else if (RightStrafe > 0) {
                 FLMotor.setPower(1);
                 BLMotor.setPower(-1);
                 FRMotor.setPower(1);
                 BRMotor.setPower(-1);
             }
// Shane wrote this, this might not work. Yeah it probably doesn't THIS IS JAVA NOT PYTHON SHANE lol
//             if FlyWheelR{
//                 Flywheel.setPower(0.5);
//             }
//             else if FlyWheelL{
//                 Flywheel.setPower(-0.5) ;
//             }

//             if (LeftDrive != 0 && RightDrive != 0) {
//            FLMotor.setPower(LeftDrive);
//            BLMotor.setPower(LeftDrive);
//            FRMotor.setPower(RightDrive);
//            BRMotor.setPower(RightDrive);
            // }
            // else if (LeftDrive > 0) {
            //     FLMotor.setPower(LeftDrive);
            //     BLMotor.setPower(LeftDrive);
            //     FRMotor.setPower(LeftDrive * 0.5);
            //     BRMotor.setPower(LeftDrive * 0.5);
            // } else if (RightDrive > 0) {
            //     FLMotor.setPower(RightDrive * 0.5);
            //     BLMotor.setPower(RightDrive * 0.5);
            //     FRMotor.setPower(RightDrive);
            //     BRMotor.setPower(RightDrive);
            // } else {
            //     FLMotor.setPower(0 * 0.5);
            //     BLMotor.setPower(0 * 0.5);
            //     FRMotor.setPower(0);
            //     BRMotor.setPower(0);
            // }

            // add random telemetry stuff (shows up on driver station app) cuz why not
            telemetry.addData("LeftDrive", LeftDrive);
            telemetry.addData("RightDrive", RightDrive);
            telemetry.addData("Flywheel", FlywheelCounterClockwise + FlywheelClockwise);
//            telemetry.addData("HorizontalSlidePack", -HorizontalSlidePackBackward + HorizontalSlidePackForward);
//            telemetry.addData("VerticalSlidePack", -VerticalSlidePackBackward + VerticalSlidePackForward);
            telemetry.addData("Grabber", -GrabberOut + GrabberIn);

            telemetry.update();
        }
    }
}