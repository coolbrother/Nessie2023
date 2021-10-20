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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class HashIsABadSpelerTeleOp extends LinearOpMode {

    @Override
    public void runOpMode () {

        DcMotor FLMotor = hardwareMap.dcMotor.get("FLMotor");
        DcMotor FRMotor = hardwareMap.dcMotor.get("FRMotor");
        DcMotor BLMotor = hardwareMap.dcMotor.get("BLMotor");
        DcMotor BRMotor = hardwareMap.dcMotor.get("BRMotor");
        DcMotor Flywheel = hardwareMap.dcMotor.get("Flywheel");
        DcMotor HorizontalSlidePack = hardwareMap.dcMotor.get("HorizontalSlidePack");
        DcMotor VerticalSlidePack = hardwareMap.dcMotor.get("VerticalSlidePack");
        DcMotor EaterMotor = hardwareMap.dcMotor.get("Eater");

        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        Flywheel.setDirection(DcMotor.Direction.FORWARD);
        HorizontalSlidePack.setDirection(DcMotor.Direction.FORWARD);
        VerticalSlidePack.setDirection(DcMotor.Direction.FORWARD);
        EaterMotor.setDirection(DcMotor.Direction.FORWARD);

        double DriveSpeed = 1;

        waitForStart();
        telemetry.addData("Status","TeleOp");
        telemetry.update();


        while(opModeIsActive()) {

            //Driver 1
            double LeftDrive = DriveSpeed * (gamepad1.left_stick_y);
            double RightDrive = DriveSpeed * (gamepad1.right_stick_y);
            double FlywheelClockwise = gamepad1.left_trigger;
            double FlywheelCounterClockwise = gamepad1.right_trigger;
            double HorizontalSlidePackForward = gamepad2.right_bumper ? 1 : 0;
            double HorizontalSlidePackBackward = gamepad2.right_trigger > 0 ? 1 : gamepad2.right_trigger < 0 ? -1 : 0;
            double VerticalSlidePackForward = gamepad2.dpad_up ? 1 : 0;
            double VerticalSlidePackBackward = gamepad2.dpad_down ? 1 : 0;
            double EaterForward = gamepad2.circle ? 1 : 0;
            double EaterBackward = gamepad2.cross ? 1 : 0;

            if (FlywheelClockwise != 0 || FlywheelCounterClockwise != 0) {
                Flywheel.setPower(FlywheelCounterClockwise + FlywheelClockwise);
            }

            if (HorizontalSlidePackBackward != 0 || HorizontalSlidePackForward != 0) {
                HorizontalSlidePack.setPower(- HorizontalSlidePackBackward + HorizontalSlidePackForward);
            }

            if (VerticalSlidePackBackward != 0 || VerticalSlidePackForward != 0) {
                VerticalSlidePack.setPower(- VerticalSlidePackBackward + VerticalSlidePackForward);
            }

            if (EaterBackward != 0 || EaterForward != 0) {
                EaterMotor.setPower(- EaterBackward + EaterForward);
            }

            FLMotor.setPower(LeftDrive);
            BLMotor.setPower(LeftDrive);
            FRMotor.setPower(RightDrive);
            BRMotor.setPower(RightDrive);

//            double LeftDrive = DriveSpeed * (gamepad1.left_stick_y);
//            double RightDrive = DriveSpeed * (gamepad1.right_stick_y);
//            double LeftStrafe = gamepad1.left_trigger;
//            double RightStrafe = gamepad1.right_trigger;
//            if (LeftStrafe == 0 && RightStrafe == 0) {
//                FLMotor.setPower(LeftDrive);
//                BLMotor.setPower(LeftDrive);
//                FRMotor.setPower(RightDrive);
//                BRMotor.setPower(RightDrive);
//            } else if (LeftStrafe > 0) {
//                FLMotor.setPower(LeftStrafe * 1);
//                BLMotor.setPower(LeftStrafe * -1);
//                FRMotor.setPower(LeftStrafe * -1);
//                BRMotor.setPower(LeftStrafe * 1);
//            } else if (RightStrafe > 0) {
//                FLMotor.setPower(RightStrafe * -1);
//                BLMotor.setPower(RightStrafe * 1);
//                FRMotor.setPower(RightStrafe * 1);
//                BRMotor.setPower(RightStrafe * -1);
//            }
            telemetry.addData("LeftDrive", LeftDrive);
            telemetry.addData("RightDrive", RightDrive);
            telemetry.addData("Flywheel", FlywheelCounterClockwise - FlywheelClockwise);
            telemetry.addData("HorizontalSlidePack", -HorizontalSlidePackBackward + HorizontalSlidePackForward);
            telemetry.addData("VerticalSlidePack", -VerticalSlidePackBackward + VerticalSlidePackForward);
            telemetry.addData("Eater", -EaterBackward + EaterForward);

            telemetry.update();
        }
    }
}
