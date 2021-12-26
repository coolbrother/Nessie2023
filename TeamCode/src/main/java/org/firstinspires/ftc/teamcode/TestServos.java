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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TestServos")
//@Disabled
public class TestServos extends LinearOpMode {

    private CRServo GrabberL;
//    private CRServo GrabberR;
    private final double DriveSpeed = 0.9;

    @Override
    public void runOpMode () {
        // Get motors, the expansion hub doesn't always properly connect which is why
        // the flywheel is commented out
        GrabberL = hardwareMap.crservo.get("GL");
//        GrabberR = hardwareMap.crservo.get("GR");

        GrabberL.setDirection(CRServo.Direction.FORWARD);
//        GrabberR.setDirection(CRServo.Direction.REVERSE);

        // wait for the coach to press start
        waitForStart();
        telemetry.addData("Status","TeleOp");
        telemetry.update();


        while(opModeIsActive()) {

            boolean GrabberIn = gamepad2.a;
            boolean GrabberOut = gamepad2.b;

            if (GrabberIn || GrabberOut) {
                GrabberL.getController().setServoPosition(GrabberL.getPortNumber(), GrabberIn ? 0.3 : 0.6);
//                GrabberR.getController().setServoPosition(GrabberR.getPortNumber(), GrabberIn ? 0.8 : 0.5);
            }
       telemetry.addData("GrabberIn", GrabberIn);
            telemetry.addData("GrabberOut", GrabberOut);
            telemetry.addData("GrabberLPosition", GrabberL.getController().getServoPosition(GrabberL.getPortNumber()));
//            telemetry.addData("GrabberRPosition", GrabberR.getController().getServoPosition(GrabberR.getPortNumber()));
            telemetry.update();
        }
    }
}