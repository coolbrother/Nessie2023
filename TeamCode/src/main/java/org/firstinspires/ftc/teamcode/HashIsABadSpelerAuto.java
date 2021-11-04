package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.ColorSensor;


@Autonomous(name="HashIsABadSpelerAuto")
public class HashIsABadSpelerAuto extends LinearOpMode {
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor Flywheel;
    // private DcMotor HorizontalSlidePack;
    // private DcMotor VerticalSlidePack;
    // private DcMotor EaterMotor;
    // private Blinker expansion_Hub_4;
    // private Orientation lastAngles = new Orientation();
    // double globalAngle, power = .30, correction;
    private ElapsedTime eTime = new ElapsedTime();
    
    enum DriveDirection {
        FORWARD,
        LEFT,
        RIGHT
    }
    
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Maps hardware for all parts
        FLMotor = hardwareMap.dcMotor.get("0");
        FRMotor = hardwareMap.dcMotor.get("1");
        BLMotor = hardwareMap.dcMotor.get("2");
        BRMotor = hardwareMap.dcMotor.get("3");
        Flywheel = hardwareMap.dcMotor.get("Fly");
        // HorizontalSlidePack = hardwareMap.dcMotor.get("HorizontalSlidePack");
        // VerticalSlidePack = hardwareMap.dcMotor.get("VerticalSlidePack");
        // EaterMotor = hardwareMap.dcMotor.get("Eater");

        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        Flywheel.setDirection(DcMotor.Direction.FORWARD);
        // HorizontalSlidePack.setDirection(DcMotor.Direction.FORWARD);
        // VerticalSlidePack.setDirection(DcMotor.Direction.FORWARD);
        // EaterMotor.setDirection(DcMotor.Direction.FORWARD);

        double DrivePower = 0.75;

        waitForStart();
        telemetry.addData("Status","TeleOp");
        telemetry.update();

        //Configures settings for different parts
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // HorizontalSlidePack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // VerticalSlidePack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // EaterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // HorizontalSlidePack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // VerticalSlidePack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // EaterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(50);
        //The actual program
        eTime.reset();
        drive(DriveDirection.FORWARD, DrivePower, 100);
        sleep(500); // For Testing Purposes
        drive(DriveDirection.LEFT, DrivePower, 100);
        sleep(500);
        drive(DriveDirection.FORWARD, DrivePower, 500);
    }
    
    private void drive(DriveDirection direction, double power, double time) {
        eTime.reset();
        switch(direction) {
            case FORWARD:
                FLMotor.setPower(power);
                FRMotor.setPower(power);
                BLMotor.setPower(power);
                BRMotor.setPower(power);
                break;
            case LEFT:
                FLMotor.setPower(-1 * power);
                FRMotor.setPower(power);
                BLMotor.setPower(-1 * power);
                BRMotor.setPower(power);
                break;
            case RIGHT:
                FLMotor.setPower(power);
                FRMotor.setPower(-1 * power);
                BLMotor.setPower(power);
                BRMotor.setPower(-1 * power);
                break;
        } 
        while(opModeIsActive() && eTime.milliseconds() < time){
            telemetry.addData("Time:", eTime);
            telemetry.update();
        }
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
    }
    private void timedrive(double power, double time){
        eTime.reset();
        while(opModeIsActive() && eTime.milliseconds() < time){
            FLMotor.setPower(power);
            FRMotor.setPower(power);
            BLMotor.setPower(power);
            BRMotor.setPower(power);
            telemetry.addData("Time:", eTime);
            telemetry.update();
        }
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
    }
    // private void strafe(double power, double time){
    //     eTime.reset();
    //     while(opModeIsActive() && eTime.seconds() < time){
    //         FLMotor.setPower(-power);
    //         FRMotor.setPower(-power);
    //         BLMotor.setPower(-power);
    //         BRMotor.setPower(-power);
    //         telemetry.addData("Time:", eTime);
    //         telemetry.update();
    //     }
    //     FLMotor.setPower(0);
    //     FRMotor.setPower(0);
    //     BLMotor.setPower(0);
    //     BRMotor.setPower(0);
    // }
    private void leftturn(double power, double time){
        eTime.reset();
        while(opModeIsActive() && eTime.milliseconds() < time){
            FLMotor.setPower(-1 * power);
            FRMotor.setPower(1 * power);
            BLMotor.setPower(-1 * power);
            BRMotor.setPower(1 * power);
        }
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
    }
    private void rightturn(double power, double time){
        eTime.reset();
        while(opModeIsActive() && eTime.milliseconds() < time){
            FLMotor.setPower(-1 * power);
            FRMotor.setPower(1 * power);
            BLMotor.setPower(-1 * power);
            BRMotor.setPower(1 *power);
        }
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
    }




    /*(gamepad2.a){
            LGServo.setPosition(0);
            RGServo.setPosition(0.75);
        }else if (gamepad2.b){
            LGServo.setPosition(0.75);
            RGServo.setPosition(0);
        }
    */
//    private void resetAngle()
//    {
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        globalAngle = 0;
//    }
//
//    //Get Angle Function
//    private double getAngle()
//    {
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }

    //Check Direction Function
//    private double checkDirection()
//    {
//        double correction, angle, gain = .10;
//
//        angle = getAngle();
//
//        if (angle == 0)
//            correction = 0;
//        else
//            correction = -angle;
//
//        correction = correction * gain;
//
//        return correction;
//    }
//
//    //Rotate Function
//    private void rotate(double power, int degrees)
//    {
//        if(opModeIsActive())
//        {
//            double currentAng = 0;
//            resetAngle();
//            if (power > 0)
//            {
//                while(opModeIsActive() && (-currentAng) < degrees)
//                {
//                    currentAng  =   getAngle();
//                    FLMotor.setPower(power);
//                    FRMotor.setPower(-power);
//                    BLMotor.setPower(power);
//                    BRMotor.setPower(-power);
//
//                    telemetry.addData("Angle: ", currentAng);
//                    telemetry.addData("Target: ", degrees);
//                }
//            }
//
//            else
//            {
//                degrees = -degrees;
//                while(opModeIsActive() && (currentAng) < degrees)
//                {
//                    currentAng  =   getAngle();
//                    FLMotor.setPower(power);
//                    FRMotor.setPower(-power);
//                    BLMotor.setPower(power);
//                    BRMotor.setPower(-power);
//
//                    telemetry.addData("Angle: ", currentAng);
//                    telemetry.addData("Target: ", degrees);
//                }
//            }
//
//            FLMotor.setPower(0);
//            FRMotor.setPower(0);
//            BLMotor.setPower(0);
//            BRMotor.setPower(0);
//
//        }
//    }

    //Encoder Drive Function
    // public void drive(double power, int centimeters)
    // {

    //     int currentPosFR    = 0;
    //     int currentPosFL    = 0;
    //     int currentPosBR    = 0;
    //     int currentPosBL    = 0;
    //     int target = centimeters * 45;

    //     // Ensure that the opmode is still active
    //     if (opModeIsActive())
    //     {
    //         FRMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
    //         FLMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
    //         BLMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
    //         BRMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

    //         // Turn On RUN_TO_POSITION
    //         FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //         BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //         FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //         BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    //         // Determine new target position, and pass to motor controller
    //         BRMotor.setTargetPosition(target);
    //         FLMotor.setTargetPosition(target);
    //         BLMotor.setTargetPosition(target);
    //         FRMotor.setTargetPosition(target);

    //         FLMotor.setPower(power);
    //         BRMotor.setPower(power);
    //         FRMotor.setPower(power);
    //         BLMotor.setPower(power);

    //         // keep looping while we are still active, and there is time left, and both motors are running.
    //         // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
    //         // its target position, the motion will stop.  This is "safer" in the event that the robot will
    //         // always end the motion as soon as possible.
    //         // However, if you require that BOTH motors have finished their moves before the robot continues
    //         // onto the next step, use (isBusy() || isBusy()) in the loop test.
    //         if (power > 0)
    //         {
    //             while (opModeIsActive() && (currentPosFL < target || currentPosBR < target ||
    //             currentPosFR < target || currentPosBL < target))
    //             {
    //                 currentPosBR    = BRMotor.getCurrentPosition();
    //                 currentPosFL    = FLMotor.getCurrentPosition();
    //                 currentPosFR    = FRMotor.getCurrentPosition();
    //                 currentPosBL    = BLMotor.getCurrentPosition();

    //                 // Display it for the driver.
    //                 telemetry.addData("Running to ", target);
    //                 telemetry.addData("Running at front left: ",  currentPosFL);
    //                 telemetry.addData("Running at back right: ", currentPosBR);
    //                 telemetry.addData("Running at front right: ",  currentPosFR);
    //                 telemetry.addData("Running at back left: ", currentPosBL);
    //                 telemetry.update();
    //             }
    //         }

    //         else
    //         {
    //             while (opModeIsActive() && (currentPosFL > target || currentPosBR > target ||
    //             currentPosFR > target || currentPosBL > target))
    //             {
    //                 currentPosBR    = BRMotor.getCurrentPosition();
    //                 currentPosFL    = FLMotor.getCurrentPosition();
    //                 currentPosFR    = FRMotor.getCurrentPosition();
    //                 currentPosBL    = BLMotor.getCurrentPosition();

    //                 // Display it for the driver.
    //                 telemetry.addData("Running to ", target);
    //                 telemetry.addData("Running at front left: ",  currentPosFL);
    //                 telemetry.addData("Running at back right: ", currentPosBR);
    //                 telemetry.addData("Running at front right: ",  currentPosFR);
    //                 telemetry.addData("Running at back left: ", currentPosBL);
    //                 telemetry.update();
    //             }
    //         }
    //     }

//        // Stop all motion;
//        BRMotor.setPower(0);
//        FRMotor.setPower(0);
//        FLMotor.setPower(0);
//        BLMotor.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //  sleep(250);   // optional pause after each move


//Land Function
}

