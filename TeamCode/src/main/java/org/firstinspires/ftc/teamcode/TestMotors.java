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


@Autonomous(name="TestMotors")
public class TestMotors extends LinearOpMode {
    enum DriveDirection {
        FORWARD,
        LEFT,
        RIGHT,
        BACK
    }
    
    enum StartingPositionEnum {
        BLUEHUB,
        BLUEWAREHOUSE,
        REDHUB,
        REDWAREHOUSE
    }
    
    private final StartingPositionEnum STARTING_POSITION = StartingPositionEnum.BLUEHUB;
    
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor Flywheel;
    private final double PPR = 145.1;
    private final double DrivePower = 0.9;
    private final double WHEEL_DIAMETER = 3.77953; // Unit: inch
    private final int TIME_BETWEEN_ACTIONS = 500; // Unit: ms
    private final double GEAR_RATIO = 1.0/3;
    // private DcMotor HorizontalSlidePack;
    // private DcMotor VerticalSlidePack;
    // private DcMotor EaterMotor;
    // private Blinker expansion_Hub_4;
    // private Orientation lastAngles = new Orientation();
    // double globalAngle, power = .30, correction;
    private ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Maps hardware for all parts
        FLMotor = hardwareMap.dcMotor.get("1");
        FRMotor = hardwareMap.dcMotor.get("0");
        BLMotor = hardwareMap.dcMotor.get("2");
        BRMotor = hardwareMap.dcMotor.get("3");
        Flywheel = hardwareMap.dcMotor.get("Fly");
        // HorizontalSlidePack = hardwareMap.dcMotor.get("HorizontalSlidePack");
        // VerticalSlidePack = hardwareMap.dcMotor.get("VerticalSlidePack");
        // EaterMotor = hardwareMap.dcMotor.get("Eater");

        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        Flywheel.setDirection(DcMotor.Direction.FORWARD);
        // HorizontalSlidePack.setDirection(DcMotor.Direction.FORWARD);
        // VerticalSlidePack.setDirection(DcMotor.Direction.FORWARD);
        // EaterMotor.setDirection(DcMotor.Direction.FORWARD);

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

        FLMotor.setTargetPosition(0);
        FRMotor.setTargetPosition(0);
        BLMotor.setTargetPosition(0);
        BRMotor.setTargetPosition(0);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // HorizontalSlidePack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // VerticalSlidePack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // EaterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        FLMotor.setPower(DrivePower);
//        BRMotor.setPower(DrivePower);
//        FRMotor.setPower(DrivePower);
//        BLMotor.setPower(DrivePower);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();
      
        int pulses = (int) distanceToPulses(12);
        setPowerAll(0.9);

        telemetry.addData("Pulses", pulses);
        telemetry.update();

        //The actual program
        FLMotor.setTargetPosition(pulses);
        FRMotor.setTargetPosition(pulses);
        BLMotor.setTargetPosition(pulses);
        BRMotor.setTargetPosition(pulses);

        eTime.reset();
        while (eTime.milliseconds() <= 3000 && opModeIsActive()) {
            telemetry.addData("FLMotorPosition", FLMotor.getCurrentPosition());
            telemetry.addData("FRMotorPosition", FRMotor.getCurrentPosition());
            telemetry.addData("BLMotorPosition", BLMotor.getCurrentPosition());
            telemetry.addData("BRMotorPosition", BRMotor.getCurrentPosition());
            telemetry.update();
        }
//         switch(STARTING_POSITION) {
//             case BLUEHUB:
//                 eTime.reset();
                
//                 drive(DriveDirection.FORWARD, DrivePower, 24); // Drive forward 24 inches
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 strafe(DriveDirection.RIGHT, DrivePower, 24); // Strafe right 24 inches
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 drive(DriveDirection.BACK, DrivePower, 12); // Drive backward 12 inches
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 spinFlywheel(.3, 2000); // Spin flywheel for 2 seconds at a power of .3
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 drive(DriveDirection.FORWARD, DrivePower, 12); // Drive forward 12 inches
//                 break;
//             case BLUEWAREHOUSE:
//                 drive(DriveDirection.FORWARD, DrivePower, 3); // Drive forward 3 inches
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 strafe(DriveDirection.LEFT, DrivePower, 30); // Strafe left 30 inches
//                 sleep(TIME_BETWEEN_ACTIONS);
//                 break;            
//             case REDHUB:
//                 eTime.reset();
                
//                 drive(DriveDirection.FORWARD, DrivePower, 24); // Drive forward 24 inches
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 strafe(DriveDirection.LEFT, DrivePower, 24); // Strafe left 24 inches
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 drive(DriveDirection.BACK, DrivePower, 12); // Drive backward 12 inches
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 spinFlywheel(.3, 2000); // Spin flywheel for 2 seconds at a power of .3
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 drive(DriveDirection.FORWARD, DrivePower, 12); // Drive forward 12 inches
//                 break;
//             case REDWAREHOUSE:
//                 drive(DriveDirection.FORWARD, DrivePower, 3); // Drive forward 3 inches
//                 sleep(TIME_BETWEEN_ACTIONS);

//                 strafe(DriveDirection.RIGHT, DrivePower, 30); // Strafe right 30 inches
//                 sleep(TIME_BETWEEN_ACTIONS);
//                 break;
//         }
    }

    private void spinFlywheel(double power, double time) {
        eTime.reset();
        Flywheel.setPower(power);
        while(opModeIsActive() && eTime.milliseconds() < time){
            telemetry.addData("Time:", eTime);
            telemetry.update();
        }
        Flywheel.setPower(0);
    }

    private void drive(DriveDirection direction, double power, double distance) {
        int pulses = (int) distanceToPulses(distance);
        setPowerAll(power);
        switch(direction) {
            case FORWARD:
                FLMotor.setTargetPosition(pulses);
                FRMotor.setTargetPosition(pulses);
                BLMotor.setTargetPosition(pulses);
                BRMotor.setTargetPosition(pulses);
                break;
            case LEFT:
                FLMotor.setTargetPosition(-1 * pulses);
                FRMotor.setTargetPosition(pulses);
                BLMotor.setTargetPosition(-1 * pulses);
                BRMotor.setTargetPosition(pulses);
                break;
            case RIGHT:
                FLMotor.setTargetPosition(pulses);
                FRMotor.setTargetPosition(-1 * pulses);
                BLMotor.setTargetPosition(pulses);
                BRMotor.setTargetPosition(-1 * pulses);
                break;
            case BACK:
                FLMotor.setTargetPosition(-pulses);
                FRMotor.setTargetPosition(-pulses);
                BLMotor.setTargetPosition(-pulses);
                BRMotor.setTargetPosition(-pulses);
        }
         setPowerAll(0);
    }
    
     private void strafe(DriveDirection direction, double power, double distance){
         int pulses = (int) distanceToPulses(distance);
         setPowerAll(power);
         switch(direction) {
             case LEFT:
                 FLMotor.setTargetPosition(-pulses);
                 BLMotor.setTargetPosition(pulses);
                 FRMotor.setTargetPosition(-pulses);
                 BRMotor.setTargetPosition(pulses);
                 break;
             case RIGHT:
                 FLMotor.setTargetPosition(pulses);
                 FRMotor.setTargetPosition(-pulses);
                 BLMotor.setTargetPosition(pulses);
                 BRMotor.setTargetPosition(-pulses);
                 break;
         }
         setPowerAll(0);
     }
    
    private double distanceToPulses(double distance) {         // Unit: inch
        double rotations = distance / (Math.PI * WHEEL_DIAMETER);
        double pulses = rotations * PPR / GEAR_RATIO;
        return pulses;
    }
    
    private void setPowerAll(double power) {
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
    }
    
//     private void leftturn(double power, double time){
//         eTime.reset();
//         while(opModeIsActive() && eTime.milliseconds() < time){
//             FLMotor.setPower(-1 * power);
//             FRMotor.setPower(1 * power);
//             BLMotor.setPower(-1 * power);
//             BRMotor.setPower(1 * power);
//         }
//         FLMotor.setPower(0);
//         FRMotor.setPower(0);
//         BLMotor.setPower(0);
//         BRMotor.setPower(0);
//     }
    
//     private void rightturn(double power, double time){
//         eTime.reset();
//         while(opModeIsActive() && eTime.milliseconds() < time){
//             FLMotor.setPower(-1 * power);
//             FRMotor.setPower(1 * power);
//             BLMotor.setPower(-1 * power);
//             BRMotor.setPower(1 *power);
//         }
//         FLMotor.setPower(0);
//         FRMotor.setPower(0);
//         BLMotor.setPower(0);
//         BRMotor.setPower(0);
//     }
    
//     private void timedrive(double power, double time){
//         eTime.reset();
//         while(opModeIsActive() && eTime.milliseconds() < time){
//             FLMotor.setPower(power);
//             FRMotor.setPower(power);
//             BLMotor.setPower(power);
//             BRMotor.setPower(power);
//             telemetry.addData("Time:", eTime);
//             telemetry.update();
//         }
//         FLMotor.setPower(0);
//         FRMotor.setPower(0);
//         BLMotor.setPower(0);
//         BRMotor.setPower(0);
//     }

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
