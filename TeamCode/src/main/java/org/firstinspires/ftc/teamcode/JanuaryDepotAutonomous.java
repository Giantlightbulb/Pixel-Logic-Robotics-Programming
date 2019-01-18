// AUTONOMOUS DEPOT SIDE

//TeleOp and Hardware
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.graphics.Color;
@Autonomous(name="JanuaryDepotAutonomous", group="Autonomous")

public class JanuaryDepotAutonomous extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        //Approximate Speed at full power: 36 inches / 1.75 seconds
        // Left motion motor 2 positive, motor 4 negative
        // Forward motion motor 1 positive motor 3 negative
        //Gyro calibating
        robot.modernRoboticsI2cGyro.calibrate();

        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(robot.timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        //Finalized gyro calibration.
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        robot.latch.setPosition(1.0); // latched
        robot.mascot.setPosition(1.0);

        waitForStart();
        robot.timer.reset();

        robot.frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//----------------------------------------End Initialization---------------------------------
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //LOWER DOWN (not written) ===========================================================

        robot.latch.setPosition(0.0); // unlatched


        /*

        //FIND SAMPLING SPOT =================================================================
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //encoderDrive(robot.frontDrive, robot.backDrive, 0.05, 1000, 1000 ,10.0);//runs left (hopefully)
        //encoderRun(robot.frontDrive, robot.backDrive,0.15 , 500, 20.0);

        encoderRun(robot.leftDrive, robot.rightDrive, 0.5,500,8 ) // 19 inches
        encoderRun(robot.frontDrive, robot.backDrive, -0.5,-500,5 ) // 14 inches
        //BEGIN SAMPLING =====================================================================
        robot.frontDrive.setPower(-0.25);
        robot.backDrive.setPower(0.25);
        robot.timer.reset();

        double timeToFind = 0;
        int alpha = 0;
        int red = 0;
        int green = 0;
        int blue = 0;
        while (opModeIsActive() &&
                (robot.timer.seconds() < 3) &&
                !((Math.abs(alpha - 70) < 75 ) &&
                        (Math.abs(red - 255) < 75 ) &&
                        (Math.abs(green - 107) < 75 ) &&
                        (Math.abs(blue) < 30 ))) {
            robot.colors = robot.colorSensor.getNormalizedColors();
            float max = Math.max(Math.max(Math.max(robot.colors.red, robot.colors.green), robot.colors.blue), robot.colors.alpha);
            robot.colors.red   /= max;
            robot.colors.green /= max;
            robot.colors.blue  /= max;
            int color = robot.colors.toColor();
            alpha = Color.alpha(color);
            red = Color.red(color);
            green = Color.green(color);
            blue = Color.blue(color);
            telemetry.addData("Sampling", "Sampling: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.addData("a", alpha);
            telemetry.addData("r", red);
            telemetry.addData("g", green);
            telemetry.addData("b", blue);
            //Amount of time taken to find the sample
            timeToFind = robot.timer.seconds();
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();
        while (opModeIsActive() && (robot.timer.seconds() < 0.2)){

        }

        //Retrieves mineral only if the mineral was correctly sampled
        if ((Math.abs(alpha - 70) < 70 ) &&
                (Math.abs(red - 255) < 70 ) &&
                (Math.abs(green - 107) < 70 ) &&
                (Math.abs(blue) < 20 )) {
            robot.motor2.setPower(0);
            robot.motor4.setPower(0);
            encoderRun(robot.leftDrive, robot.rightDrive, 0.5,500,8 ) // 5 inches
            encoderRun(robot.leftDrive, robot.rightDrive, -0.5,-500,8 ) // 5 inches
          }
          robot.timer.reset();
        //Reversed motion
        robot.frontMotor.setPower(0.3);
        robot.backMotor.setPower(-0.3);
        //Moves for the same length of time
        while (opModeIsActive() && (robot.timer.seconds() < (timeToFind + 0.4))){
            telemetry.addData("Returning to initial position (" + timeToFind + " sec)", "Returning: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.frontMotor.setPower(0);
        robot.backMotor.setPower(0);
        robot.timer.reset();

        */


        robot.frontDrive.setPower(0);
        robot.backDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        telemetry.addLine()
                .addData("Parked:", "Completed");

    }
    public void encoderRun(DcMotor firstMotor, DcMotor secondMotor, double power, int distance, double timeOut) {

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int firstTarget = firstMotor.getCurrentPosition() + distance;
        int secondTarget = secondMotor.getCurrentPosition() - distance;


        firstMotor.setTargetPosition(firstTarget);
        secondMotor.setTargetPosition(secondTarget);

        firstMotor.setPower(power);
        secondMotor.setPower(-power);
        robot.timer.reset();

        if (distance > 0) {
            while (((firstMotor.getTargetPosition() >= firstMotor.getCurrentPosition()) ||
                    (secondMotor.getTargetPosition() <= secondMotor.getCurrentPosition())) && (opModeIsActive()) &&
                    (robot.timer.seconds() < timeOut)) {

//            telemetry.addData("Running", "%2.5f distance left (firstMotor) %2.5f time left", (firstMotor.getTargetPosition() - firstMotor.getCurrentPosition()), (timeOut - robot.timer.seconds()));
                telemetry.addData("Path1",  "Going to %7d ,  currently at %7d and %7d.", firstMotor.getTargetPosition(),  firstMotor.getCurrentPosition(), secondMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        else{
            while (((firstMotor.getTargetPosition() <= firstMotor.getCurrentPosition()) ||
                    (secondMotor.getTargetPosition() >= secondMotor.getCurrentPosition())) && (opModeIsActive()) &&
                    (robot.timer.seconds() < timeOut)) {
                telemetry.update();
            }
        }

    }
    public void encoderDrive(DcMotor firstMotor, DcMotor secondMotor,
                             double speed,
                             int first, int second,
                             double timeoutS) {
        int newFirstTarget;
        int newSecondTarget;

        // Ensure that the op-mode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFirstTarget = firstMotor.getCurrentPosition() - (first);
            newSecondTarget = secondMotor.getCurrentPosition() + (second);

            firstMotor.setTargetPosition(newFirstTarget);
            secondMotor.setTargetPosition(newSecondTarget);

            // Turn On RUN_TO_POSITION
            firstMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            secondMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.timer.reset();
            firstMotor.setPower(1);
            secondMotor.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.timer.seconds() < timeoutS) &&
                    (firstMotor.isBusy() || secondMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFirstTarget,  newSecondTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        firstMotor.getCurrentPosition(),
                        secondMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            firstMotor.setPower(0);
            secondMotor.setPower(0);

              sleep(250);   // optional pause after each move
        }
    }
}


