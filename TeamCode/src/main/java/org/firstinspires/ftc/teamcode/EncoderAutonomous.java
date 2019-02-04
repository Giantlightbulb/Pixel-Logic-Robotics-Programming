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
@Autonomous(name="EncoderAutonomous", group="Autonomous")

public class EncoderAutonomous extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
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

        //Initiate Servos
        robot.mascot.setPosition(0.8);//mascot up

        // Start Button
        waitForStart();
        robot.timer.reset();

        robot.frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//----------------------------------------End Initialization---------------------------------
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //===================================
        //===================================

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        telemetry.addLine()
                .addData("Encoder Drive", "");
        telemetry.update();
        DriveForwardDistance(robot.leftDrive, robot.rightDrive, 0.35, 4000,10.0); // 44 in
        sleep(3000);

        runDriveTrain(robot.leftDrive, robot.rightDrive, 0.35, 3, "Drive by Time"); // 28 in

        //===================================
        //===================================

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void encoderDrive(DcMotor motor, double speed, double distance, double timeout) {

    }

    public void encoderDrive(DcMotor motor1, DcMotor motor2,
                             double speed,
                             double distance1, double distance2,
                             double timeout) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor1.getCurrentPosition() + (int) (distance1 * COUNTS_PER_INCH);
            newRightTarget = motor2.getCurrentPosition() + (int) (distance2 * COUNTS_PER_INCH);
            motor1.setTargetPosition(newLeftTarget);
            motor2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.timer.reset();
            motor1.setPower(Math.abs(speed));
            motor2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.timer.seconds() < timeout) &&
                    (motor1.isBusy() && motor2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor1.setPower(0);
            motor2.setPower(0);

            // Turn off RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void DriveForwardDistance(DcMotor firstMotor, DcMotor secondMotor, double power, int distance, double timeOut){

        // Reset encoders
        firstMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Target Position b
        firstMotor.setTargetPosition(distance);
        secondMotor.setTargetPosition(distance);

        // Set to RUN_TO_POSITION mode
        firstMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        secondMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Drive Power
        firstMotor.setPower(power);
        secondMotor.setPower(power);
        robot.timer.reset();

        while((opModeIsActive())&&(firstMotor.isBusy() || secondMotor.isBusy())){
            // wait
            telemetry.addData("Path1",  "Going to %7d ,  currently at %7d and %7d.", firstMotor.getTargetPosition(),  firstMotor.getCurrentPosition(), secondMotor.getCurrentPosition());
            telemetry.update();
        }

        firstMotor.setPower(0);
        secondMotor.setPower(0);

        firstMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runDriveTrain(DcMotor motor1, DcMotor motor2, double power, double time, String label) {
        motor1.setPower(power);
        motor2.setPower(power);
        robot.timer.reset();
        while (opModeIsActive() && (robot.timer.seconds() < time)) {
            telemetry.addData(label, "%2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        motor1.setPower(0);
        motor2.setPower(0);
        robot.timer.reset();
    }
}


