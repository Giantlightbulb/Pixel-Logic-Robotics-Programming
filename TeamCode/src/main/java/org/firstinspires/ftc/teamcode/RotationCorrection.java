package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class RotationCorrection extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();
    public void runOpMode() {


        /*
        Driver:
        -Drivetrain
        -Mascot
        -Latch
        -Vaccuum

        Killer:
        -extension
        -arm
        -vertical lift
         */
        //Retrieves the mappings from runtime
        robot.init(this, hardwareMap, telemetry);

        robot.frontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Heading
        int heading;
        double angle;
        double rotation;


        //Telemetry initialized message
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        //Gyro calibrating
        robot.modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(robot.timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        //Finalized gyro calibration.
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
        //double servoPosition = 0.0;
        //Wait until op-mode start
        waitForStart();
        //Resets timer
        robot.timer.reset();
        while (opModeIsActive()) {
            double zAngle = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle/180.0;
            if (!(Math.abs(zAngle - 0.005) < 0.0055)) {
                rotation = 0.3 * Math.cbrt(zAngle);
            } else {
                rotation = 0;
            }
            telemetry.addLine()
                    .addData("Heading:", zAngle)
                    .addData("Rotation:", rotation);
            telemetry.update();
            robot.frontDrive.setPower(rotation);
            robot.backDrive.setPower(-rotation);
            robot.leftDrive.setPower(rotation);
            robot.rightDrive.setPower(-rotation);
        }
    }
}
