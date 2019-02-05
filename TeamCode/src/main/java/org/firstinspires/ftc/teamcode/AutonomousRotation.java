package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutonomousRotation", group="Autonomous")

public class AutonomousRotation extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();
    public void runOpMode() {
        robot.init(this, hardwareMap, telemetry);
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        // Left motion motor 2 positive, motor 4 positive
        // Forward motion motor 1 positive motor 3 positive
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

        //Finished initializing
        waitForStart();

        robot.timer.reset();

        robot.frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Rotates 90 degrees left
        robot.rotateTheta(5, 90, "Left Rotation");
        sleep(3000);

        //Rotates 90 degrees right

        sleep(3000);

        //Rotates 45 degrees right

        sleep(3000);

        //Rotates 45 degrees left

        telemetry.addLine()
                .addData("Test Finished", "Please reset.");
    }
}
