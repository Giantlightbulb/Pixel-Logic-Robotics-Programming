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
        //Handles the current autonomous with drive by time
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
        waitForStart();
        robot.timer.reset();
//----------------------------------------End Initialization
        robot.encoderRun(robot.frontDrive, robot.backDrive,0.5 , 400, 2);

    }
    public void encoderRun(DcMotor firstMotor, DcMotor secondMotor, double power, int distance, double timeOut) {
        firstMotor.setTargetPosition(distance);
        secondMotor.setTargetPosition(distance);
        firstMotor.setPower(power);
        secondMotor.setPower(power);
        robot.timer.reset();
        while ((firstMotor.getTargetPosition() > firstMotor.getCurrentPosition()) &&
                (secondMotor.getTargetPosition() > secondMotor.getCurrentPosition()) &&
                (robot.timer.seconds() < timeOut) &&
                (opModeIsActive())) {

         //   telemetry.addData("Running", "%2.5f distance left (firstMotor) %2.5f time left", (firstMotor.getTargetPosition()) - firstMotor.getCurrentPosition()), (timeOut - robot.timer.seconds()));
            telemetry.update();

        }
    }
}


