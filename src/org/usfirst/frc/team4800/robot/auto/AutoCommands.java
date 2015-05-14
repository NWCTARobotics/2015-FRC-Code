package org.usfirst.frc.team4800.robot.auto;

import org.usfirst.frc.team4800.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoCommands {

    private Robot robot;
    private Autonomous auto;

    private boolean liftDone = true;

    private int dashboardTimer = 0;
    private int elevatorTarget = 0;

    public AutoCommands(Robot robot, Autonomous auto) {
        this.robot = robot;
        this.auto = auto;
    }

    public double clamp(double value) {
        if (value > 1.0)
            value = 1.0;
        else if (value < -1.0)
            value = -1.0;
        return value;
    }

    private boolean update() {
        dashboardTimer++;
        if (dashboardTimer > 30) {
            SmartDashboard.putNumber("Ultrasonic sensor", robot.getSonar().getRangeInches());
            dashboardTimer = 0;
        }

        double cur_lift = robot.getElevatorEndoder().getDistance();
        double lift_error = elevatorTarget - cur_lift;
        liftDone = false;

        if ((lift_error > 20.0) && (auto.fault() == false)) {
            double lift_cmd = clamp(0.01 * lift_error);
            robot.setLiftMotors(lift_cmd); // up
        } else if ((lift_error < -20.0) && (auto.fault() == false)) {
            double lift_cmd = clamp(0.01 * lift_error);
            robot.setLiftMotors(lift_cmd);
        } else {
            robot.setLiftMotors(0.0);
            liftDone = true;
        }

        if (robot.isAutonomous() && robot.isEnabled() && !auto.fault()) {
            return false; // keep going!
        } else {
            return true;
        }
    }

    public void driveTimed(double forwardSpeed, double sideSpeed, double timeout, boolean useSonar) {
        boolean done = update();
        auto.getTimer().reset();
        double angle = robot.getGyro().getAngle();

        while (!done) {
            double angleError = angle - robot.getGyro().getAngle();
            double angleCmd = clamp(0.025 * angleError);
            double distanceError = auto.getInitialSonar() - robot.getSonar().getRangeInches();
            double distanceCmd = clamp(0.05 * distanceError);

            if (robot.getSonar().getRangeInches() == 0)
                distanceCmd = 0;

            if (useSonar)
                robot.getRobotDrive().mecanumDrive_Cartesian(distanceCmd, -forwardSpeed, angleCmd, robot.getGyro().getAngle());
            else
                robot.getRobotDrive().mecanumDrive_Cartesian(sideSpeed, -forwardSpeed, angleCmd, robot.getGyro().getAngle());

            done = done || update();
            done = done || (auto.getTimer().get() > timeout);
        }
    }

    public void driveUntilTote(double speed, double angle, double timeout) {
        boolean done = update();
        auto.getTimer().reset();

        while (!done) {
            double angleError = angle - robot.getGyro().getAngle();
            double angleCmd = clamp(0.025 * angleError);
            double distanceError = auto.getInitialSonar() - robot.getSonar().getRangeInches();
            double distanceCmd = clamp(0.05 * distanceError);

            if (robot.getSonar().getRangeInches() == 0)
                distanceCmd = 0;

            robot.getRobotDrive().mecanumDrive_Cartesian(distanceCmd, -speed, angleCmd, robot.getGyro().getAngle());

            if (auto.getTimer().get() > timeout) {
                done = true;
                auto.setFault(true);
            }

            if (robot.rightSideTote() || robot.leftSideTote()) {
                done = true;
            }

            done = done || update();
        }
    }

    public void driveRotation(double maxSpeed, double angleTarget, double timeout) {
        boolean done = update();
        auto.getTimer().reset();

        while (!done) {
            double currentAngle = robot.getGyro().getAngle();
            double angleError = angleTarget - currentAngle;
            double command = clamp(0.2 * angleError);

            if (angleError > 1) {
                robot.getRobotDrive().mecanumDrive_Cartesian(0.0, 0.0, maxSpeed * command, robot.getGyro().getAngle());
            } else if (angleError < -1) {
                robot.getRobotDrive().mecanumDrive_Cartesian(0.0, 0.0, maxSpeed * command, robot.getGyro().getAngle());
            } else {
                done = true;
            }

            done = done || update();
            done = done || (auto.getTimer().get() > timeout);
        }
    }

    public void setClaw(boolean deployed) {
        if (deployed) {
            robot.getClaw().set(Value.kForward);
        } else {
            robot.getClaw().set(Value.kReverse);
        }
    }

    public void wait(double seconds) {
        auto.getTimer().reset();
        boolean done = update();

        while (!done) {
            done = done || update();
            done = done || (auto.getTimer().get() >= seconds);
        }
    }

    public void waitForLift(double additionalTime) {
        auto.getTimer().reset();
        boolean done = update();

        while (!done) {
            done = done || update();
            done = done || liftDone;
        }

        if (additionalTime > 0) {
            wait(additionalTime);
        }
    }

    public void setLiftTarget(int target) {
        this.elevatorTarget = target;
    }

    public void setLiftOneTote() {
        elevatorTarget = 852;
    }

    public void stop() {
        robot.getRobotDrive().mecanumDrive_Cartesian(0.0, 0.0, 0.0, robot.getGyro().getAngle());
    }
}
