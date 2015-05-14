package org.usfirst.frc.team4800.robot;

import org.usfirst.frc.team4800.robot.AutoButton.DetectionType;
import org.usfirst.frc.team4800.robot.auto.Autonomous;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {

    private Joystick left;
    private Joystick right;
    private Joystick eControl;

    private Victor lift1;
    private Victor lift2;

    private RobotDrive drive;
    private Gyro gyro;
    private Ultrasonic sonar;

    private AnalogInput leftButton;
    private AnalogInput rightButton;

    private Encoder liftE;

    private DoubleSolenoid claw;
    private DoubleSolenoid intake;

    private SendableChooser auto;

    boolean fieldDrive = false;
    boolean trash = false;
    boolean intakeDep = false;

    public Robot() {
        send("Initializing gyro");
        gyro = new Gyro(0);
        gyro.initGyro();
        send("Gyro done");

        drive = new RobotDrive(0, 2, 3, 1);
        drive.setInvertedMotor(MotorType.kFrontRight, true);
        drive.setInvertedMotor(MotorType.kRearRight, true);

        left = new Joystick(0);
        right = new Joystick(1);
        eControl = new Joystick(2);

        lift1 = new Victor(4);
        lift2 = new Victor(5);
        claw = new DoubleSolenoid(0, 1);
        intake = new DoubleSolenoid(2, 3);

        leftButton = new AnalogInput(2);
        rightButton = new AnalogInput(3);

        liftE = new Encoder(8, 9);
        resetEncoders();

        auto = new SendableChooser();
        auto.addDefault("TRIPLE STACK MY GOATS", 1);
        auto.addObject("Auto Zone Drive", 4);
        auto.addObject("Demo of Sonar", 5);
        auto.addObject("Demo of Replay", 2);
        SmartDashboard.putData("Autonomous Chooser", auto);

        sonar = new Ultrasonic(1, 0);
        sonar.setEnabled(true);
        sonar.setAutomaticMode(true);
    }

    public void autonomous() {
        drive.setSafetyEnabled(false);
        lift1.setSafetyEnabled(false);
        lift2.setSafetyEnabled(false);
        resetEncoders();
        new Autonomous(this).runAutonomous();
    }

    public void operatorControl() {
        send("OP starting");
        drive.setSafetyEnabled(true);
        AutoButton oneTote = new AutoButton(eControl, 2, DetectionType.HOLD);
        AutoButton twoTote = new AutoButton(eControl, 3, DetectionType.HOLD);
        AutoButton threeTote = new AutoButton(eControl, 4, DetectionType.HOLD);
        AutoButton fourTote = new AutoButton(eControl, 5, DetectionType.HOLD);
        AutoButton trashCan = new AutoButton(eControl, 1, DetectionType.ON_PRESS);
        AutoButton fieldButton = new AutoButton(right, 3, DetectionType.ON_PRESS);
        AutoButton resetEncoders = new AutoButton(eControl, 8, DetectionType.ON_PRESS);
        AutoButton intakeButton = new AutoButton(left, 3, DetectionType.ON_PRESS);
        while (isEnabled() && isOperatorControl()) {
            double rightX = right.getX();
            double rightY = right.getY();
            double leftX = left.getX();
            if (rightX < 0.05 && rightX > -0.05) {
                rightX = 0;
            }
            if (rightY < 0.05 && rightY > -0.05) {
                rightY = 0;
            }
            if (leftX < 0.05 && leftX > -0.05) {
                leftX = 0;
            }
            if (fieldDrive) {
                drive.mecanumDrive_Cartesian(rightX, rightY * 0.75, leftX, gyro.getAngle());
            } else {
                drive.mecanumDrive_Cartesian(rightX, rightY * 0.75, leftX, 0);
            }
            if (oneTote.isPressed()) {
                if (liftE.get() < 900) {
                    setLiftMotors(1.0);
                }

                else if (liftE.get() > 1100) {
                    setLiftMotors(-0.8);
                } else {
                    setLiftMotors(0);
                }
            }

            else if (twoTote.isPressed()) {
                if (liftE.get() < 1400) {
                    setLiftMotors(1.0);
                }

                else if (liftE.get() > 1600) {
                    setLiftMotors(-0.8);
                } else {
                    setLiftMotors(0);
                }
            }

            else if (threeTote.isPressed()) {
                if (liftE.get() > -2000) {
                    setLiftMotors(1.0);
                }

                else if (liftE.get() < -2300) {
                    setLiftMotors(-0.8);
                } else {
                    setLiftMotors(0);
                }
            }

            else if (fourTote.isPressed()) {
                if (liftE.get() > -2591) {
                    setLiftMotors(1.0);
                }

                else if (liftE.get() < -2750) {
                    setLiftMotors(-0.8);
                } else {
                    setLiftMotors(0);
                }
            } else {
                setLiftMotors(eControl.getY());
            }

            if (fieldButton.isPressed()) {
                if (fieldDrive) {
                    fieldDrive = false;
                } else {
                    fieldDrive = true;
                }
            }

            if (trashCan.isPressed()) {
                toggleTrash();
            }

            if (intakeButton.isPressed()) {
                toggleIntake();
            }

            if (resetEncoders.isPressed()) {
                resetEncoders();
            }
            updateDash();
            Timer.delay(0.005);
        }
    }

    private void updateDash() {
        SmartDashboard.putBoolean("Left Side", leftSideTote());
        SmartDashboard.putBoolean("Right Side", rightSideTote());
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
        SmartDashboard.putNumber("Lift Distance", liftE.getDistance());
        SmartDashboard.putNumber("Lift Rate", liftE.getRate());
        SmartDashboard.putBoolean("Field Drive", fieldDrive);
        SmartDashboard.putNumber("Ultrasonic sensor", sonar.getRangeInches());
    }

    public void setLiftMotors(double speed) {
        lift1.set(speed);
        lift2.set(speed);
    }

    public void resetEncoders() {
        liftE.reset();
    }

    public boolean isToteIn() {
        if (leftButton.getVoltage() < 0.1 && rightButton.getVoltage() < 0.1) {
            return true;
        } else {
            return false;
        }
    }

    public boolean leftSideTote() {
        if (leftButton.getVoltage() < 0.1) {
            return true;
        } else {
            return false;
        }
    }

    public boolean rightSideTote() {
        if (rightButton.getVoltage() < 0.1) {
            return true;
        } else {
            return false;
        }
    }

    public void toggleTrash() {
        if (trash) {
            trash = false;
            claw.set(Value.kForward);
        } else {
            trash = true;
            claw.set(Value.kReverse);
        }
    }

    public void toggleIntake() {
        if (intakeDep) {
            intakeDep = false;
            intake.set(Value.kReverse);
        } else {
            intakeDep = true;
            intake.set(Value.kForward);
        }
    }

    public void send(String message) {
        SmartDashboard.putString("last_Message", message);
    }

    public RobotDrive getRobotDrive() {
        return drive;
    }

    public SendableChooser getSendable() {
        return auto;
    }

    public Ultrasonic getSonar() {
        return sonar;
    }

    public Encoder getElevatorEndoder() {
        return liftE;
    }

    public Gyro getGyro() {
        return gyro;
    }

    public DoubleSolenoid getClaw() {
        return claw;
    }
}
