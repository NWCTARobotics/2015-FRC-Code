package org.usfirst.frc.team4800.robot.auto;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

import org.usfirst.frc.team4800.robot.ControllerInformation;
import org.usfirst.frc.team4800.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class Autonomous {

    private Robot robot;
    private AutoCommands commands;
    private Timer time;

    private double initialSonar = 34.45;

    private boolean fault = false;

    public Autonomous(Robot robot) {
        this.robot = robot;
        commands = new AutoCommands(robot, this);
        time = new Timer();
        time.start();
    }

    public void runAutonomous() {
        int autoType = 1;
        try {
            autoType = (int) robot.getSendable().getSelected();
        } catch (Exception e) {
            send("Exception in auto: " + e.getMessage());
        }

        if (autoType == 2) {
            ArrayList<ControllerInformation> list = loadFile("auto-2");
            double u = list.size() / 15;
            double delay = 1 / u;
            send("delay: " + delay + ", list size: " + list.size());
            for (ControllerInformation i : list) {
                if (!robot.isAutonomous() || !robot.isEnabled()) {
                    break;
                }
                robot.getRobotDrive().mecanumDrive_Cartesian(i.getRightX(), i.getRightY() * 0.75, i.getLeftX(), 0);
                robot.setLiftMotors(i.geteControlY());
                Timer.delay(delay);
            }
        } else if (autoType == 4) {
            commands.driveTimed(0.5, 0.0, 1.1, false);
            commands.stop();
        } else if (autoType == 5) {
            commands.driveUntilTote(0.0, 0.0, 500);
            commands.stop();
        } else {
            // 1. lift tote
            commands.setLiftOneTote();
            commands.waitForLift(0);

            // 2. get can out of the way
            commands.setClaw(true);
            commands.driveRotation(0.4, 60, 4);
            commands.setClaw(false);
            commands.driveRotation(0.35, 0, 4);

            // 3. drive to second tote
            commands.driveTimed(0.45, 0.0, 0.2, true);
            commands.driveTimed(0.9, 0.0, 0.63, true);
            commands.driveUntilTote(0.2, 0.0, 5);
            commands.stop();
            commands.setLiftTarget(0);
            commands.waitForLift(0.25);
            commands.setLiftOneTote();
            commands.waitForLift(0);

            // 4. move #2 can
            commands.setClaw(true);
            commands.driveRotation(0.4, 60, 4);
            commands.setClaw(false);
            commands.driveRotation(0.25, 0, 4);

            // 5. drive to third tote
            commands.driveTimed(0.45, 0.0, 0.2, true);
            commands.driveTimed(0.6, 0.0, 0.58, true);
            commands.driveUntilTote(0.2, 0, 5);
            commands.stop();
            commands.setLiftTarget(0);
            commands.waitForLift(0.25);
            commands.setLiftTarget(250);

            // 6. drive to auto zone
            commands.driveTimed(0.0, 1.0, 1.4, false);
            commands.driveTimed(0.0, 0.7, 0.5, false);
            commands.driveTimed(0.0, 0.45, 0.6, false);
            commands.setLiftTarget(0);
            commands.waitForLift(0);
            commands.driveTimed(-1.0, 0.0, 0.4, false);
            commands.driveTimed(0.5, 0.0, 0.08, false);
            commands.stop();
        }
    }

    public ArrayList<ControllerInformation> loadFile(String name) {
        ArrayList<ControllerInformation> list = new ArrayList<ControllerInformation>();
        File f = new File("/home/lvuser/" + name + ".txt");
        String input = null;
        try {
            BufferedReader br = new BufferedReader(new FileReader(f));
            input = br.readLine();
            br.close();
            send("Buffer over");
        } catch (Exception e) {
        }
        if (input != null) {
            for (String i : input.split(";")) {
                String[] t = i.split(",");
                double leftX = Double.valueOf(t[0]);
                double leftY = Double.valueOf(t[1]);
                double rightX = Double.valueOf(t[2]);
                double rightY = Double.valueOf(t[3]);
                double eControlX = Double.valueOf(t[4]);
                double eControlY = Double.valueOf(t[5]);
                list.add(new ControllerInformation(leftX, leftY, rightX, rightY, eControlX, eControlY));
            }
        }
        return list;
    }

    private void send(String message) {
        robot.send(message);
    }

    public double getInitialSonar() {
        return initialSonar;
    }

    public boolean fault() {
        return fault;
    }

    public void setFault(boolean fault) {
        this.fault = fault;
    }

    public Timer getTimer() {
        return time;
    }
}
