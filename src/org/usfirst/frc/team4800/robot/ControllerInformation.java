package org.usfirst.frc.team4800.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ControllerInformation {

    private double leftX;
    private double leftY;
    private double rightX;
    private double rightY;
    private double eControlX;
    private double eControlY;

    public ControllerInformation(double leftX, double leftY, double rightX, double rightY, double eControlX, double eControlY) {
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;
        this.eControlX = eControlX;
        this.eControlY = eControlY;
    }

    public ControllerInformation(Joystick left, Joystick right, Joystick eControl) {
        this.leftX = left.getX();
        this.leftY = left.getY();
        this.rightX = right.getX();
        this.rightY = right.getY();
        this.eControlX = eControl.getX();
        this.eControlY = eControl.getY();
    }

    public double getLeftX() {
        return leftX;
    }

    public void setLeftX(double leftX) {
        this.leftX = leftX;
    }

    public double getLeftY() {
        return leftY;
    }

    public void setLeftY(double leftY) {
        this.leftY = leftY;
    }

    public double getRightX() {
        return rightX;
    }

    public void setRightX(double rightX) {
        this.rightX = rightX;
    }

    public double getRightY() {
        return rightY;
    }

    public void setRightY(double rightY) {
        this.rightY = rightY;
    }

    public double geteControlX() {
        return eControlX;
    }

    public void seteControlX(double eControlX) {
        this.eControlX = eControlX;
    }

    public double geteControlY() {
        return eControlY;
    }

    public void seteControlY(double eControlY) {
        this.eControlY = eControlY;
    }

}
