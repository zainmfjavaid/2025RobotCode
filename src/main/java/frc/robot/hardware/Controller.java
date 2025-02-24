package frc.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OperatorConstants;

public class Controller {

    public static class DriverController {
        private final Joystick joystick = new Joystick(OperatorConstants.kDriverControllerPort);

        public enum Button
        {
            X(1),
            A(2),
            B(3),
            Y(4),
            LB(5),
            RB(6),
            LT(7),
            RT(8),
            Back(9),
            Start(10),
            LJ(11),
            RJ(12);

            private int port;

            Button(int port)
            {
                this.port = port;
            }
            public int getPort()
            {
                return port;
            }
        }

        public JoystickButton getButton(Button button) {
            return new JoystickButton(joystick, button.getPort());
        }

        // Left is positive
        public double getLeftStickX() {
            return -joystick.getRawAxis(0);
        }
        // Up is positive
        public double getLeftStickY() {
            return -joystick.getRawAxis(1);
        }
        // Left is positive
        public double getRightStickX() {
            return -joystick.getRawAxis(2);
        }
        // Up is positive
        public double getRightStickY() {
            return -joystick.getRawAxis(3);
        }

        public void printJoystickAxes() {
            System.out.println("Joystick Axes");
            System.out.println("LX: " + getLeftStickX());
            System.out.println("LY: " + getLeftStickY());
            System.out.println("RX: " + getRightStickX());
            System.out.println("RY: " + getRightStickY());
        }
    }

    public static class OperatorController {
        private final Joystick joystick = new Joystick(OperatorConstants.kOperatorControllerPort);

        public static enum Button {
            X(1),
            A(2),
            B(3),
            Y(4),
            LB(5), // Left Bumper
            RB(6), // Right Bumper
            LT(7), // Left Trigger
            RT(8), // Right Trigger
            Back(9),
            Start(10),
            LJ(11), // Left Joystick Button
            RJ(12),  // Right Joystick Button
            POVUP(0),
            POVDOWN(180),
            POVLEFT(270),
            POVRIGHT(90);
            
            private final int port; 
            
            private Button(int port) {
                this.port = port;
            }
            
            public int getPort() {
                return port;
            }
        }

        public JoystickButton getButton(Button button) {
            return new JoystickButton(joystick, button.getPort());
        }
    }
}