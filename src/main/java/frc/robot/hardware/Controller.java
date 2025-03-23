package frc.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants;

public class Controller {

    public static class DriverController {
        private final Joystick joystick = new Joystick(Constants.kDriverControllerPort);

        public enum Button
        {
            A(1),
            B(2),
            X(3),
            Y(4),
            LB(5),
            RB(6),
            Back(7),
            Start(8),
            LJ(9),
            RJ(10);

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

        public double getLeftTrigger() {
            return joystick.getRawAxis(2);
        }
        public double getRightTrigger() {
            return joystick.getRawAxis(3);
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
            return -joystick.getRawAxis(4);
        }
        // Up is positive
        public double getRightStickY() {
            return -joystick.getRawAxis(5);
        }
    }

    public static class OperatorController {
        private final Joystick joystick = new Joystick(Constants.kOperatorControllerPort);

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