package frc.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants;

public class Controller {

    public static class DriverController {
        private final XboxController joystick = new XboxController(Constants.kDriverControllerPort);

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

        public enum Axis {
            /** Left X axis. */
            kLeftX(0),
            /** Right X axis. */
            kRightX(4),
            /** Left Y axis. */
            kLeftY(1),
            /** Right Y axis. */
            kRightY(5),
            /** Left trigger. */
            kLeftTrigger(2),
            /** Right trigger. */
            kRightTrigger(3);
            public final int value;
            Axis(int value){
                this.value = value;
            }
        }

        public JoystickButton getButton(Button button) {
            return new JoystickButton(joystick, button.getPort());
        }

        public double getAxis(Axis axis) {
            return joystick.getRawAxis(axis.value);
        }

        public Boolean getLeftTrigger() {
            return !(joystick.getLeftTriggerAxis() == 0);
        }
        public Boolean getRightTrigger() {
            return !(joystick.getRightTriggerAxis() == 0);
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
        public void activateRumble(){
            int cycle = 0;
            while (cycle < 20){
                joystick.setRumble(RumbleType.kBothRumble, 0);
                cycle++;
            }
            joystick.setRumble(RumbleType.kBothRumble, 0);
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