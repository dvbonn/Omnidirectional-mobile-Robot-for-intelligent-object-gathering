namespace Robot_controller.Models
{
    public static class RobotControl
    {
        private static readonly float X_Speed = 0.15f;
        private static readonly float Y_Speed = 0.15f;
        private static readonly float Omega = 0.3f;
        private static bool IsMoving = false;

        public static void SendControl(string key)
        {
            if (string.IsNullOrEmpty(key) || !Socket_handler.IsConnected() || IsMoving) return;
            IsMoving = true;
            float dot_x = 0;
            float dot_y = 0;
            float omega = 0;
            switch (key)
            {
                case "W":
                    dot_y = Y_Speed;
                    break;
                case "S":
                    dot_y = -Y_Speed;
                    break;
                case "A":
                    dot_x = -X_Speed;
                    break;
                case "D":
                    dot_x = X_Speed;
                    break;
                case "Q":
                    omega = Omega;
                    break;
                case "E":
                    omega = -Omega;
                    break;
                default:
                    break;
            }
            System.Diagnostics.Debug.WriteLine(key);
            VelocityMsgT VelocityMsg = new()
            {
                Cmd = CmdType.SET_ROBOT_VELOCITY,
                velocity = new VelocityT
                {
                    VX = dot_x,
                    VY = dot_y,
                    VTheta = omega,
                }
            };
            Socket_handler.SendCommand(VelocityMsg);
        }

        public static void SendBrake()
        {
            if(Socket_handler.IsConnected())
            {
                IsMoving = false;
                Socket_handler.SendCommand(new GenericMsgT { Cmd = CmdType.STOP_ROBOT });
            }
        }

    }
}
