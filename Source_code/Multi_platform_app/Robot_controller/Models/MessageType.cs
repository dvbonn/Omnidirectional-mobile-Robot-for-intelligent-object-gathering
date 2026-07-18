
using System.Runtime.InteropServices;

namespace Robot_controller.Models
{
    public enum CmdType : byte
    {
        WIFI_SET,
        OTA_UPDATE,
        SET_MOTOR_SPEED,
        SET_ROBOT_VELOCITY,
        STOP_ROBOT,
        AUTO_TUNE,
        MOTOR_SPECS,
        MOTOR_SPEED,
        ROBOT_STATE,
        BNO055_DATA,
        BNO055_RECALIBRATION,
        PMW3901_DATA
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct MotorMsgT
    {
        public CmdType Cmd;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public float[] MotorSpeeds;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct VelocityT
    {
        public float VX;
        public float VY;
        public float VTheta;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct PositionT
    {
        public float X;
        public float Y;
        public float Theta;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct RobotStateT
    {
        public VelocityT Velocity;
        public PositionT Position;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct RobotStateMsgT 
    {
        public CmdType Cmd;
        public RobotStateT RobotState;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct MotorParams
    {
        public float J;
        public float B;
        public float K1; 
        public float K2;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct MotorSpecsMsgT
    {
        public CmdType Cmd;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public MotorParams[] Specs;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct OTAUpdateMsgT
    {
        public CmdType Cmd;
        [MarshalAs(UnmanagedType.I1)] //because of C# consist bool as 4 bytes not 1 byte so we need this to avoid 3 bytes padding
        public bool State;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct WifiSetMsgT
    {
        public CmdType Cmd;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)]
        public string SSID;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
        public string Password;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct OTAUpdatePreMsgT
    {
        public CmdType Cmd;
        public uint FirmwareSize;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 65)]
        public string Sha256;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct VelocityMsgT
    {
        public CmdType Cmd;
        public VelocityT velocity;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct BNO055MsgT
    {
        public CmdType Cmd;
        public byte CalibrationStatus;
        public float heading;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct PMW3901MsgT{
        public CmdType Cmd;
        public float VX;
        public float VY;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct GenericMsgT
    {
        public CmdType Cmd;
    }
}
