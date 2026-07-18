using CommunityToolkit.Maui.Alerts;
using CommunityToolkit.Maui.Core;
using Robot_controller.Models;
using Robot_controller.Views;
using System.Collections.ObjectModel;
using System.ComponentModel;

namespace Robot_controller
{
    public partial class MainPage : ContentPage, INotifyPropertyChanged
    {
        private FileResult? FirmwareFile;
        private readonly MapDrawable _mapDrawable;
        private string _socketstate = "Offline";
        private Color _socketstatecolor = Colors.Red;
        public Color SocketStateColor
        {
            get => _socketstatecolor;
            set
            {
                if (_socketstatecolor != value)
                {
                    _socketstatecolor = value;

                    OnPropertyChanged(nameof(SocketStateColor));
                }
            }
        }
        public string SocketState
        {
            get => _socketstate;
            set
            {
                if(_socketstate != value)
                {
                    _socketstate = value;
                    OnPropertyChanged(nameof(SocketState));

                    SocketStateColor = _socketstate switch
                    {
                        "Online" => Colors.LimeGreen,
                        "Offline"=> Colors.Red,
                        _ => Colors.Gray
                    };

                }
            }
        }

        public WheelSpeed Speeds { get; } = new WheelSpeed();
        public ObservableCollection<Motor_Second_Order_Specs> Specs { get; } =
        [
            new (1), 
            new (2),
            new (3), 
            new (4)
        ];
        public EKF Ekf { get; } = new EKF();
        public BNO055Data BNO055 { get; } = new BNO055Data();
        public PMW3901Data PMW3901 { get; } = new PMW3901Data();

        private readonly Database database;
        private bool isCollectData = false;
        private int RunId = 0;
        public List<RecordBone> Records { get; set; } = [];

        private readonly Window MotorChart;

        private void UpdateMap(float x, float y)
        {
            MapDrawable.RobotPosition = new(x * 100, y * 100);
            if (MapDrawable.TrajectoryPath.OperationCount > 0)
            {
                MapDrawable.TrajectoryPath.LineTo(MapDrawable.RobotPosition);
            }
            else
            {
                MapDrawable.TrajectoryPath.MoveTo(MapDrawable.RobotPosition);
            }
            RobotCanvas.Invalidate();
        }
        private void StartReceiveMessage()
        {
            Task.Run(async () =>
            {
                while (true)
                {
                    try
                    {
                        SocketState = Socket_handler.IsConnectedStr();
                        var rawbytes = await Socket_handler.ReceiveData();
                        if (rawbytes != null)
                        {
                            var cmd = (CmdType)rawbytes[0];
                            switch (cmd)
                            {
                                case CmdType.OTA_UPDATE: 
                                    var OTAMsg = Socket_handler.BytesToStruct<OTAUpdateMsgT>(rawbytes);
                                    Socket_handler.SetOTAResult(OTAMsg.State);
                                    break;
                                case CmdType.MOTOR_SPEED: 
                                    var MotorMsg = Socket_handler.BytesToStruct<MotorMsgT>(rawbytes);
                                    Speeds.W1 = MotorMsg.MotorSpeeds[0];
                                    Speeds.W2 = MotorMsg.MotorSpeeds[1];
                                    Speeds.W3 = MotorMsg.MotorSpeeds[2];
                                    Speeds.W4 = MotorMsg.MotorSpeeds[3];
                                    break;
                                case CmdType.ROBOT_STATE:
                                    var RobotStateMsg = Socket_handler.BytesToStruct<RobotStateMsgT>(rawbytes);
                                    Ekf.X = RobotStateMsg.RobotState.Position.X;
                                    Ekf.Y = RobotStateMsg.RobotState.Position.Y;
                                    Ekf.Theta = RobotStateMsg.RobotState.Position.Theta;
                                    Ekf.V_X = RobotStateMsg.RobotState.Velocity.VX;
                                    Ekf.V_Y = RobotStateMsg.RobotState.Velocity.VY;
                                    Ekf.Omega = RobotStateMsg.RobotState.Velocity.VTheta;
                                    if(isCollectData)
                                    Records.Add(new RecordBone{ RunId = RunId, TimeStamp = 0, 
                                                                X = Ekf.X, Y = Ekf.Y, 
                                                                Theta = Ekf.Theta, V_X = Ekf.V_X, 
                                                                V_Y = Ekf.V_Y, Omega = Ekf.Omega });

                                    UpdateMap(Ekf.X, Ekf.Y);
                                    break;
                                case CmdType.MOTOR_SPECS:
                                    var MotorSpecsMsg = Socket_handler.BytesToStruct<MotorSpecsMsgT>(rawbytes);
                                    for ( int i = 0; i < 4;  i++ )
                                    {
                                        Specs[i].J = MotorSpecsMsg.Specs[i].J;
                                        Specs[i].B = MotorSpecsMsg.Specs[i].B;
                                        Specs[i].K1 = MotorSpecsMsg.Specs[i].K1;
                                        Specs[i].K2 = MotorSpecsMsg.Specs[i].K2;
                                    }
                                    break;
                                case CmdType.BNO055_DATA:
                                    var BNO055Data = Socket_handler.BytesToStruct<BNO055MsgT>(rawbytes);
                                    BNO055.Accel_Status = (byte)((BNO055Data.CalibrationStatus >> 2) & 0x03);
                                    BNO055.Gyro_Status = (byte)(BNO055Data.CalibrationStatus & 0x03);
                                    BNO055.Heading = BNO055Data.heading;
                                    break;
                                case CmdType.PMW3901_DATA:
                                    var PMW3901Data = Socket_handler.BytesToStruct<PMW3901MsgT>(rawbytes);
                                    PMW3901.V_X = PMW3901Data.VX;
                                    PMW3901.V_Y = PMW3901Data.VY;
                                    break;
                                default:
                                    break;
                            }
                        }
                        else
                        { 
                            await Task.Delay(500);
                        }
                    }
                    catch (Exception)
                    {
                        await Task.Delay(500);
                        // Handle exceptions if necessary
                    }
                }
            });
        }
        public MainPage()
        {
            InitializeComponent();
            MotorChart = new Window(new MotorChart(Speeds, Specs));
            _mapDrawable = new MapDrawable();
            RobotCanvas.Drawable = _mapDrawable;
            database = new Database();
        }

        protected override async void OnAppearing()
        {
            base.OnAppearing();
            BindingContext = this;

            await Task.Delay(500);
            MDNS.StartMDNS();
            StartReceiveMessage();
        }

        private async void ConnectWifiButton_Clicked(object sender, EventArgs e)
        {
            
            try
            {
                if(SSIDValid.IsNotValid || PassValid.IsNotValid)
                {
                    throw new Exception("SSID hoặc mật khẩu không hợp lệ");
                }
                WifiSetMsgT msg = new ()
                {
                    Cmd = CmdType.WIFI_SET,
                    SSID = WifiSsidEntry.Text,
                    Password = WifiPasswordEntry.Text,
                };
                await Socket_handler.SendCommand(msg);
            }
            catch (Exception E)
            {
                await Toast.Make("❌ " + E.Message, ToastDuration.Long).Show();
            }
        }
        private async void SelectFirmwarreBtn_Clicked(object sender, EventArgs e)
        {
            try
            {
                var FileType = new FilePickerFileType(new Dictionary<DevicePlatform, IEnumerable<string>>
            {
                { DevicePlatform.iOS, new[] { "public.item" } },
                { DevicePlatform.Android, new[] { "application/octet-stream" } },
                { DevicePlatform.WinUI, new[] { ".bin"} },
                { DevicePlatform.macOS, new[] { "bin"} },
            });

                PickOptions options = new()
                {
                    PickerTitle = "Chọn firmware (.bin)",
                    FileTypes = FileType,
                };

                FirmwareFile = await FilePicker.Default.PickAsync(options);
                if (FirmwareFile != null)
                {
                    if (FirmwareFile.FileName.EndsWith(".bin", StringComparison.OrdinalIgnoreCase))
                    {
                        SelectFirmwarreBtn.Text = FirmwareFile.FileName;
                        UpdateOTABtn.IsEnabled = true;
                    }
                    else
                    {
                        UpdateOTABtn.IsEnabled = false;
                        await Toast.Make("❌ Vui lòng chọn file có định dạng .bin", ToastDuration.Long).Show();
                    }

                }
            }
            catch (Exception E)
            {
                UpdateOTABtn.IsEnabled = false;
                await Toast.Make("❌ " + E.Message, ToastDuration.Long).Show();
            }
        }
        private async void UpdateOTABtn_Clicked(object sender, EventArgs e)
        {
            try
            {
                UpdateProgressBar.IsVisible = true;
                SelectFirmwarreBtn.IsEnabled = false;
                UpdateOTABtn.IsEnabled = false;
                var progess = new Progress<double>(value =>
                {
                    UpdateProgressBar.Progress = value;     
                });

                bool success = await Socket_handler.SendFile(FirmwareFile, progess);
                if (success)
                {
                    await Toast.Make("✔️ Cập nhật firmware thành công", ToastDuration.Long).Show();
                }
                else
                {
                    await Toast.Make("❌ Cập nhật firmware thất bại!", ToastDuration.Long).Show();
                }

            }
            catch (Exception E)
            {
                await Toast.Make("❌ " + E.Message, ToastDuration.Long).Show();
            }
            finally
            {
                UpdateProgressBar.IsVisible = false;
                SelectFirmwarreBtn.IsEnabled = true;
                UpdateOTABtn.IsEnabled = true;
                UpdateProgressBar.Progress = 0;
                SelectFirmwarreBtn.Text = "Chọn file firmware";
                Socket_handler.CloseSocket();
            }
        }

        private void ShowMotorChartButton_Clicked(object sender, EventArgs e)
        {
            if(!Application.Current?.Windows.Contains(MotorChart) ?? false)
            {
                Application.Current?.OpenWindow(MotorChart);
            }
            else
            {
                Application.Current?.ActivateWindow(MotorChart);
            }
        }


        private async void CollectDataBtn_Clicked(object sender, EventArgs e)
        {
            try
            {
                RunId = await database.SaveDataAsync(new RunSession { StartTime = DateTime.Now });
                isCollectData = true;
                StopCollectBtn.IsEnabled = true;
                CollectDataBtn.IsEnabled = false;
            }
            catch (Exception E)
            {
                await Toast.Make("❌ " + E.Message, ToastDuration.Long).Show();
            }
        }

        private async void StopCollectBtn_Clicked(object sender, EventArgs e)
        {
            try
            {
                isCollectData = false;
                await database.SaveDataAsync(Records);
                await Toast.Make($"✔️ Lưu thành công bản ghi {RunId}", ToastDuration.Short).Show();
                StopCollectBtn.IsEnabled = false;
                CollectDataBtn.IsEnabled = true;
            }
            catch (Exception E)
            {
                await Toast.Make("❌ " + E.Message, ToastDuration.Long).Show();
            }
        }

        float StartX, StartY;

        private async void AutoPidButton_Clicked(object sender, EventArgs e)
        {
            try
            {
                await Socket_handler.SendCommand(new GenericMsgT { Cmd = CmdType.AUTO_TUNE });
            }
            catch (Exception E)
            {
                await Toast.Make("❌ " + E.Message, ToastDuration.Long).Show();
            }
        }

        private async void ReCalibrationBtn_Clicked(object sender, EventArgs e)
        {
            try
            {
                await Socket_handler.SendCommand(new GenericMsgT { Cmd = CmdType.BNO055_RECALIBRATION });
            }
            catch (Exception E)
            {
                await Toast.Make("❌ " + E.Message, ToastDuration.Long).Show();
            }
        }

        private void TapGestureRecognizer_Tapped(object sender, TappedEventArgs e)
        {
            MapDrawable.Offset = new(0, 0);
            RobotCanvas.Invalidate();
        }

        private void PanGestureRecognizer_PanUpdated(object sender, PanUpdatedEventArgs e)
        {
            switch (e.StatusType)
            {
                case GestureStatus.Started:
                    StartX = MapDrawable.Offset.X;
                    StartY = MapDrawable.Offset.Y;
                    break;
                case GestureStatus.Running:
                    MapDrawable.Offset = new(StartX + (float)e.TotalX, StartY + (float)e.TotalY);
                    RobotCanvas.Invalidate();
                    break;
                default:
                    break;
            }
        }

    }
}
