using Robot_controller.Models;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;

namespace Robot_controller.Views;

public class ChartPoint
{
	public float W1 { get; set; }
	public float W2 { get; set; }
	public float W3 { get; set; }
	public float W4 { get; set; }
	public int TimeStamp { get; set; }
}

public partial class MotorChart : ContentPage
{
	private readonly int MaxPoints = 100;
	private readonly WheelSpeed _wheelspeed;
	public ObservableCollection<Motor_Second_Order_Specs> Specs { get; private set; }
	private readonly Stopwatch _stopwatch = new (); 
	public ObservableCollection<ChartPoint> ChartData { get; private set; } 
	public MotorChart(WheelSpeed wheelSpeed, ObservableCollection<Motor_Second_Order_Specs> motor_Second_Order_Specs)
	{
		InitializeComponent();
		ChartData = new ObservableCollection<ChartPoint>();
		_wheelspeed = wheelSpeed;
		Specs = motor_Second_Order_Specs;
		BindingContext = this;
	}

    protected override void OnAppearing()
    {
        base.OnAppearing();
		_stopwatch.Restart();
		_wheelspeed.PropertyChanged += OnWheelSpeedChange;
    }

	private void OnWheelSpeedChange(object? sender, PropertyChangedEventArgs e)
	{
		MainThread.BeginInvokeOnMainThread(() =>
		{
			var datapoint = new ChartPoint
			{
				W1 = _wheelspeed.W1,
				W2 = _wheelspeed.W2,
				W3 = _wheelspeed.W3,
				W4 = _wheelspeed.W4,
				TimeStamp = (int)_stopwatch.ElapsedMilliseconds,
			};

			ChartData.Add(datapoint);

			if (ChartData.Count > MaxPoints)
			{
				ChartData.RemoveAt(0);
			}
		});
	}

    protected override void OnDisappearing()
    {
        base.OnDisappearing();
		ChartData.Clear();
		_wheelspeed.PropertyChanged -= OnWheelSpeedChange;
    }

    private void SetSpeedBtn_Clicked(object sender, EventArgs e)
    {
		var SpeedW1 = CheckBoxW1.IsChecked ? float.Parse(SpeedEntry.Text) : 0;
		var SpeedW2 = CheckBoxW2.IsChecked ? float.Parse(SpeedEntry.Text) : 0;
		var SpeedW3 = CheckBoxW3.IsChecked ? float.Parse(SpeedEntry.Text) : 0;
		var SpeedW4 = CheckBoxW4.IsChecked ? float.Parse(SpeedEntry.Text) : 0;

		MotorMsgT MotorMsg = new()
		{ 
			Cmd = CmdType.SET_MOTOR_SPEED,
			MotorSpeeds = [SpeedW1, SpeedW2, SpeedW3, SpeedW4]
		};

		Socket_handler.SendCommand(MotorMsg);
	}

    private void GetSpecs_Clicked(object sender, EventArgs e)
    {
		Socket_handler.SendCommand(new GenericMsgT { Cmd = CmdType.MOTOR_SPECS });
    }
}