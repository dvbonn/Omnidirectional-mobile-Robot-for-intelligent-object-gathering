
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Robot_controller.Models
{
    public partial class BNO055Data : INotifyPropertyChanged
    {
        private byte _accel_status;
        private byte _gyro_status;
        private float _heading;
        public byte Accel_Status { get => _accel_status; set => SetProperty(ref _accel_status, value); }
        public byte Gyro_Status { get => _gyro_status; set => SetProperty(ref _gyro_status, value); }
        public float Heading { get => _heading; set => SetProperty(ref _heading, value); }

        public event PropertyChangedEventHandler? PropertyChanged;

        protected bool SetProperty<T>(ref T backingStore, T value, [CallerMemberName] string propertyName = "")
        {
            //if (Equals(backingStore, value))
            //    return false;
            backingStore = value;
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
            return true;
        }

    }
}
