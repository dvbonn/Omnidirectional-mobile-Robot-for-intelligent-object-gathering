
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Robot_controller.Models
{
    public partial class EKF : INotifyPropertyChanged
    {
        private float _x;
        private float _y;
        private float _theta;
        private float _v_x;
        private float _v_y;
        private float _omega;
        public float X { get => _x; set => SetProperty(ref _x, value); }
        public float Y { get => _y; set => SetProperty(ref _y, value); }
        public float Theta { get => _theta; set => SetProperty(ref _theta, value); }
        public float V_X { get => _v_x; set => SetProperty(ref _v_x, value); }
        public float V_Y { get => _v_y; set => SetProperty(ref _v_y, value); }
        public float Omega { get => _omega; set => SetProperty(ref _omega, value); }

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
