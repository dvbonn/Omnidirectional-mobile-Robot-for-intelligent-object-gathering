using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Robot_controller.Models
{
    public partial class PMW3901Data : INotifyPropertyChanged
    {
        private float _vx;
        private float _vy;
        public float V_X { get => _vx; set => SetProperty(ref _vx, value); }
        public float V_Y { get => _vy; set => SetProperty(ref _vy, value); }

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
