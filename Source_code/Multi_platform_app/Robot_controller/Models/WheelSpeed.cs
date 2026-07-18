
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Robot_controller.Models
{
    public partial class WheelSpeed : INotifyPropertyChanged
    {
        private float _w1;
        private float _w2;
        private float _w3;
        private float _w4;

        public float W1 { get => _w1; set => SetProperty(ref _w1, value); }
        public float W2 { get => _w2; set => SetProperty(ref _w2, value); }
        public float W3 { get => _w3; set => SetProperty(ref _w3, value); }
        public float W4 { get => _w4; set => SetProperty(ref _w4, value); }

        public event PropertyChangedEventHandler? PropertyChanged;
        protected bool SetProperty<T>(ref T backingStore, T value, [CallerMemberName] string propertyName = "")
        {
            //if(Equals(backingStore, value))
            //    return false;
            backingStore = value;
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
            return true;
        }

    }
}
