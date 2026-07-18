using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Robot_controller.Models
{
    public class Motor_Second_Order_Specs : INotifyPropertyChanged
    {
        private float _j;
        private float _b;
        private float _k1;
        private float _k2;

        public int MotorID { get; set; }
        public float J { get => _j; set => SetProperty(ref _j, value); }
        public float B { get => _b; set => SetProperty(ref _b, value); }
        public float K1 { get => _k1; set => SetProperty(ref _k1, value); }
        public float K2 { get => _k2; set => SetProperty(ref _k2, value); }

        public Motor_Second_Order_Specs(int motorid) 
        {
            MotorID = motorid;
            _j = 0;
            _b = 0;
            _k1 = 0;
            _k2 = 0;
        }

        public event PropertyChangedEventHandler? PropertyChanged;
        protected bool SetProperty<T>(ref T backingstore, T value, [CallerMemberName] string propertyname = "")
        {
            backingstore = value;
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyname));
            return true;
        }
    
    }
}
