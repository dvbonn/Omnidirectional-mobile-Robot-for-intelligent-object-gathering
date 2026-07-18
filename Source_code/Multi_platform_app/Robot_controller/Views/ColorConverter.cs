using System.Globalization;

namespace Robot_controller.Views
{
    public class ColorConverter : IValueConverter
    {
        public Color PosColor { get; set; } = Colors.LimeGreen;
        public Color NegColor { get; set; } = Colors.Red;
        public Color ZeroColor { get; set; } = Colors.Yellow;

        public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture)
        {
            if (value is not float floatValue)
            {
                if (float.TryParse(value?.ToString(), out float d))
                   floatValue = d;
                else
                    return ZeroColor;
            }

            if(floatValue > 0)
            {
                return PosColor;
            }
            else if(floatValue < 0)
            {
                return NegColor;
            }
            return ZeroColor;
        }

        public object ConvertBack(object? value, Type targetType, object? parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
