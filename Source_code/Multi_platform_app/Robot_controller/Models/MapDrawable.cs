
namespace Robot_controller.Models
{
    public class MapDrawable : IDrawable
    {
        public static PathF TrajectoryPath { get; set; } = new();
        public static PointF RobotPosition { get; set; } = new(0, 0);
        public static PointF Offset { get; set; } = new(0, 0);
        public void Draw(ICanvas canvas, RectF dirtyRect)
        {
            canvas.SaveState();
            float centerX = MathF.Floor(dirtyRect.Width / (2 * 60)) * 60;
            float centerY = MathF.Floor(dirtyRect.Height / (2 * 60)) * 60;
            float deltaX = MathF.Floor(centerX - (RobotPosition.X - Offset.X));
            float deltaY = MathF.Floor(centerY - (RobotPosition.Y - Offset.Y));
            canvas.Translate(deltaX, deltaY);
            canvas.StrokeColor = Colors.DarkGray;
            canvas.StrokeSize = 0.5f;
            canvas.FontColor = Colors.White;
            canvas.FontSize = 13;
            float minX = -deltaX + 60;
            float maxX = dirtyRect.Width - deltaX;
            float minY = -deltaY;
            float maxY = dirtyRect.Height - deltaY - 60;
            
            float last_y = minY;
            for (; last_y < maxY; last_y += 60)
            {
                canvas.DrawLine(minX, last_y, maxX, last_y);
                if(last_y != minY)
                canvas.DrawString($"{(last_y / 100.0f):0.0}", minX - 10, last_y, HorizontalAlignment.Right);
            }
            last_y -= 60;

            for (float x = minX; x < maxX; x += 60)
            {
                canvas.DrawLine(x, minY, x, last_y);
                canvas.DrawString($"{(x / 100.0f):0.0}", x, maxY, HorizontalAlignment.Justified);
            }

            if(RobotPosition.X >= minX &&  RobotPosition.Y <= last_y)
            {
                canvas.FillColor = Colors.Orange;
                canvas.FillCircle(RobotPosition.X, RobotPosition.Y, 5);
            }

            canvas.SaveState();
            float width = maxX - minX;
            float height = last_y - minY;

            //canvas.ClipRectangle(deltaX, deltaY, width, height);
            canvas.StrokeColor = Colors.BlueViolet;
            canvas.StrokeSize = 1.5f;
            canvas.DrawPath(TrajectoryPath);


            canvas.RestoreState();

            canvas.RestoreState();
        }
    }
}
