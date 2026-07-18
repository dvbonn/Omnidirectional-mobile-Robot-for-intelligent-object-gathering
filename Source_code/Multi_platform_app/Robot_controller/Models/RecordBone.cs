using SQLite;

namespace Robot_controller.Models
{
    public class RecordBone
    {
        [PrimaryKey, AutoIncrement]
        public int Id { get; set; }
        [Indexed]
        public int RunId { get; set; }
        public int TimeStamp { get; set; }
        public float X { get; set; }
        public float Y { get; set; }
        public float Theta { get; set; }
        public float V_X { get; set; }
        public float V_Y { get; set; }
        public float Omega { get; set; }

    }
}
