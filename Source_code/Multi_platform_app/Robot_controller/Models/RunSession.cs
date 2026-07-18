using SQLite;

namespace Robot_controller.Models
{
    public class RunSession
    {
        [PrimaryKey, AutoIncrement]
        public int RunId { get; set; }
        public DateTime StartTime { get; set; }
    }
}
