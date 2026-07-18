using Microsoft.Maui.Controls;
using SQLite;

namespace Robot_controller.Models
{
    public class Database
    {
        private readonly string Databasefilename = "data.db3";
        private readonly SQLite.SQLiteOpenFlags Flags =
            SQLite.SQLiteOpenFlags.ReadWrite |
            SQLite.SQLiteOpenFlags.Create |
            SQLite.SQLiteOpenFlags.NoMutex |
            SQLite.SQLiteOpenFlags.SharedCache |
            SQLite.SQLiteOpenFlags.ProtectionNone;
        private string DatabasePath => Path.Combine(FileSystem.AppDataDirectory, Databasefilename);
        SQLiteAsyncConnection database = null!;
        async Task Init()
        {
            if (database is not null) return;
            database = new SQLiteAsyncConnection(DatabasePath, Flags);
            await database.CreateTableAsync<RunSession>();
            await database.CreateTableAsync<RecordBone>();
        }

        public async Task<int> SaveDataAsync(RunSession session)
        {
            await Init();
            await database.InsertAsync(session);
            return session.RunId;
        }

        public async Task<int> SaveDataAsync(List<RecordBone> records)
        {
            await Init();
            return await database.InsertAllAsync(records);
        }
    }
}
