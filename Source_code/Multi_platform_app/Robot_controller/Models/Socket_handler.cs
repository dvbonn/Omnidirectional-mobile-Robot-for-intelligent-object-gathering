using CommunityToolkit.Maui.Alerts;
using CommunityToolkit.Maui.Core;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Security.Cryptography;


namespace Robot_controller.Models
{
    public static class Socket_handler
    {
        private static readonly int MaxSend = 64 * 1024;
        private static readonly int MaxReceive = 500;
        private static TaskCompletionSource<bool>? OTAResultTcs;

        private static Socket ClientSocket = new(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp)
        {
            NoDelay = true,
            ReceiveBufferSize = MaxReceive,
            SendBufferSize = MaxSend
        };

        public static string IsConnectedStr()
        {
            if (ClientSocket.Connected)
            {
                return "Online";
            }
            else
            {
                CloseSocket();
                MDNS.StopMDNS(); //restart mDNS to find the device again after OTA
                MDNS.StartMDNS();
                return "Offline";
            }
        }

        public static bool IsConnected()
        {
            return ClientSocket.Connected;
        }
        public static async void ConnectSocket(String Ip)
        {
            try
            {
                if (!ClientSocket.Connected)
                {
                    await ClientSocket.ConnectAsync(IPAddress.Parse(Ip), 2004);
                }
            }
            catch (Exception)
            {}
        }

        public static async Task SendCommand<T>(T msg) where T : struct
        {
            try
            {
                byte[] data = StructToBytes(msg);
                using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(10));
                await ClientSocket.SendAsync(data, SocketFlags.None, cts.Token);
            }
            catch (Exception)
            {
                MainThread.BeginInvokeOnMainThread(async () =>
                {
                    await Toast.Make("❌ Không tìm thấy IP Robot", ToastDuration.Long).Show();
                    CloseSocket();
                });
            }
        }

        public static void SetOTAResult(bool result)
        {
            OTAResultTcs?.TrySetResult(result);
        }

        private static async Task<string> CalculateSHA256(Stream stream)
        {
            using var sha256 = SHA256.Create();
            stream.Position = 0;
            var hash = await sha256.ComputeHashAsync(stream);
            return Convert.ToHexStringLower(hash);
        }
        public static async Task<bool> SendFile(FileResult? file, IProgress<double>? progess = null)
        {
            try
            {
                if (file == null) return false;

                using var stream = await file.OpenReadAsync();

                OTAUpdatePreMsgT OTAUpdatePreMsg = new()
                {
                    Cmd = CmdType.OTA_UPDATE,
                    FirmwareSize = (uint)stream.Length,
                    Sha256 = await CalculateSHA256(stream),
                };

                await SendCommand(OTAUpdatePreMsg);
                OTAResultTcs = new TaskCompletionSource<bool>();
                var status = await OTAResultTcs.Task.WaitAsync(TimeSpan.FromSeconds(10));
                if (status)
                {
                    var totalChunks = (stream.Length + MaxSend - 1) / MaxSend;
                    var buffer = new byte[MaxSend];
                    int currentChunk = 0;
                    stream.Position = 0;
                    while (stream.Position < stream.Length)
                    {
                        var bytesRead = await stream.ReadAsync(buffer.AsMemory());
                        using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(10)); 
                        await ClientSocket.SendAsync(buffer.AsMemory(0, bytesRead), SocketFlags.None, cts.Token);
                        currentChunk++;
                        progess?.Report((double)currentChunk / totalChunks);
                    }
                    OTAResultTcs = new TaskCompletionSource<bool>();
                    status = await OTAResultTcs.Task.WaitAsync(TimeSpan.FromSeconds(10));
                    if (status)
                    {
                        CloseSocket();
                        return true;
                    }
                    return false;
                }
                return false;
            }
            catch (Exception E)
            {
                throw new Exception($"OTA file send failed: {E.Message}", E);
            }
        }

        private static async Task ReceiveExactAsync(byte[] buffer, int targetBytes)
        {
            int totalRead = 0;
            while (totalRead < targetBytes)
            {
                int bytesLeft = targetBytes - totalRead;

                var bufferSlice = new ArraySegment<byte>(buffer, totalRead, bytesLeft);

                using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(10));
                int bytesReadNow = await ClientSocket.ReceiveAsync(bufferSlice, SocketFlags.None, cts.Token);

                if (bytesReadNow == 0)
                {
                    throw new SocketException((int)SocketError.ConnectionReset);
                }

                totalRead += bytesReadNow;
            }

        }

        public static T BytesToStruct<T>(byte[] bytes) where T : struct
        {
            int size = Marshal.SizeOf<T>();
            IntPtr ptr = Marshal.AllocHGlobal(size);
            try
            {
                Marshal.Copy(bytes, 0, ptr, size);
                return Marshal.PtrToStructure<T>(ptr);
            }
            finally 
            { 
                Marshal.FreeHGlobal(ptr); 
            }
        }

        public static byte[] StructToBytes<T>(T str) where T : struct
        {
            int size = Marshal.SizeOf(str);
            byte[] bytes = new byte[size];
            IntPtr ptr = Marshal.AllocHGlobal(size);
            try
            {
                Marshal.StructureToPtr(str, ptr, true);
                Marshal.Copy(ptr, bytes, 0, size);
                return bytes;
            }
            finally
            {
                Marshal.FreeHGlobal(ptr);
            }
        }

        public static async Task<byte[]> ReceiveData()
        {
            try
            {
                if (ClientSocket.Connected)
                {
                    byte[] buffer_len = new byte[1];
                    await ReceiveExactAsync(buffer_len, 1);

                    int packetSize = buffer_len[0];

                    if (packetSize <= 0 || packetSize > 255)
                    {
                        return null!;
                    }

                    byte[] payload = new byte[packetSize];
                    await ReceiveExactAsync(payload, packetSize);
                    return payload;
                }
                return null!;
            }
            catch (Exception)
            {
                return null!;
            }
        }

        public static void CloseSocket()
        {
            ClientSocket?.Dispose();
            ClientSocket = new(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp)
            {
                NoDelay = true,
                ReceiveBufferSize = MaxReceive,
                SendBufferSize = MaxSend
            };
        }
    }
}
