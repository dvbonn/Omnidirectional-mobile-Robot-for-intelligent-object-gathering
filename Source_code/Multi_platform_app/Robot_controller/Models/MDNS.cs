using CommunityToolkit.Maui.Alerts;
using CommunityToolkit.Maui.Core;
using Tmds.MDns;

namespace Robot_controller.Models
{
    public static class MDNS
    {
        private static ServiceBrowser serviceBrowser = null!;

        public static void StartMDNS()
        {
            if (serviceBrowser != null) return;
            serviceBrowser = new ServiceBrowser();
            serviceBrowser.ServiceAdded += OnServiceAdded;
            serviceBrowser.ServiceRemoved += OnServiceRemoved;
            serviceBrowser.StartBrowse("_robot._tcp");
        }

        public static void StopMDNS()
        {
            if (serviceBrowser == null) return;
            serviceBrowser.ServiceAdded -= OnServiceAdded;
            serviceBrowser.ServiceRemoved -= OnServiceRemoved;
            serviceBrowser.StopBrowse();
            serviceBrowser = null!;
        }

        private static void OnServiceAdded(object? sender, ServiceAnnouncementEventArgs e)
        {
            AlertService("+", e.Announcement);
            Socket_handler.ConnectSocket(e.Announcement.Addresses.First().ToString());
        }

        private static void OnServiceRemoved(object? sender, ServiceAnnouncementEventArgs e)
        {
            AlertService("–", e.Announcement);
            Socket_handler.CloseSocket();
        }

        private static void AlertService(string startchar, ServiceAnnouncement service)
        {
            MainThread.BeginInvokeOnMainThread(async () =>
            {
                await Toast.Make($"{startchar} Service {service.Hostname} - {service.Addresses.First()}:{service.Port}", ToastDuration.Long).Show();
            });
        }
    }
}
