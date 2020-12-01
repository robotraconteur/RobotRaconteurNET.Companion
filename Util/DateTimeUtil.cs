using com.robotraconteur.device;
using com.robotraconteur.device.clock;
using System;
using System.Collections.Generic;
using System.Text;

namespace RobotRaconteur.Companion.Util
{
    public static class DateTimeUtil
    {
        public static com.robotraconteur.datetime.DateTimeUTC UtcNow(RobotRaconteurNode node = null, DeviceInfo info = null)
        {
            if (node == null)
            {
                node = RobotRaconteurNode.s;
            }
            var now = node.NowUTC;
            var epoch_now = now - (new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc));
            var ret = new com.robotraconteur.datetime.DateTimeUTC();
            ret.seconds = (long)Math.Floor(epoch_now.TotalSeconds);
            ret.nanoseconds = (int)((epoch_now - TimeSpan.FromSeconds(ret.seconds)).Ticks * ((long)1e6 / TimeSpan.TicksPerMillisecond));
            var clock_info = new com.robotraconteur.datetime.ClockInfo();
            clock_info.clock_type = (int)com.robotraconteur.datetime.ClockTypeCode.default_;
            var device_uuid_bytes = info?.device?.uuid.uuid_bytes;
            clock_info.clock_uuid.uuid_bytes = device_uuid_bytes ?? new byte[16];
            ret.clock_info = clock_info;
            return ret;            
        }

        public static com.robotraconteur.datetime.DateTimeLocal LocalNow(RobotRaconteurNode node = null, DeviceInfo info = null)
        {
            if (node == null)
            {
                node = RobotRaconteurNode.s;
            }
            var now = node.NowUTC;
            var epoch_now = now - (new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc));
            var ret = new com.robotraconteur.datetime.DateTimeLocal();
            ret.seconds = (long)Math.Floor(epoch_now.TotalSeconds);
            ret.nanoseconds = (int)((epoch_now - TimeSpan.FromSeconds(ret.seconds)).Ticks * ((long)1e6 / TimeSpan.TicksPerMillisecond));
            var utc_offset = TimeZoneInfo.Local.GetUtcOffset(now);
            ret.seconds += (long)utc_offset.TotalSeconds;
            var clock_info = new com.robotraconteur.datetime.ClockInfo();
            clock_info.clock_type = (int)com.robotraconteur.datetime.ClockTypeCode.default_;
            var device_uuid_bytes = info?.device?.uuid.uuid_bytes;
            clock_info.clock_uuid.uuid_bytes = device_uuid_bytes ?? new byte[16];
            ret.clock_info = clock_info;
            ret.utc_offset_seconds = (int)utc_offset.TotalSeconds;
            ret.timezone_name = TimeZoneInfo.Local.Id;
            return ret;
        }

        public static com.robotraconteur.datetime.TimeSpec2 TimeSpec2Now(RobotRaconteurNode node = null, DeviceInfo info = null)
        {
            if (node == null)
            {
                node = RobotRaconteurNode.s;
            }
            var now = node.NowTimeSpec;
            var ret = new com.robotraconteur.datetime.TimeSpec2();
            ret.seconds = now.seconds;
            ret.nanoseconds = now.nanoseconds;
            var clock_info = new com.robotraconteur.datetime.ClockInfo();
            clock_info.clock_type = (int)com.robotraconteur.datetime.ClockTypeCode.node_default;
            var device_uuid_bytes = info?.device?.uuid.uuid_bytes;
            clock_info.clock_uuid.uuid_bytes = device_uuid_bytes ?? new byte[16];
            ret.clock_info = clock_info;
            return ret;
        }

        public static com.robotraconteur.datetime.TimeSpec3 TimeSpec3Now(RobotRaconteurNode node = null)
        {
            if (node == null)
            {
                node = RobotRaconteurNode.s;
            }
            var now = node.NowTimeSpec;
            var ret = new com.robotraconteur.datetime.TimeSpec3();
            ret.microseconds = now.seconds*1000000 + now.nanoseconds/1000;
            return ret;
        }

        public static DeviceTime FillDeviceTime(RobotRaconteurNode node, DeviceInfo device_info, ulong seqno)
        {
            var ret = new DeviceTime();
            ret.device_seqno = seqno;
            ret.device_ts = TimeSpec2Now(node ,device_info);
            ret.device_utc = UtcNow(node, device_info);            
            return ret;
        }


    }
}
