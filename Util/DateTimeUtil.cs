using System;
using System.Collections.Generic;
using System.Text;

namespace RobotRaconteur.Companion.Util
{
    public static class DateTimeUtil
    {
        public static com.robotraconteur.datetime.DateTimeUTC UtcNow
        {
            get
            {
                var t = DateTime.UtcNow - (new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc));
                var ret = new com.robotraconteur.datetime.DateTimeUTC();
                ret.seconds = (long)Math.Floor(t.TotalSeconds);
                ret.nanoseconds = (int)((t.TotalMilliseconds * 1e6) % 1e9);
                var clock_info = new com.robotraconteur.datetime.ClockInfo();
                clock_info.clock_type = (int)com.robotraconteur.datetime.ClockTypeCode.default_;
                clock_info.clock_uuid.uuid_bytes = new byte[16];
                ret.clock_info = clock_info;
                return ret;
            }
        }

        public static com.robotraconteur.datetime.DateTimeLocal LocalNow
        {
            get
            {
                var now = DateTime.Now;
                var utc_offset = TimeZoneInfo.Local.GetUtcOffset(now);
                var t = now - (new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)) + utc_offset;
                var ret = new com.robotraconteur.datetime.DateTimeLocal();
                ret.seconds = (long)Math.Round(t.TotalSeconds);
                ret.nanoseconds = (int)Math.IEEERemainder(t.TotalMilliseconds * 1e6, 1e9);
                var clock_info = new com.robotraconteur.datetime.ClockInfo();
                clock_info.clock_type = (int)com.robotraconteur.datetime.ClockTypeCode.default_;
                clock_info.clock_uuid.uuid_bytes = new byte[16];
                ret.clock_info = clock_info;
                ret.utc_offset_seconds = (int)utc_offset.TotalSeconds;
                ret.timezone_name = TimeZoneInfo.Local.Id;
                return ret;
            }
        }


    }
}
