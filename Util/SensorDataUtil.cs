using com.robotraconteur.device;
using com.robotraconteur.sensordata;
using System;
using System.Collections.Generic;
using System.Text;

namespace RobotRaconteur.Companion.Util
{
    public static class SensorDataUtil
    {
        public static SensorDataHeader FillSensorDataHeader(DeviceInfo device_info, ulong seqno)
        {
            var ret = new SensorDataHeader();
            ret.seqno = seqno;
            ret.ts = DateTimeUtil.UtcNow;            
            if (device_info != null && device_info.device != null)
            {
                var source_info = new SensorDataSourceInfo();
                source_info.source = device_info.device;
                source_info.source_config_nonce = "";
                ret.source_info = source_info;               
            }
            return ret;
        }
    }
}
