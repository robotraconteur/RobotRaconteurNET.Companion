using com.robotraconteur.device;
using com.robotraconteur.sensordata;
using System;
using System.Collections.Generic;
using System.Text;

namespace RobotRaconteur.Companion.Util
{
    public static class SensorDataUtil
    {
        public static SensorDataHeader FillSensorDataHeader(RobotRaconteurNode node, DeviceInfo device_info, ulong seqno)
        {
            var ret = new SensorDataHeader();
            ret.seqno = seqno;
            ret.ts = DateTimeUtil.TimeSpec2Now(node,device_info);         
            return ret;
        }
    }
}
