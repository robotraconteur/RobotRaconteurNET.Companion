using com.robotraconteur.device;
using com.robotraconteur.identifier;
using System;
using System.Collections.Generic;
using System.Diagnostics.Tracing;
using System.Text;

namespace RobotRaconteur.Companion.Util
{
    public static class AttributesUtil
    {
        private static bool TryAddIdentifier(Dictionary<string, object> o, string name, Identifier id)
        {
            if (!IdentifierUtil.IsIdentifierAny(id))
            {
                o.Add(name, IdentifierUtil.IdentifierToString(id));
                return true;
            }
            return false;
        }

        private static bool TryAddString(Dictionary<string, object> o, string name, string str)
        {
            if (!String.IsNullOrEmpty(str))
            { 
                o.Add(name, str);
                return true;
            }
            return false;
        }

        public static Dictionary<string,object> GetDefaultServiceAtributesFromDeviceInfo(DeviceInfo info)
        {
            var o = new Dictionary<string, object>();
            TryAddIdentifier(o,"device", info.device);
            TryAddIdentifier(o, "parent_device", info.parent_device);
            TryAddIdentifier(o, "manufacturer", info.manufacturer);
            TryAddIdentifier(o, "model", info.model);
            TryAddString(o, "serial_number", info.serial_number);
            TryAddString(o, "user_description", info.user_description);
            List<Identifier> tags = info?.extended?.ContainsKey("tags") ?? false ? (List<Identifier>)info.extended["tags"] : null;
            if (tags != null)
            {
                o.Add("tags", string.Join(",",tags.ConvertAll(x => IdentifierUtil.IdentifierToString(x))));
            }
            return o;
        }
    }
}
