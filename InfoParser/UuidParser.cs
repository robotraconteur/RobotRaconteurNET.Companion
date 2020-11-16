using RobotRaconteur.Companion.Util;
using System;
using System.Collections.Generic;
using System.Text;

namespace RobotRaconteur.Companion.InfoParser
{
    public class YamlUuid
    {
        public string source_string { get; set; }
        public byte[] uuid_bytes { get; set; } = new byte[16];

        public YamlUuid(string str_uuid)
        {
            source_string = str_uuid;
            if (!UuidUtil.TryParse(str_uuid, out byte[] uuid_bytes))
            {
                throw new ArgumentException("Invalid UUID string in yaml config file");
            }
            this.uuid_bytes = uuid_bytes;
        }




        public static explicit operator YamlUuid(string s) => new YamlUuid(s);

        public void CopyTo(ref com.robotraconteur.uuid.UUID uuid)
        {
            uuid.uuid_bytes = (byte[])uuid_bytes.Clone() ?? new byte[16];
        }

        public com.robotraconteur.uuid.UUID ToRRInfo()
        {
            var uuid = new com.robotraconteur.uuid.UUID();
            CopyTo(ref uuid);
            return uuid;
        }
    }
}
