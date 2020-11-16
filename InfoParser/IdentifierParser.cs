using System;
using System.Collections.Generic;
using System.Text;

namespace RobotRaconteur.Companion.InfoParser
{
    public class YamlIdentifier
    {
        public string name { get; set; }
        public YamlUuid uuid { get; set; }

        public static explicit operator YamlIdentifier(string s) => new YamlIdentifier() { name = s };

        public void CopyTo(com.robotraconteur.identifier.Identifier id)
        {
            id.name = name;
            if (uuid == null)
            {
                id.uuid = new com.robotraconteur.uuid.UUID();
                id.uuid.uuid_bytes = new byte[16];
            }
            else
            {
                id.uuid = uuid.ToRRInfo();
            }
        }

        public com.robotraconteur.identifier.Identifier ToRRInfo()
        {
            var id = new com.robotraconteur.identifier.Identifier();
            CopyTo(id);
            return id;
        }
    }
}
