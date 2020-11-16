using System;
using System.Collections.Generic;
using System.Text;

namespace RobotRaconteur.Companion.InfoParser
{
    public class YamlResourceIdentifier
    {
        public YamlIdentifier bucket { get; set; }
        public string key { get; set; }

        public void CopyTo(com.robotraconteur.resource.ResourceIdentifier id)
        {
            id.bucket = bucket?.ToRRInfo();
            id.key = key ?? "";
        }

        public com.robotraconteur.resource.ResourceIdentifier ToRRInfo()
        {
            var id = new com.robotraconteur.resource.ResourceIdentifier();
            CopyTo(id);
            return id;
        }
    }
}
