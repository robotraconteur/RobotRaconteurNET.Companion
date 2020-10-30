// Copyright 2020 Rensselaer Polytechnic Institute
//                Wason Technology, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using System.Text;
using com.robotraconteur.device;
using com.robotraconteur.robotics.robot;
using com.robotraconteur.uuid;
using com.robotraconteur.geometry;
using System.IO;
using System.Linq;
using com.robotraconteur.robotics.tool;
using com.robotraconteur.sensor;

namespace RobotRaconteur.Companion.InfoParser
{
    public class YamlToolInfo
    {
        public YamlDeviceInfo device_info { get; set; }
        public ToolTypeCode tool_type { get; set; }
        public List<com.robotraconteur.robotics.tool.ToolCapabilities> tool_capabilities { get; set; }
        public YamlTransform tcp { get; set; }
        public YamlSpatialInertia inertia { get; set; }
        public double actuation_time { get; set; }
        public double close_position { get; set; }
        public double open_position { get; set; }
        public double command_min { get; set; }
        public double command_max { get; set; }
        public double command_open { get; set; }
        public double command_close { get; set; }
        public List<SensorTypeCode> sensor_type { get; set; }
        public double[] sensor_min { get; set; }
        public double[] sensor_max { get; set; }

        public Dictionary<string, YamlVarValue> extended { get; set; }

        public void CopyTo(com.robotraconteur.robotics.tool.ToolInfo info)
        {
            info.device_info = device_info?.ToRRInfo();
            info.tool_type = tool_type;
            info.tool_capabilities = (uint)(tool_capabilities?.Aggregate((x, y) => x | y) ?? 0);
            info.tcp = tcp?.ToRRInfo() ?? new Transform();
            info.inertia = inertia?.ToRRInfo() ?? new SpatialInertia();
            info.actuation_time = actuation_time;
            info.close_position = close_position;
            info.open_position = open_position;
            info.command_min = command_min;
            info.command_max = command_max;
            info.command_close = command_close;
            info.command_open = command_open;
            info.sensor_type = sensor_type;
            info.sensor_min = sensor_min ?? new double[0];
            info.sensor_max = sensor_max ?? new double[0];
            info.extended = extended?.ToDictionary(x => x.Key, x => x.Value?.value);
        }

        public com.robotraconteur.robotics.tool.ToolInfo ToRRInfo()
        {
            var info = new com.robotraconteur.robotics.tool.ToolInfo();
            CopyTo(info);
            return info;
        }
    }

    public static class ToolInfoParser
    {
        public static Tuple<ToolInfo, LocalIdentifierLocks> LoadToolInfoYamlWithIdentifierLocks(string filename)
        {
            using (var f = new StreamReader(filename))
            {
                var deserializer = new YamlDotNet.Serialization.Deserializer();
                var yaml_tool_info = deserializer.Deserialize<YamlToolInfo>(f);
                return LoadToolInfoYamlWithIdentifierLocks(yaml_tool_info);
            }
        }

        public static Tuple<ToolInfo, LocalIdentifierLocks> ParseToolInfoYamlWithIdentifierLocks(string data)
        {
            using (var f = new StringReader(data))
            {
                var deserializer = new YamlDotNet.Serialization.Deserializer();
                var yaml_tool_info = deserializer.Deserialize<YamlToolInfo>(f);
                return LoadToolInfoYamlWithIdentifierLocks(yaml_tool_info);
            }
        }


        public static Tuple<ToolInfo, LocalIdentifierLocks> LoadToolInfoYamlWithIdentifierLocks(YamlToolInfo yaml_tool_info)
        {
            var tool_info = yaml_tool_info.ToRRInfo();
            LocalIdentifierLocks ident_locks = null;
            if (tool_info.device_info != null && tool_info.device_info.device != null
                && !String.IsNullOrEmpty(tool_info.device_info.device.name)
                && tool_info.device_info.device.uuid.uuid_bytes.All(x => x == 0))
            {
                var l = LocalIdentifiersManager.GetIdentifierForNameAndLock("tool", tool_info.device_info.device.name);
                tool_info.device_info.device.uuid = l.Identifier.uuid;
                ident_locks = new LocalIdentifierLocks(l);
            }
            return Tuple.Create<ToolInfo, LocalIdentifierLocks>(tool_info, ident_locks);
        }
    }
}
