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
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using com.robotraconteur.identifier;
using com.robotraconteur.uuid;
using RobotRaconteur;
using RobotRaconteur.Companion.Util;

namespace RobotRaconteur.Companion.Util
{
    // This file is largely based on RobotRaconteurWeb.LocalTransport NodeID generation and locking


    /// <summary>
    /// Utility class to generate a local UUID for an identifier name
    /// and persist the generated UUID
    /// </summary>
    public static class LocalIdentifiersManager
    {
        public static  LocalIdentifierLock GetIdentifierForNameAndLock(string category, string name)
        {
            NodeID nodeid = null;

            if (!Regex.IsMatch(name, "^[a-zA-Z][a-zA-Z0-9_\\.\\-]*$"))
            {
                throw new ArgumentException("\"" + name + "\" is an invalid identifier name");
            }

            var node_dirs = RobotRaconteurNode.s.GetNodeDirectories();

            // Same scope as the C++ and Python companions: identifiers/<category>/<name>
            var categories = new vectorstring();
            categories.Add("identifiers");
            categories.Add(category);

            // Migrate a UUID stored by older versions of this library under <category>/<name>
            // (without the "identifiers" prefix) so existing devices keep their identifier
            MigrateLegacyIdentifier(node_dirs, category, name);

            var p = RobotRaconteurNET.GetUuidForNameAndLock(node_dirs, name, categories);

            var ret_id = IdentifierUtil.CreateIdentifier(name, p.uuid.ToString("D"));

            return new LocalIdentifierLock(ret_id, p.fd);            
        }

        static void MigrateLegacyIdentifier(NodeDirectories node_dirs, string category, string name)
        {
            try
            {
                string config_dir = node_dirs.user_config_dir.string_();
                string new_path = Path.Combine(config_dir, "identifiers", category, name);
                string legacy_path = Path.Combine(config_dir, category, name);
                if (File.Exists(new_path) || !File.Exists(legacy_path))
                {
                    return;
                }
                Directory.CreateDirectory(Path.GetDirectoryName(new_path));
                File.Copy(legacy_path, new_path, false);
            }
            catch (IOException)
            {
                // Best effort: if the legacy identifier cannot be migrated, a new UUID is generated
            }
            catch (UnauthorizedAccessException)
            {
            }
        }
    }

    public class LocalIdentifierLock : IDisposable
    {
        internal LocalIdentifierLock(Identifier id, NodeDirectoriesFD fd)
        {
            this.fd = fd;
            this.Identifier = id;
        }

        NodeDirectoriesFD fd;
        public Identifier Identifier { get; }

        public void Dispose()
        {
            fd?.Dispose();
        }
    }

    public class LocalIdentifierLocks : IDisposable
    {
        public LocalIdentifierLock[] IdentifierLocks { get; private set; }

        public LocalIdentifierLocks(LocalIdentifierLock[] locks)
        {
            IdentifierLocks = locks;
        }

        public LocalIdentifierLocks(LocalIdentifierLock lock_)
        {
            IdentifierLocks = new LocalIdentifierLock[] { lock_ };
        }

        public void Dispose()
        {
            if (IdentifierLocks != null)
            {
                foreach (var d in IdentifierLocks)
                {
                    d?.Dispose();
                }
            }
        }

        public void Release()
        {
            IdentifierLocks = null;
        }
    }
}
