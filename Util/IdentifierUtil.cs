using com.robotraconteur.identifier;
using com.robotraconteur.uuid;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotRaconteurNET.Companion.Util
{
    public static class IdentifierUtil
    {
        public static bool IsIdentifierAnyUuid(Identifier identifier)
        {
            if (identifier == null)
            {
                return true;
            }
            foreach (var b in identifier.uuid.uuid_bytes)
            {
                if (b != 0)
                {
                    return false;
                }                
            }
            return true;
        }

        public static bool IsIdentifierAnyName(Identifier identifier)
        {
            if (identifier == null)
            {
                return true;
            }
            if (String.IsNullOrEmpty(identifier.name))
            {
                return true;
            }
            return false;
        }

        public static bool IsIdentifierAny(Identifier identifier)
        {
            if (identifier == null)
            {
                return true;
            }

            if (!IsIdentifierAnyUuid(identifier))
            {
                return false;
            }
            if (!IsIdentifierAnyName(identifier))
            {
                return false;
            }
            return true;
        }

        public static bool IsIdentifierMatch(Identifier expected, Identifier test)
        {
            if (IsIdentifierAny(expected) || IsIdentifierAny(test))
            {
                return true;
            }

            bool name_match = false;
            bool uuid_match = false;

            if (IsIdentifierAnyName(expected) || IsIdentifierAnyName(test))
            {
                name_match = true;
            }
            else
            {
                if (expected.name == test.name)
                {
                    name_match = true;
                }
            }

            if (IsIdentifierAnyUuid(expected) || IsIdentifierAnyUuid(test))
            {
                uuid_match = true;
            }
            else
            {
                if (Enumerable.SequenceEqual(expected.uuid.uuid_bytes, test.uuid.uuid_bytes))
                {
                    uuid_match = true;
                }
            }

            return name_match && uuid_match;
        }

        public static Identifier CreateIdentifier(string name, string uuid)
        {
             if(!UuidUtil.TryParse(uuid, out var uuid_r))
             {
                 throw new ArgumentException("Invalid UUID format, could not parse");
             }
            var ret = new Identifier()
            {
                name = name,
                uuid = new UUID { uuid_bytes = uuid_r }
            };
            return ret;
        }

        public static Identifier CreateIdentifierFromName(string name)
        {
            return new Identifier()
            {
                name = name,
                uuid = new UUID { uuid_bytes = new byte[16] }
            };
        }
    }
}
