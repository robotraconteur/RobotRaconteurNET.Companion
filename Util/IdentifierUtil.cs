using com.robotraconteur.identifier;
using com.robotraconteur.uuid;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;

namespace RobotRaconteur.Companion.Util
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

        public static string IdentifierToString(Identifier id)
        {
            if (id == null)
            {
                return null;
            }
            if (!IsIdentifierAnyName(id) && !IsIdentifierAnyUuid(id))
            {
                return id.name + "|" + UuidUtil.UuidToString(id.uuid);
            }
            if (!IsIdentifierAnyName(id))
            {
                return id.name;
            }
            if (!IsIdentifierAnyUuid(id))
            {
                return UuidUtil.UuidToString(id.uuid);
            }
            return null;
        }

        public static Identifier StringToIdentifier(string string_id)
        {
            const string name_regex = "(?:[a-zA-Z](?:\\w*[a-zA-Z0-9])?)(?:\\.[a-zA-Z](?:\\w*[a-zA-Z0-9])?)+";
            const string uuid_regex = @"\{?([a-fA-F0-9]{8})-([a-fA-F0-9]{4})-([a-fA-F0-9]{4})-([a-fA-F0-9]{4})-([a-fA-F0-9]{12})\}?";
            string identifier_regex = $"(?{name_regex}\\|{uuid_regex})|({name_regex})|({uuid_regex})";
            var r = new Regex(identifier_regex);
            var res = r.Match(string_id);
            if (!res.Success)
            {
                throw new ArgumentException("Invalid Identifier string");
            }
            if (res.Groups[1].Success && res.Groups[2].Success)
            {
                return CreateIdentifier(res.Groups[1].Value, res.Groups[2].Value);
            }
            if (res.Groups[3].Success)
            {
                return CreateIdentifierFromName(res.Groups[3].Value);
            }
            if (res.Groups[4].Success)
            {
                return CreateIdentifier("", res.Groups[4].Value);
            }
            throw new ArgumentException("Invalid Identifier string");
        }
    }
}
