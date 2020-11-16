using com.robotraconteur.uuid;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;

namespace RobotRaconteur.Companion.Util
{
    public static class UuidUtil
    {
        public static bool TryParse(string stringid, out byte[] bytes)
        {
            if (stringid == "{0}")
            {
                bytes = new byte[16];
                return true;
            }

            bytes = null;
            Regex r = new Regex(@"\{?([a-fA-F0-9]{8})-([a-fA-F0-9]{4})-([a-fA-F0-9]{4})-([a-fA-F0-9]{4})-([a-fA-F0-9]{12})\}?");
            var res = r.Match(stringid);
            if (!res.Success) return false;
            string res1 = "";
            for (int i = 1; i < 6; i++) res1 += res.Groups[i].Value;
            bytes = new byte[16];
            for (int i = 0; i < 16; i++)
            {
                bytes[i] = Convert.ToByte(res1.Substring(i * 2, 2), 16);
            }
            return true;
        }

        public static string UuidToString(UUID uuid)
        {
            string[] hexvals = uuid.uuid_bytes.Select(x => String.Format("{0:x2}", x)).ToArray();
            string g1 = String.Join("", hexvals, 0, 4);
            string g2 = String.Join("", hexvals, 4, 2);
            string g3 = String.Join("", hexvals, 6, 2);
            string g4 = String.Join("", hexvals, 8, 2);
            string g5 = String.Join("", hexvals, 10, 6);

            return String.Join("-", new string[] { g1, g2, g3, g4, g5 });
        }
    }
}
