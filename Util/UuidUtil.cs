using System;
using System.Collections.Generic;
using System.Text;
using System.Text.RegularExpressions;

namespace RobotRaconteurNET.Companion.Util
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
    }
}
