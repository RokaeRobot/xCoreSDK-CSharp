using rokae.clr;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace xCoreSDK_CSharp
{
    internal class PrintHelper
    {
        public static void printArray(double val)
        {
            Console.Write("{0:0.000} ", val);
        }

        static public Action<double> print = new Action<double>(PrintHelper.printArray);

        static public void checkError(string content, ErrorCode ec)
        {
            if (ec.value != 0)
            {
                Console.WriteLine($"{content}:{ec.message}");
            }
        }
    }
}
