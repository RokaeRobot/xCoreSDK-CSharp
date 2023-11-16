using System;
using System.Collections.Generic;
using System.Data.SqlTypes;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.ExceptionServices;
using System.Threading;
using rokae.clr;
using EventInfos = System.Collections.Generic.Dictionary<System.String, System.Object>;
namespace xCoreSDK_CSharp
{
    internal class Program
    {
        // 协作6轴机器人
        static xMateRobotDemo xmate = new xMateRobotDemo();
        // 工业4轴机器人
        static PCB4RobotDemo pcb4 = new PCB4RobotDemo();
        // 协作7轴机器人
        static xMateErProRobotDemo xmatepro = new xMateErProRobotDemo();
        
        static void Main(string[] args)
        {
            try
            {
                // might throw
                xmate.connect("192.168.21.10");
            } catch (Exception e)
            {
                Console.WriteLine(e.ToString());
                return;
            }

            // 执行示例
            xmate.Example_OtherOperations();
           
            Console.ReadKey(true);
            Console.WriteLine(" --- BYE --- ");
        }
    }
}
