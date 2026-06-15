using rokae.clr;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using xCoreSDK_CSharp.examples;

namespace xCoreSDK_CSharp
{
    internal class PCB4RobotDemo: ExampleInterface
    {
        /// <summary>
        /// 工业4轴机型
        /// </summary>
        PCB4Robot robot = new PCB4Robot();

        BaseRobot ExampleInterface.RobotInst { get { return robot; } }
        Cobot ExampleInterface.CobotInst { get { return null; } }

        /// <summary>
        /// 等待运动指令执行结束
        /// </summary>
        /// <param name="cmdID"></param>
        /// <param name="index"></param>
        void waitForFinish(String cmdID, int index)
        {
            ErrorCode ec;
            while (true)
            {
                var info = robot.queryEventInfo(Event.moveExecution, out ec);
                var _cmdID = (String)info["cmdID"];
                var _idx = (int)info["wayPointIndex"];
                var _err = (ErrorCode)info["error"];
                if (_err.value != 0)
                {
                    Console.WriteLine($"指令{_cmdID}:{_idx} 错误: {_err.message}");
                    return;
                }

                if (cmdID == _cmdID && _idx == index)
                {
                    Console.WriteLine($"指令 {cmdID}:{index} 已完成");
                    return;
                }

                Thread.Sleep(200);

            }
        }

        /// <summary>
        /// 示例 - 连接到机器人
        /// </summary>
        /// <param name="remoteIP">机器人地址</param>
        public void connect(string remoteIP)
        {
            robot.connectToRobot(remoteIP);
        }

        /// <summary>
        /// 示例 - 读取和设置软限位
        /// </summary>
        public void example_SoftLimit()
        {
            ErrorCode ec;
            double[,] limits = new double[4, 2] { { double.MaxValue, double.MaxValue }, { double.MaxValue, double.MaxValue },{ double.MaxValue, double.MaxValue },
                { double.MaxValue, double.MaxValue }};
           
            // 当limits为double类型数据最大值且enable为false时，代表关闭软限位
            robot.setSoftLimit(false, limits, out ec);
            PrintHelper.checkError("设置软限位", ec);

            bool v = robot.getSoftLimit(ref limits, out ec);
            Console.WriteLine($"软限位已打开 {v}");
            
            foreach (double i in limits)
            {
                Console.Write($"{i} ");
            }
            PrintHelper.checkError("读取软限位", ec);
            // 示例：设置2轴上限为 1.1
            // limits[1,1] = 1.1;
            robot.setSoftLimit(true, limits, out ec);
            
        }

        /// <summary>
        /// 示例 - 获取基本信息
        /// </summary>
        public void Example_BasicInfo()
        {
            ErrorCode ec;
            var info = robot.robotInfo(out ec);
            Console.WriteLine($"控制器版本 {info.version}, 机器人型号 {info.type}");
            var pos = robot.cartPosture(CoordinateType.flangeInBase, out ec);
            PrintHelper.checkError("位姿", ec);
            Console.Write("当前位姿 [");
            Array.ForEach(pos.trans, PrintHelper.printArray);
            Array.ForEach(pos.rpy, PrintHelper.printArray);
            Console.WriteLine("]");
        }

        /// <summary>
        /// 示例 - 运动指令
        /// </summary>
        public void Example_Move()
        {
            ErrorCode ec;
            robot.setOperateMode(OperateMode.automatic, out ec);
            robot.setPowerState(true, out ec);

            string id = "";
            // 设置工具工件坐标系
            Toolset defaultToolset = new Toolset();
            robot.setToolset(defaultToolset, out ec);
            MoveCommand movej1
                = new MoveCommand(), movej2 = new MoveCommand();

            // MoveJ指令的目标点
            // MoveJ1 X-0.448, Y-0, Z-0.4115, A-3.14, B-0.047, C-3.14
            movej1.cartTarget.trans[0] = 0.448;
            movej1.cartTarget.trans[1] = 0.0;
            movej1.cartTarget.trans[2] = 0.4115;
            movej1.cartTarget.rpy[0] = Math.PI;
            movej1.cartTarget.rpy[1] = 0.047;
            movej1.cartTarget.rpy[2] = Math.PI;

            // MoveJ2 X-0.502, Y-0, Z-0.63, A-3.14, B-1.416, C-3.14
            movej2.cartTarget.trans[0] = 0.502;
            movej2.cartTarget.trans[1] = 0.0;
            movej2.cartTarget.trans[2] = 0.63;
            movej2.cartTarget.rpy[0] = Math.PI;
            movej2.cartTarget.rpy[1] = 1.4167484;
            movej2.cartTarget.rpy[2] = Math.PI;
            // 沿外部工件坐标系X+偏移0.01
            movej2.cartTargetOffset.type = CartesianPosition.Offset.Type.offs;
            movej2.cartTargetOffset.frame.trans[0] = 0.01;

            var cmds = new List<MoveCommand> { movej1, movej2 };
            robot.moveAppend(MoveCommand.Type.MoveJ, cmds, ref id, out ec);
            robot.moveStart(out ec);
            waitForFinish(id, cmds.Count - 1);
        }

        string ExampleInterface.Demo_JointMove()
        {
            ErrorCode ec;

            MoveCommand p0 = new MoveCommand()
            {
                jointTarget = new JointPosition()
                {
                    joints = new List<double> { 0, 0, 0, 0}
                }
            }, p1 = new MoveCommand()
            {
                jointTarget = new JointPosition()
                {
                    joints = new List<double> { -0.5667424136157343, 0.90671615744485567, 1.2157964616589836, -0.79241537135616735 }
                }
            };
            string id = "";
            robot.moveAppend(MoveCommand.Type.MoveAbsJ, new List<MoveCommand> { p0, p1, p0, p1 }, ref id, out ec);


            robot.moveStart(out ec);
            if (ec.value == 0)
            {
                return "";
            }
            return ec.message;
        }
        string ExampleInterface.Demo_StartRecordingPos()
        {
            return "该机型不支持";
        }

        string ExampleInterface.Demo_StopRecordingPos()
        {
            return "该机型不支持";
        }
    }
}
