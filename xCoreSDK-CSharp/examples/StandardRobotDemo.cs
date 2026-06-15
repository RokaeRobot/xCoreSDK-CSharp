using rokae.clr;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using xCoreSDK_CSharp.examples;

namespace xCoreSDK_CSharp
{
    internal class StandardRobotDemo: ExampleInterface
    {
        // 工业六轴机型
        private StandardRobot robot = new StandardRobot();

        CancellationTokenSource cts = new CancellationTokenSource();

        BaseRobot ExampleInterface.RobotInst { get { return robot; } }
        Cobot ExampleInterface.CobotInst { get { return null; } }

        /// <summary>
        /// 示例 - 连接到机器人
        /// </summary>
        /// <param name="remoteIP">机器人地址</param>
        public void connect(string remoteIP)
        {
            robot.connectToRobot(remoteIP);
        }

        public void Example_OtherOperation()
        {            
            ErrorCode ec;

            Frame new_bf = new Frame(robot.baseFrame(out ec));
            new_bf.trans[0] += 0.1;
            // 设置基坐标系为当前基坐标系X +0.1m
            // 基坐标系需要重启控制器后生效
            Console.WriteLine($"设置基坐标系为 {new_bf.trans[0]}, {new_bf.trans[1]}, " +
                $"{new_bf.trans[2]}, {new_bf.rpy[0]}, {new_bf.rpy[1]}, {new_bf.rpy[2]}");
            robot.setBaseFrame(new_bf, out ec);

            // 恢复设置
            Console.WriteLine("恢复基座标设置");
            new_bf.trans[0] -= 0.1;
            robot.setBaseFrame(new_bf, out ec);
        }

        private void readStateDate()
        {
            double[] jntPos = new double[6], pos = new double[6], exJnt = new double[6];
            UInt64 timestamp = 0;

            // Set a variable to the Desktop path.
            string filePath =
              Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            DateTime currentTime = DateTime.Now;
            // Write to a new file named "recordData_HHmmss.csv".
            using (StreamWriter outputFile = new StreamWriter(Path.Combine(filePath, $"recordData_{currentTime.Hour}{currentTime.Minute}{currentTime.Second}.csv")))
            {
                outputFile.WriteLine("Timestamp,,J1,J2,J3,J4,J5,J6,," +
                        "X,Y,Z,Rx,Ry,Rz,," +
                        "EJ");
                do
                {
                    // 每周期获取数据前，需要更新一次数据
                    // startReceiveRobotState设置的发送周期是1ms,所以这里更新数据的等待时间也是1ms
                    robot.updateRobotState(1);

                    // 按照数据名称查数据
                    int ret = robot.getStateData("q_m", ref jntPos);
                    robot.getStateData("pos_abc_m", ref pos);
                    robot.getStateData("ex_q_m", ref exJnt);
                    robot.getStateData("ts", ref timestamp);
                    outputFile.WriteLine($"'{timestamp},,{jntPos[0]},{jntPos[1]},{jntPos[2]},{jntPos[3]},{jntPos[4]},{jntPos[5]},," +
                        $"{pos[0]},{pos[1]},{pos[2]},{pos[3]},{pos[4]},{pos[5]},," +
                        $"{exJnt[0]}");

                } while (!cts.IsCancellationRequested);
            }

            Console.WriteLine("finish record");
        }


        string ExampleInterface.Demo_StartRecordingPos()
        {
            string keyJntPos = "q_m"; // 轴角度
            string posAbcKey = "pos_abc_m"; // 笛卡尔位姿，末端相对于外部工件坐标系
            string keyExJntPos = "ex_q_m"; // 外部轴数值

            ErrorCode ec;

            try
            {
                // 开始接收数据，设置周期为1ms （允许的时长：1ms/2ms/4ms/8ms/1s）
                // 设置接收轴角度、位姿和外部轴数据。所有支持的数据见getStateData()函数说明
                robot.startReceiveRobotState(1, new string[] { keyJntPos, posAbcKey, keyExJntPos });
            }
            catch (ApiException e)
            {
                // 异常情况：网络错误，一般由于本机地址错误或防火墙问题；
                return e.Message;
            }

            // 开启一个线程读数据
            cts.Dispose();
            cts = new CancellationTokenSource();
            Task.Factory.StartNew(readStateDate, cts.Token);
            cts.Token.Register(() =>
            {
                robot.stop(out ErrorCode ec);
            });
            return "";
        }

        /// <summary>
        /// 停止记录
        /// </summary>
        /// <returns></returns>
        string ExampleInterface.Demo_StopRecordingPos()
        {
            if (cts != null) { cts.Cancel(); }
            robot.stopReceiveRobotState();
            return "";
        }

        /// <summary>
        /// 运行测试运动指令
        /// </summary>
        /// <returns></returns>
        string ExampleInterface.Demo_JointMove()
        {
            ErrorCode ec;
            // 设置默认速度100
            robot.setDefaultSpeed(100, out ec);
            // 设置默认转弯区0
            robot.setDefaultZone(0, out ec);

            MoveCommand p0 = new MoveCommand()
            {
                jointTarget = new JointPosition()
                {
                    joints = new List<double> { 0, 0, 0, 0, 0, 0 }

                }
            }, p1 = new MoveCommand()
            {
                jointTarget = new JointPosition()
                {
                    joints = new List<double> { 0.0, -0.26179938779914941, 1.0471975511965976, 0.0, 0.78539816339744828, 0.0 }
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
    }
}
