using rokae.clr;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using xCoreSDK_CSharp.examples;
using EventInfos = System.Collections.Generic.Dictionary<System.String, System.Object>;

namespace xCoreSDK_CSharp
{
    internal class xMateErProRobotDemo: ExampleInterface
    {
        /// <summary>
        /// 协作7轴机型
        /// </summary>
        xMateErProRobot robot = new xMateErProRobot();
        BaseRobot ExampleInterface.RobotInst { get { return robot; } }
        Cobot ExampleInterface.CobotInst { get { return robot; } }

        CancellationTokenSource cts = new CancellationTokenSource();

        string keyJntVel = "dq_m"; // 轴速度
        string keyJntTorque = "tau_m"; // 关节力矩
        string keyExJntVel = "ex_dq_m"; // 外部轴速度

        /// <summary>
        /// 等待运动结束 - 通过查询机械臂是否在运动的方式
        /// </summary>
        void waitRobot()
        {
            bool moving = true;
            while (moving)
            {
                Thread.Sleep(300);
                ErrorCode ec;
                OperationState st = robot.operationState(out ec);
                if (st == OperationState.idle || st == OperationState.unknown)
                {
                    moving = false;
                }
            }

        }

        /// <summary>
        /// 事件处理 - 模拟发生碰撞后等待5秒上电并继续运行
        /// </summary>
        /// <param name="safetyInfo">反馈的信息</param>
        void RecoverFromCollision(EventInfos safetyInfo)
        {
            ErrorCode ec;
            var isCollided = (Boolean)safetyInfo["collided"]; // 指令ID
            if (isCollided)
            {
                Console.WriteLine("发生碰撞");
                Thread.Sleep(5000);
                robot.setPowerState(true, out ec);
                robot.moveStart(out ec);
                Console.WriteLine("已上电并继续运动");
            }
            else
            {
                Console.WriteLine("状态已恢复");
            }

        }

        /// <summary>
        /// 等待运动结束 - 通过查询执行信息
        /// </summary>
        /// <param name="cmdID">指令ID</param>
        /// <param name="index">最后一个目标点的下标</param>
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
        /// 示例：七轴冗余运动，及发生碰撞检测后恢复运动, 点位适用机型xMateER3 Pro
        /// </summary>
        public void Example_RedundantMove()
        {
            ErrorCode ec;
            string id = "";

            // 本段示例使用默认工具工件, 速度v500, 转弯区z10
            Toolset defaultToolset = new Toolset();
            robot.setToolset(defaultToolset, out ec);
            robot.setDefaultSpeed(500, out ec);
            robot.setDefaultZone(10, out ec);

            // 可选: 设置碰撞检测事件回调函数
            robot.setEventWatcher(Event.safety, RecoverFromCollision, out ec);

            robot.setOperateMode(OperateMode.manual, out ec);
            robot.setPowerState(true, out ec);

            MoveCommand moveAbsj = new MoveCommand(),
                moveL1 = new MoveCommand(), moveL2 = new MoveCommand(), moveL3 = new MoveCommand(),
                moveC1 = new MoveCommand(), moveC2 = new MoveCommand();

            // MoveAbsJ运动到拖拽位姿
            moveAbsj.jointTarget.joints.Add(0);
            moveAbsj.jointTarget.joints.Add(Math.PI / 6);
            moveAbsj.jointTarget.joints.Add(0);
            moveAbsj.jointTarget.joints.Add(Math.PI / 3);
            moveAbsj.jointTarget.joints.Add(0);
            moveAbsj.jointTarget.joints.Add(Math.PI / 2);
            moveAbsj.jointTarget.joints.Add(0);

            // ** 1) 变臂角运动 **
            moveL1.cartTarget.trans[0] = 0.562;
            moveL1.cartTarget.trans[1] = 0;
            moveL1.cartTarget.trans[2] = 0.432;
            moveL1.cartTarget.rpy[0] = Math.PI;
            moveL1.cartTarget.rpy[1] = 0;
            moveL1.cartTarget.rpy[2] = -Math.PI;
            moveL2.cartTarget.trans = moveL1.cartTarget.trans;
            moveL2.cartTarget.rpy = moveL1.cartTarget.rpy;

            moveL1.cartTarget.elbow = 1.45;
            moveL2.cartTarget.elbow = -1.51;
            var moveLCmds = new List<MoveCommand> { moveL1, moveL2 };


            robot.moveAppend(MoveCommand.Type.MoveAbsJ, new List<MoveCommand>{ moveAbsj}, ref id, out ec);
            robot.moveAppend(MoveCommand.Type.MoveL, moveLCmds, ref id, out ec);
            
        
            robot.moveStart(out ec);
            waitForFinish(id, moveLCmds.Count - 1);

            // ** 2) 60°臂角圆弧 **
            moveL3.cartTarget.trans[0] = 0.472;
            moveL3.cartTarget.trans[1] = 0;
            moveL3.cartTarget.trans[2] = 0.342;
            moveL3.cartTarget.rpy[0] = Math.PI;
            moveL3.cartTarget.rpy[1] = 0;
            moveL3.cartTarget.rpy[2] = -Math.PI;
            // 臂角都是60°
            moveL3.cartTarget.elbow = Math.PI / 3;

            moveC1.cartTarget.trans[0] = 0.602;
            moveC1.cartTarget.trans[1] = 0;
            moveC1.cartTarget.trans[2] = 0.342;
            moveC1.cartTarget.rpy[0] = Math.PI;
            moveC1.cartTarget.rpy[1] = 0;
            moveC1.cartTarget.rpy[2] = -Math.PI;
            moveC1.cartTarget.elbow = Math.PI / 3;
            moveC1.auxPoint.trans[0] = 0.537;
            moveC1.auxPoint.trans[1] = 0.065;
            moveC1.auxPoint.trans[2] = 0.342;
            moveC1.auxPoint.rpy[0] = Math.PI;
            moveC1.auxPoint.rpy[1] = 0;
            moveC1.auxPoint.rpy[2] = -Math.PI;
            moveC1.auxPoint.elbow = Math.PI / 3;

            moveC2.cartTarget = moveL3.cartTarget;
            moveC2.auxPoint.trans[0] = 0.537;
            moveC2.auxPoint.trans[1] = -0.065;
            moveC2.auxPoint.trans[2] = 0.342;
            moveC2.auxPoint.rpy[0] = Math.PI;
            moveC2.auxPoint.rpy[1] = 0;
            moveC2.auxPoint.rpy[2] = -Math.PI;
            moveC2.auxPoint.elbow = Math.PI / 3;
            var moveCCmds = new List<MoveCommand> { moveC1, moveC2 };

            moveLCmds.Clear();
            moveLCmds.Add(moveL3);
            robot.moveAppend(MoveCommand.Type.MoveL, moveLCmds, ref id, out ec);
            robot.moveAppend(MoveCommand.Type.MoveC, moveCCmds, ref id, out ec);
            robot.moveStart(out ec);
            waitForFinish(id, moveCCmds.Count - 1);
        }

        /// <summary>
        /// 示例 - 轴空间力控
        /// </summary>
        public void Example_ForceControl_JointSpace()
        {
            ErrorCode ec;

            // 自动模式下上电
            robot.setOperateMode(OperateMode.automatic, out ec);
            robot.setPowerState(true, out ec);

            // 先运动到拖拽位姿
            MoveCommand drag = new MoveCommand();
            drag.jointTarget.joints = new List<double> { 0, Math.PI / 6, 0, Math.PI / 3, 0, Math.PI / 2, 0 };
            String cmd_id = "";
            robot.moveAppend(MoveCommand.Type.MoveAbsJ, new List<MoveCommand> { drag }, ref cmd_id, out ec);
            robot.moveStart(out ec);
            waitRobot();

            var fc = robot.forceControl();
            fc.fcInit(FrameType.baseFrame, out ec);
            // 设置力控类型, 0代表轴空间
            fc.setControlType(0, out ec);
            // 设置各轴刚度。2轴4轴7轴小阻抗，其余轴阻抗较大
            double[] stiffness = new double[] { 1000, 10, 1000, 5, 50, 50, 0};
            fc.setJointStiffness(stiffness, out ec);
            Console.WriteLine("开始关节模式力控, 10秒后结束");
            fc.fcStart(out ec);

            // 设置期望力
            double[] desired = new double[] { 1, 1, 3, 0, 0, 0, 0 };
            fc.setJointDesiredTorque(desired, out ec);

            // 等待10s
            Thread.Sleep(10000);

            fc.fcStop(out ec);
        }

        string ExampleInterface.Demo_JointMove()
        {
            ErrorCode ec;

            MoveCommand p0 = new MoveCommand()
            {
                jointTarget = new JointPosition()
                {
                    joints = new List<double> { 0, 0, 0, 0, 0, 0, 0 },
                    external = new List<double> { -0.2 }
                }
            }, p1 = new MoveCommand()
            {
                jointTarget = new JointPosition()
                {
                    joints = new List<double> { 0, 30.0 / 180.0 * Math.PI, 0, 60.0 / 180.0 * Math.PI, 0, 60.0 / 180.0 * Math.PI, 0 },
                    external = new List<double> { 0.4 }
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

        private void readStateDate()
        {
            double[] jntVel = new double[7], jntTorque = new double[6], exJntVel = new double[6];

            // Set a variable to the Desktop path.
            string filePath =
              Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            DateTime currentTime = DateTime.Now;
            // Write to a new file named "recordData_HHmmss.csv".
            using (StreamWriter outputFile = new StreamWriter(Path.Combine(filePath, $"recordData_{currentTime.Hour}{currentTime.Minute}{currentTime.Second}.csv")))
            {
                outputFile.WriteLine("J1,J2,J3,J4,J5,J6,J7,," +
                        "Tau1,Tau2,Tau3,Tau4,Tau5,Tau6,Tau7,," +
                        "EJ");
                do
                {
                    // 每周期获取数据前，需要更新一次数据
                    // startReceiveRobotState设置的发送周期是8ms,所以这里更新数据的等待时间也是8ms
                    robot.updateRobotState(8);

                    // 按照数据名称查数据
                    robot.getStateData(keyJntVel, ref jntVel);
                    robot.getStateData(keyExJntVel, ref exJntVel);
                    robot.getStateData(keyJntTorque, ref jntTorque);
                    outputFile.WriteLine($"{jntVel[0]},{jntVel[1]},{jntVel[2]},{jntVel[3]},{jntVel[4]},{jntVel[5]},{jntVel[6]},," +
                        $"{jntTorque[0]},{jntTorque[1]},{jntTorque[2]},{jntTorque[3]},{jntTorque[4]},{jntTorque[5]},{jntTorque[6]},," +
                        $"{exJntVel[0]}");

                } while (!cts.Token.IsCancellationRequested);
            }

            Console.WriteLine("finish record");
        }

        string ExampleInterface.Demo_StartRecordingPos()
        {

            ErrorCode ec;

            try
            {
                // 开始接收数据，设置周期为1ms （允许的时长：1ms/2ms/4ms/8ms/1s）
                // 设置接收关节速度、外部轴速度，和关节力矩。所有支持的数据见getStateData()函数说明
                robot.startReceiveRobotState(8, new string[] { keyJntVel, keyExJntVel, keyJntTorque });
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

        string ExampleInterface.Demo_StopRecordingPos()
        {
            if (cts != null) { cts.Cancel(); }
            robot.stopReceiveRobotState();
            return "";
        }
    }
}
