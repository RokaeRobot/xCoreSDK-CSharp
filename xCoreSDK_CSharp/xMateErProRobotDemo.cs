using rokae.clr;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using EventInfos = System.Collections.Generic.Dictionary<System.String, System.Object>;

namespace xCoreSDK_CSharp
{
    internal class xMateErProRobotDemo
    {
        /// <summary>
        /// 协作7轴机型
        /// </summary>
        xMateErProRobot robot = new xMateErProRobot();

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
    }
}
