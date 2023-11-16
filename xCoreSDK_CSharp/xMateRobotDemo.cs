using rokae.clr;
using System;
using System.Collections.Generic;
using System.Runtime.Serialization;
using System.Threading;
using EventInfos = System.Collections.Generic.Dictionary<System.String, System.Object>;

namespace xCoreSDK_CSharp
{
    internal class xMateRobotDemo
    {
        // 协作六轴机型
        // 方式一: 创建对象并连接到机器人
        // xMateRobot robot = new xMateRobot("192.168.0.160");
   
        // 方式二：default constructor，然后调用connectToRobot(remoteIP)
        private xMateRobot robot = new xMateRobot();

        // 协作七轴
        // xMateErProRobot robot = new xMateErProRobot();

        CancellationTokenSource cts = new CancellationTokenSource();

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
        /// 等待Jog运动停止
        /// </summary>
        void waitForJogStop()
        {
            bool moving = true;
            while (moving)
            {
                Thread.Sleep(300);
                ErrorCode ec;
                OperationState st = robot.operationState(out ec);
                //Console.Write($"{st.ToString()} ");
                // 如果Jog在运动中，则返回OperationState.jogging
                if (st == OperationState.jog || st == OperationState.idle)
                {
                    moving = false;
                }
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

        void printMoveExecutionInfo(EventInfos info)
        {
            var _cmdID = (String)info["cmdID"]; // 指令ID
            var _idx = (int)info["wayPointIndex"]; // 路点下标, 从0开始标号
            var _err = (ErrorCode)info["error"]; // 错误信息
            var _remark = (string)info["remark"]; // 警告信息（相近定位）

            if (_err.value != 0)
            {
                Console.WriteLine($"ID: {_cmdID}:{_idx} ,Err: {_err.value}:{_err.message}");
            }
            if(_remark.Length > 0)
            {
                Console.WriteLine($"Warning: {_remark}");
            }
            // 是否达到目标点
            if ((bool)info["reachTarget"])
            {
                Console.WriteLine($"ID: {_cmdID}:{_idx} reach target");
            }

        }

        /// <summary>
        /// 每2秒打印机器人末端位姿，轴角度，轴速度
        /// </summary>
        void printState(CancellationToken token)
        {
            ErrorCode ec;
            do
            {
                // 末端在外部参考坐标系中的位姿
                Console.Write("End in ref posture: ");
                Array.ForEach(robot.posture(CoordinateType.endInRef, out ec), PrintHelper.print);

                // 法兰在基坐标系中的位姿，包括轴配置数据
                var cartPos = robot.cartPosture(CoordinateType.flangeInBase, out ec);
                Console.Write("\nFlange in base posture: ");
                Array.ForEach(cartPos.trans, PrintHelper.print);
                Array.ForEach(cartPos.rpy, PrintHelper.print);
                Console.Write("\n Confdata: ");
                foreach (var d in cartPos.confData)
                {
                    Console.Write($"{d} ");
                }

                // 轴角度
                Console.Write("\nJoint positions - [");
                Array.ForEach(robot.jointPos(out ec), PrintHelper.print);
                // 轴速度
                Console.Write("]\nJoint velocity - [");
                Array.ForEach(robot.jointVel(out ec), PrintHelper.print);
                // 轴力矩
                Console.Write("]\nJoint Torque - [");
                Array.ForEach(robot.jointTorque(out ec), PrintHelper.print);
                Console.WriteLine("]\n");
                Thread.Sleep(2000);

            } while (!token.IsCancellationRequested);
        }

        /// <summary>
        /// 执行运动之前一些操作：
        /// 必要操作：切换到自动模式上电；
        /// 可选操作：清除缓存，设置默认工具工件，速度和转弯区
        /// </summary>
        /// <param name="default_speed">速度</param>
        /// <param name="default_zone">转弯区</param>
        private void Move_Preset(int default_speed, int default_zone)
        {
            ErrorCode ec;
            // 设置为自动模式上电
            robot.setOperateMode(OperateMode.automatic, out ec);
            // 上电
            robot.setPowerState(true, out ec);

            // 可选：设置接收运动执行信息回调函数
            robot.setEventWatcher(Event.moveExecution, printMoveExecutionInfo, out ec);

            // 执行运动指令之前可调用moveReset()重置轨迹缓存、和执行反馈信息
            robot.moveReset(out ec);
            // 设置默认速度。对于每个MoveCommnad, 若没有指定速度，则使用这个默认速度
            robot.setDefaultSpeed(default_speed, out ec);
            // 设置默认转弯区。对于每个MoveCommnad, 若没有指定转弯区，则使用默认转弯区
            robot.setDefaultZone(default_zone, out ec);

            // 设置工具组坐标。示例里点位都是不带工具工件的
            var defaultToolset = new Toolset();
            robot.setToolset(defaultToolset, out ec);

            // alternative interface: set toolset by names
            // 需要先加载对应工程
            // robot.setToolset("tool1", "wobj1", out ec);
            
        }

        public void connect(string remoteIP)
        {
            robot.connectToRobot(remoteIP);
        }

        public void disconnect()
        {
            ErrorCode ec;
            robot.disconnectFromRobot(out ec);
            PrintHelper.checkError("断开连接", ec);
        }

        /// <summary>
        /// 示例：各运动指令的调用（MoveJ/MoveL/MoveAbsJ/MoveC)，使用executeCommand()接口
        /// </summary>
        public void Example_Move1()
        {
            ErrorCode ec;
            Move_Preset(200, 50);

            Thread t = new Thread(new ThreadStart(() => printState(cts.Token)));
            t.Start();
            MoveCommand absj1 = new MoveCommand(), drag = new MoveCommand(),
                movel1 = new MoveCommand();

            // MoveAbsJ指令 - 轴运动, 目标点类型为关节
            absj1.jointTarget.joints = new List<double> { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
            drag.jointTarget.joints = new List<double> { 0, Math.PI / 6, -Math.PI / 2, 0, -Math.PI / 3, 0 };
            absj1.speed = 1000;
            drag.speed = 200;
            List<MoveCommand> cmds = new List<MoveCommand>();
            cmds.Add(absj1);
            cmds.Add(drag);

            // MoveL指令 - 末端运动，目标点类型为笛卡尔
            movel1.cartTarget.trans = new double[] { 0.558, -0.3388, 0.413 }; // unit: m
            movel1.cartTarget.rpy = new double[] { -2.568, 0, Math.PI }; // unit: rad
            robot.executeCommand(MoveCommand.Type.MoveAbsJ, cmds, out ec);
            robot.executeCommand(MoveCommand.Type.MoveL, new List<MoveCommand> { movel1 }, out ec);
            robot.executeCommand(MoveCommand.Type.MoveAbsJ, new List<MoveCommand> { drag }, out ec);
            // 查询运动指令错误信息
            // Console.WriteLine("ec" + robot.lastMoveError().message);

            // 等待运动结束
            waitRobot();

            // 设置工具工件组
            Toolset toolset = new Toolset();
            // 末端位姿 (可视为手持工具的位姿)
            toolset.end.trans = new double[] { 0.1, 0, 0 }; // X +0.1m
            toolset.end.rpy = new double[] { 0.0, Math.PI / 2, 0 }; // Ry +90°
            // 负载3kg
            toolset.load.mass = 3;
            robot.setToolset(toolset, out ec);

            // MoveJ指令 - 轴运动，目标点是笛卡尔点位
            MoveCommand movej1 = new MoveCommand();
            movej1.cartTarget.rpy = new double[] { Math.PI, -1.2, Math.PI };
            movej1.cartTarget.trans = new double[] { 0.48, -0.32, 0.61 };
            movej1.cartTarget.confData = new List<int> { -18, 22, -75, 6, -64, -20 };
            robot.executeCommand(MoveCommand.Type.MoveJ, new List<MoveCommand> { movej1 }, out ec);

            // MoveC指令 - 圆弧运动
            MoveCommand movec = new MoveCommand();
            movec.auxPoint.rpy = new double[] { -Math.PI, -1.2, Math.PI };
            movec.auxPoint.trans = new double[] { 0.27, -0.36, 0.528 };
            movec.cartTarget.rpy = new double[] { -Math.PI, -1.2, Math.PI };
            movec.cartTarget.trans = new double[] { 0.26, -0.178, 0.5 };
            robot.executeCommand(MoveCommand.Type.MoveC, new List<MoveCommand> { movec }, out ec);
            waitRobot();

            robot.executeCommand(MoveCommand.Type.MoveAbsJ, cmds, out ec);
            Thread.Sleep(8000);
            robot.stop(out ec); // 停止运动

            cts.Cancel();
            t.Join();
            cts.Dispose();
        }

        /// <summary>
        /// 示例：使用moveAppend() + moveStart()运动指令；
        /// 以及设置轴配置数据的示例
        /// </summary>
        public void Example_Move2()
        {
            ErrorCode ec;
            Move_Preset(100, 10);


            // 方式二: 查询是否发生碰撞，但是使用前提是需要先设置回调
            // var safetyInfo = robot.queryEventInfo(Event.safety, out ec);
            // Console.WriteLine($"isCollided {(Boolean)safetyInfo["collided"]}");

            string cmdid = "";

            MoveCommand p1 = new MoveCommand(), p2 = new MoveCommand(), p3 = new MoveCommand(), p4 = new MoveCommand();
            p1.cartTarget.trans = new double[] { 0.2434, -0.314, 0.591 };
            p1.cartTarget.rpy = new double[] { 1.5456, 0.314, 2.173 };
            p2.cartTarget.trans = p3.cartTarget.trans = p1.cartTarget.trans;
            p2.cartTarget.rpy = p3.cartTarget.rpy = p1.cartTarget.rpy;

            // 3个目标点，末端位姿相同，轴角度不同
            p1.cartTarget.confData = new List<int> { -67, 100, 88, -79, 90, -120 };
            p2.cartTarget.confData = new List<int> { -76, 8, -133, -106, 103, 108 };
            p3.cartTarget.confData = new List<int> { -70, 8, -88, 90, -105, -25 };

            // 设置一下速度
            p1.speed = 200;
            p2.speed = 1000;
            p3.speed = 400;

            var cmds = new List<MoveCommand> { p1, p2, p3 };
            robot.moveAppend(MoveCommand.Type.MoveJ, cmds, ref cmdid, out ec);
            robot.moveStart(out ec);
            waitForFinish(cmdid, cmds.Count - 1);
        }

        public void Example_MoveSpiral()
        {
            ErrorCode ec;
            Move_Preset(100, 10);

            string cmdid = "";

            MoveCommand absj = new MoveCommand(), sp1 = new MoveCommand(), sp2 = new MoveCommand();
            absj.jointTarget.joints = new List<double> { 0.0, 0.22150561307150393, 1.4779577696969546, 0.0, 1.2675963456219013, 0.0 };
            // 螺旋线1 终点姿态
            sp1.cartTarget.rpy[0] = 2.967;
            sp1.cartTarget.rpy[1] = -0.2;
            sp1.cartTarget.rpy[2] = 3.1415;
            // 螺旋线1 初始半径0.01m, 半径变化步长0.0005m/rad, 逆时针旋转720°，速度v500
            sp1.args.Add(0.01);
            sp1.args.Add(0.0005);
            sp1.args.Add(Math.PI * 4);
            sp1.args.Add(0);

            // 螺旋线2 终点姿态
            sp2.cartTarget.rpy[0] = -2.787577;
            sp2.cartTarget.rpy[1] = 0.1639;
            sp2.cartTarget.rpy[2] = -2.9;
            // 螺旋线2 初始半径0.05m, 半径变化步长0.001m/rad, 顺时针旋转360°，速度v100
            sp2.args.Add(0.05);
            sp2.args.Add(0.001);
            sp2.args.Add(Math.PI * 2);
            sp2.args.Add(1);
          
            // 设置一下速度
            sp1.speed = 500;
            sp2.speed = 100;

            var spcmds = new List<MoveCommand> { sp1, sp2 };
            robot.moveAppend(MoveCommand.Type.MoveAbsJ, new List<MoveCommand> { absj} , ref cmdid, out ec);
            robot.moveAppend(MoveCommand.Type.MoveSP, spcmds, ref cmdid, out ec);

            robot.moveStart(out ec);
            waitForFinish(cmdid, spcmds.Count - 1);
        }


        /// <summary>
        /// 示例：Jog机械臂
        /// </summary>
        void Example_Jog()
        {
            ErrorCode ec;

            // 先运动到拖拽位姿，保证后面的Jog操作可执行
            MoveCommand drag = new MoveCommand();
            drag.jointTarget.joints = new List<double>{ 0, Math.PI / 6, -Math.PI / 2, 0, -Math.PI / 3, 0 };
            robot.executeCommand(MoveCommand.Type.MoveAbsJ, new List<MoveCommand> { drag }, out ec);
            waitRobot();

            // Jog
            // Jog需要手动模式上电
            robot.setOperateMode(OperateMode.manual, out ec);
            robot.setPowerState(true, out ec);

            // 基坐标系，速率0.5(50%), 步长500mm, 沿Z+方向
            // 实际Jog距离以是否超过机器人运动范围为准
            // 笛卡尔坐标系下，index参数0-5分别代表X,Y,Z,Rx,Ry,Rz; 轴空间则代表第n轴
            robot.startJog(JogSpace.baseFrame, 0.5, 500, 2, true, out ec);
            Thread.Sleep(5000);
            robot.stop(out ec); // 需要stop()停止Jog状态
        }

        /// <summary>
        /// 示例：设置碰撞检测灵敏度；打开/关闭碰撞检测
        /// </summary>
        public void Example_SetCollision()
        {
            ErrorCode ec;
            // 设置各轴灵敏度，范围0.01 ~ 2.0，相当于示教器上设置的1% ~ 200%
            var sensitivity = new double[] { 1.0, 1.0, 0.01, 2.0, 1.0, 1.0 };
            // 设置碰撞检测参数，打开碰撞检测
            // 触发行为示例1：安全停止；回退距离0.01m
            robot.enableCollisionDetection(sensitivity, StopLevel.stop1, 0.01, out ec);
            // 触发行为示例2：柔顺停止，柔顺度50%
            // robot.enableCollisionDetection(sensitivity, StopLevel.suppleStop, 0.5, out ec);
            
            // 关闭碰撞检测
            robot.disableCollisionDetection(out ec);
    }

        /// <summary>
        /// 示例：读写寄存器；读写IO
        /// </summary>
        public void Example_Register_IO()
        {
            ErrorCode ec;

            // 寄存器
            List<int> intValues = new List<int>();
            // 读取int16/int32类型寄存器数组
            robot.readRegister("register1", ref intValues, out ec);
            Console.WriteLine("register1 " + intValues[0]);
            // 写入float类型寄存器数组下标5
            robot.writeRegister("register0", 5, (float)132.1, out ec);

            // DIO
            Console.WriteLine("DI1_1 - " + robot.getDI(1, 1, out ec));
            robot.setDO(0, 1, true, out ec);
            Console.WriteLine("DO0_1 - " + robot.getDO(0, 1, out ec));
            // 修改DI/AI需要打开输入仿真模式
            robot.setSimulationMode(true, out ec);
            robot.setDI(0, 0, true, out ec);
            robot.setSimulationMode(false, out ec);

            // 示例: 设置SR机型xPanel输出电压
            robot.setxPanelVout(xPanelOpt.Vout.supply12v, out ec);
        }

        /// <summary>
        /// 示例：打开关闭拖动
        /// </summary>
        public void Example_Drag()
        {
            ErrorCode ec;
            // 打开拖动前需下电，切到手动模式
            robot.setOperateMode(OperateMode.manual, out ec);
            robot.setPowerState(false, out ec);
            robot.enableDrag(DragOpt.Space.cartesianSpace, DragOpt.Type.freely, out ec);

            // 开始录制路径
            Console.WriteLine("开始录制路径, 录制时长限定在30分钟内, 按回车停止录制");
            robot.startRecordPath(out ec);
            while (Console.ReadKey().Key != ConsoleKey.Enter) { }

            // 停止录制路径
            robot.stopRecordPath(out ec);

            // 保存路径。saveAs参数为可选参数，可为空字符串
            robot.saveRecordPath("track0", "", out ec);
            // 取消录制，当录制完不想保存并重新录制时候可以调用
            robot.cancelRecordPath(out ec);

            // 关闭拖动
            robot.disableDrag(out ec);

            // 查询已保存的路径
            var paths = robot.queryPathLists(out ec);
            if (paths.Count >= 1)
            {
                // 路径回放并等待运动结束, 这里的示例是回放第一条
                robot.replayPath(paths[0], 1.0, out ec);
                waitRobot();
                // 删除路径。
                robot.removePath(paths[0], false, out ec);
            }

        }

        /// <summary>
        /// 示例：其它SDK接口调用示例
        /// </summary>
        public void Example_OtherOperations()
        {
            ErrorCode ec;

            // xCore-SDK版本
            robot.sdkVersion();

            // 查询上下电状态/操作模式
            Console.WriteLine("上电状态:" + robot.powerState(out ec).ToString() +
               " | 操作模式:" + robot.operateMode(out ec).ToString());

            // 读取基坐标系、计算正逆解
            Console.Write("\n基坐标系 - [ ");
            Array.ForEach(robot.baseFrame(out ec), PrintHelper.printArray);
            Console.Write("]\n当前轴角度 [ ");
            var jntpos = robot.jointPos(out ec);
            Array.ForEach(jntpos, PrintHelper.printArray);
            Console.Write("]\n计算正解 [ ");
            CartesianPosition fkPos = robot.calcFk(jntpos, out ec);
            Array.ForEach(fkPos.trans, PrintHelper.printArray);
            Array.ForEach(fkPos.rpy, PrintHelper.printArray);

            Console.Write("]\n计算逆解 [ ");
            Array.ForEach(robot.calcIk(fkPos, out ec), PrintHelper.printArray);
            Console.WriteLine("]");

            // 读取末端力矩信息
            double[] joint_torque = new double[6], external_torque = new double[6];
            double[] cart_torque = new double[3], cart_force = new double[3];
            robot.getEndTorque(FrameType.flange, ref joint_torque, ref external_torque,
                ref cart_torque, ref cart_force, out ec);
            Console.Write("轴力矩 [ ");
            Array.ForEach(joint_torque, PrintHelper.printArray);
            Console.Write("]\n外部力矩 [");
            Array.ForEach(external_torque, PrintHelper.printArray);
            Console.Write("]\n笛卡尔空间力矩 [");
            Array.ForEach(cart_torque, PrintHelper.printArray);
            Console.Write("]\n笛卡尔空间力 [");
            Array.ForEach(cart_force, PrintHelper.printArray);
            Console.WriteLine("]");


            // 查询控制器日志
            // 查询最近5条"error"级别日志
            List<LogInfo> logs = robot.queryControllerLog(5, LogInfo.Level.error, out ec);
            foreach (var log in logs)
            {
                Console.Write("Log ID" + log.id + " Content " + log.content + "\n");
            }
        }

        /// <summary>
        /// 运动示例 - 奇异规避模式的直线运动。点位适用机型：xMateCR7
        /// </summary>
        public void Example_AvoidSingularityMove()
        {
            ErrorCode ec;
            robot.setOperateMode(OperateMode.automatic, out ec);
            robot.setPowerState(true, out ec);
            
            string id = "";

            MoveCommand absj1 = new MoveCommand(), movel1 = new MoveCommand(),
                movel2 = new MoveCommand();

            // 先用MoveAbsJ指令运动到起始点
            absj1.jointTarget.joints = new List<double> { 0.453, 0.539, -1.581, 0.0, 0.026, 0 };
            movel1.cartTarget.trans = new double[] { 0.66675437164302165, -0.23850040314585069, 0.85182031 };
            movel1.cartTarget.rpy = new double[] { -3.1415926535897931, 1.0471975511965979, 3.01151 };
            movel2.cartTarget.trans = new double[] { 0.66675437164302154, 0.15775146321850292, 0.464946 };
            movel2.cartTarget.rpy = new double[] { -3.1415926535897931, 1.0471975511965979, -2.688554712 };

            Toolset defaultToolset = new Toolset();
            robot.setToolset(defaultToolset, out ec);
            robot.moveAppend(MoveCommand.Type.MoveAbsJ, new List<MoveCommand>{ absj1 }, ref id, out ec);
            robot.moveStart(out ec);
            waitForFinish(id, 0);

            // 打开4轴锁定模式;
            robot.setAvoidSingularity(true, out ec);
            // 查询是否处于锁定模式
            bool isLimit = robot.getAvoidSingularity(out ec);
            Console.WriteLine($"已打开奇异规避模式 {isLimit}");
            robot.moveAppend(MoveCommand.Type.MoveL, new List<MoveCommand> { movel1, movel2 }, ref id, out ec);
            robot.moveStart(out ec);
            waitForFinish(id, 1);

            // 关闭4轴锁定模式;
            robot.setAvoidSingularity(false, out ec);
        }

        /// <summary>
        /// 示例 - 运行RL工程
        /// </summary>
        public void Example_RLProject()
        {
            ErrorCode ec;
            // 查询所有工程
            var projects = robot.projectsInfo(out ec);
            if (projects.Count >= 1)
            {
                foreach (var project in projects)
                {
                    // 示例：运行返回的例表里第一个工程
                    if (project.taskList.Count > 0)
                    {
                        // 查询该工程下已创建的工具工件
                        var tools = robot.toolsInfo(out ec);
                        Console.Write("工具名称 - ");
                        foreach (var tool in tools)
                        {
                            Console.Write($"{tool.name} ");
                        }
                        Console.WriteLine();
                        var wobjs = robot.wobjsInfo(out ec);
                        Console.Write("工件名称 - ");
                        foreach (var wobj in wobjs)
                        {
                            Console.Write($"{wobj.name} ");
                        }
                        Console.WriteLine();
                        Console.WriteLine($"开始运行RL工程 {project.name}, 按Enter暂停");
                        // 加载工程
                        robot.loadProject(project.name, project.taskList, out ec);
                        // pp-to-main
                        robot.ppToMain(out ec);
                        // 运行程序
                        robot.runProject(out ec);
                        // 示例: 设置单次运行，速率50%
                        robot.setProjectRunningOpt(0.5, false, out ec);
                        while (Console.ReadKey().Key != ConsoleKey.Enter) { }
                        robot.pauseProject(out ec);

                        break;
                    }
                }

            }
        }
        /// <summary>
        /// 示例：工具坐标系的标定
        /// </summary>
        public void Example_CalibrateToolFrame()
        {
            ErrorCode ec;

            int point_num = 4;
            Console.WriteLine($"开始{point_num}点法标定手持工具");
            // 四点法标定手持工具
            CalibrateFrame calibrateFrame = new CalibrateFrame(robot, FrameType.tool, point_num, true);

            for (int i = 0; i < point_num; i++)
            {
                Console.WriteLine("Press <Enter> to confirm next calibraiton point");
                while (Console.ReadKey().Key != ConsoleKey.Enter) { }
                calibrateFrame.SetPoint(i);
                Console.WriteLine($"Set point {i + 1}");
            }

            // 返回标定结果
            var result = calibrateFrame.confirm(out ec);
            if (ec.value != 0)
            {
                Console.WriteLine($"坐标系标定失败: {ec.message}");
                return;
            }
            Action<double> print = new Action<double>(PrintHelper.printArray);
            Console.Write($"标定结果: [ ");
            Array.ForEach(result.frame.trans, print);
            Array.ForEach(result.frame.rpy, print);
            Console.WriteLine(" ]");
            Console.WriteLine("标定误差 - 最小{0:0.000} m, 最大{1:0.000}m, 平均{2:0.000}m", result.errors[0], result.errors[2], result.errors[1]);
        }

        public void Example_AvoidSingularityJog()
        {
            ErrorCode ec;
            robot.setOperateMode(OperateMode.manual, out ec);
            Console.WriteLine("准备Jog机器人, 需手动模式上电, 请确认已上电后按回车键");
            robot.setPowerState(true, out ec);
            Console.WriteLine("-- 开始Jog机器人-- \n奇异规避模式, 沿Y+方向运动50mm, 速率20%，等待机器人停止运动后按回车继续");
            robot.startJog(JogSpace.singularityAvoidMode, 0.2, 50, 1, true, out ec);
            waitForJogStop();
            robot.stop(out ec);
        }
    }
}
