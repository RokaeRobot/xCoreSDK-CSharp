using rokae.clr;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using EventInfos = System.Collections.Generic.Dictionary<System.String, System.Object>;

namespace xCoreSDK_CSharp.examples
{
    public class xMateRobotDemo: ExampleInterface
    {
        // 协作六轴机型
        // 方式一: 创建对象并连接到机器人
        // xMateRobot robot = new xMateRobot("192.168.0.160");

        // 方式二：default constructor，然后调用connectToRobot(remoteIP)
        public xMateRobot robot = new xMateRobot();

        CancellationTokenSource cts = new CancellationTokenSource();
        Thread recordThread;

        BaseRobot ExampleInterface.RobotInst { get {
                return robot; } }
        Cobot ExampleInterface.CobotInst { get { return robot; } }


        /// <summary>
        /// 按回车结束
        /// </summary>
        void waitEnter()
        {
            ConsoleKeyInfo keyInfo = Console.ReadKey();
            while (keyInfo.Key != ConsoleKey.Enter)
                keyInfo = Console.ReadKey();
        }

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
        private void waitForFinish(String cmdID, int index)
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
            if (_remark.Length > 0)
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

        private void readStateDate(CancellationToken token)
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

                } while (!token.IsCancellationRequested);
            }
            
            Console.WriteLine("finish record");
        }

        public void connect(string remoteIP)
        {
            robot.connectToRobot(remoteIP);
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
            robot.moveAppend(MoveCommand.Type.MoveAbsJ, new List<MoveCommand> { absj }, ref cmdid, out ec);
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
            drag.jointTarget.joints = new List<double> { 0, Math.PI / 6, -Math.PI / 2, 0, -Math.PI / 3, 0 };
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
            String register1_name = "register1";
            robot.readRegister(register1_name, ref intValues, out ec);
            if (intValues.Count == 0)
            {
                Console.WriteLine($"未创建寄存器 {register1_name}");
            }
            else
            {
                Console.Write($"寄存器{register1_name}: ");
                foreach (int v in intValues)
                {
                    Console.Write($"{v} ");
                }
                Console.WriteLine();
            }

            // 写入float类型寄存器数组下标5
            robot.writeRegister("register0", 5, (float)132.1, out ec);
            //写入寄存器数组
            robot.writeRegister("register0", 1, new bool[] { true,true,false }, out ec);

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

            // 示例：CR机型读取末端按键
            var keypad = robot.getKeypadState(out ec);
            Console.Write("末端按键: ");
            foreach (Boolean state in keypad)
            {
                Console.Write($"{state} ");
            }
            Console.WriteLine("");

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
            var toolset = robot.toolset(out ec);
            Array.ForEach(jntpos, PrintHelper.printArray);
            Console.Write("]\n计算正解 [ ");
            CartesianPosition fkPos = robot.calcFk(jntpos, toolset, out ec);
            Array.ForEach(fkPos.trans, PrintHelper.printArray);
            Array.ForEach(fkPos.rpy, PrintHelper.printArray);

            Console.Write("]\n计算逆解 [ ");
            Array.ForEach(robot.calcIk(fkPos, toolset, out ec), PrintHelper.printArray);
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
            robot.moveAppend(MoveCommand.Type.MoveAbsJ, new List<MoveCommand> { absj1 }, ref id, out ec);
            robot.moveStart(out ec);
            waitForFinish(id, 0);

            // 打开4轴锁定模式;
            robot.setAvoidSingularity(AvoidSingularityMethod.lockAxis4, true, 0, out ec);
            // 查询是否处于锁定模式
            bool isLimit = robot.getAvoidSingularity(AvoidSingularityMethod.lockAxis4, out ec);
            Console.WriteLine($"已打开奇异规避模式 {isLimit}");
            robot.moveAppend(MoveCommand.Type.MoveL, new List<MoveCommand> { movel1, movel2 }, ref id, out ec);
            robot.moveStart(out ec);
            waitForFinish(id, 1);

            // 关闭4轴锁定模式;
            robot.setAvoidSingularity(AvoidSingularityMethod.lockAxis4, false, 0, out ec);
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

        /// <summary>
        /// 示例 - 奇异规避模式Jog
        /// </summary>
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

        /// <summary>
        /// 可达性校验示例
        /// </summary>
        /// <param name="ec"></param>
        /// <returns></returns>
        public void Example_CheckPath()
        {
            // xMateEr7点位
            /* --------------------无辅助点----------------*/
            CartesianPosition start = new CartesianPosition  //t拖拽位姿
            {
                trans = new double[] { 0.631250, 0.0, 0.507386 },
                rpy = new double[] { 180.0 * Math.PI / 180, 0.0, 180.0 * Math.PI / 180 },
            };
            double[] start_joint = new double[] { 0.000, 30.0 * Math.PI / 180, 60.0 * Math.PI / 180, 0.0, 90.0 * Math.PI / 180, 0.0 };

            CartesianPosition target = new CartesianPosition
            {
                trans = new double[] { 0.615167, 0.141585, 0.507386 },
                rpy = new double[] { 180.000 * Math.PI / 180, 0.0, -167.039 * Math.PI / 180 },
            };

            CartesianPosition aux = new CartesianPosition
            {
                trans = new double[] { 0.583553, 0.134309, 0.628928 },
                rpy = new double[] { 180.000 * Math.PI / 180, 11.286 * Math.PI / 180, -167.039 * Math.PI / 180 },
            };
            robot.checkPath(start, start_joint, target, out ErrorCode ec); //直线目标点

            robot.checkPath(start, start_joint, aux, target, out ec); //圆弧目标点

            double angle = 2 * Math.PI; //全圆
            double rot_type = 0;//旋转类型
            robot.checkPath(start, start_joint, aux, target, angle, rot_type, out ec); //整圆目标点

            CartesianPosition target2 = new CartesianPosition
            {
                trans = new double[] { 0.0, 0.0, 1.5295 },
                rpy = new double[] { 0.0, 0.0, 0.0 },
            };
            robot.checkPath(start, start_joint, target2, out ec); //奇异点时

            CartesianPosition target3 = new CartesianPosition
            {
                trans = new double[] { 0.615167, 0.141585, 3.507386 },
                rpy = new double[] { 180.000 * Math.PI / 180, 0.0, -167.039 * Math.PI / 180 },
            };
            robot.checkPath(start, start_joint, target3, out ec); //超出工作范围时时

            // 示例：校验多个点位
            double[] target_joint_calc = new double[] { };
            CartesianPosition[] points = new CartesianPosition[] { start, target, target2, target3 };
            // 若校验失败，返回不可达点的下标
            int wrong_point_index = robot.checkPath(start_joint, points, ref target_joint_calc, out ec);
        }

        /// <summary>
        /// 示例 - 笛卡尔空间力控
        /// </summary>
        private void Example_ForceControl_CartesianSpace(ForceControl fc)
        {
            ErrorCode ec;

            // 设置手持工具坐标系，Ry旋转90°
            Toolset toolset = new Toolset();
            toolset.end.rpy[1] = Math.PI / 2;
            robot.setToolset(toolset, out ec);

            // 力控初始化，使用工具坐标系
            fc.fcInit(FrameType.tool, out ec);

            // 笛卡尔控制模式
            fc.setControlType(1, out ec);

            // 设置笛卡尔刚度。本示例用的是工具坐标系，所以工具坐标系的x方向0阻抗，其余方向阻抗较大
            double[] stiffness = new double[] { 0, 1000, 1000, 500, 500, 500 };
            fc.setCartesianStiffness(stiffness, out ec);

            Console.WriteLine("开始笛卡尔模式力控, [Enter]结束控制");
            // 开始力控
            fc.fcStart(out ec);

            // 设置负载, 请根据实际情况设置，确保安全
            //Load load = new Load();
            //load.mass = 1;
            //fc.setLoad(load, out ec);

            // 设置期望力
            double[] desired_force = new double[] { 0, 0, 1, 0, 0, 0 };
            fc.setCartesianDesiredForce(desired_force, out ec);

            // 按回车结束力控
            waitEnter();
            fc.fcStop(out ec);
        }

        /// <summary>
        /// 示例 - 轴空间力控
        /// </summary>
        private void Example_ForceControl_JointSpace(ForceControl fc)
        {
            ErrorCode ec;

            fc.fcInit(FrameType.baseFrame, out ec);
            // 设置力控类型, 0代表轴空间
            fc.setControlType(0, out ec);
            // 设置各轴刚度。2轴4轴小阻抗，其余轴阻抗较大
            double[] stiffness = new double[] { 1000, 10, 1000, 5, 50, 50 };
            fc.setJointStiffness(stiffness, out ec);
            Console.WriteLine("开始关节模式力控, [Enter]结束控制");
            fc.fcStart(out ec);

            // 设置期望力
            double[] desired = new double[] { 1, 1, 3, 0, 0, 0 };
            fc.setJointDesiredTorque(desired, out ec);

            // 等待输入
            waitEnter();

            fc.fcStop(out ec);
        }

        /// <summary>
        /// 示例 - 力控搜索运动
        /// </summary>
        private void Example_ForceControl_Overlay(ForceControl fc)
        {
            ErrorCode ec;
            // 设置手持工具坐标系，Ry旋转90°
            Toolset toolset1 = new Toolset();
            toolset1.end.rpy[1] = Math.PI / 2;
            robot.setToolset(toolset1, out ec);

            // 可选：设置力控监控参数，示例里用的参数是xMateER3机型默认阈值
            // 设置最大轴速度
            double[] max_vel = new double[] { 3.0, 3.0, 3.5, 3.5, 3.5, 4.0 };
            fc.setJointMaxVel(max_vel, out ec);
            // 设置最大轴动量
            double[] max_momentum = new double[] { 0.1, 0.1, 0.1, 0.055, 0.055, 0.055 };
            fc.setJointMaxMomentum(max_momentum, out ec);
            // 设置最大轴动能
            double[] max_energy = new double[] { 250, 250, 250, 150, 150, 100 };
            fc.setJointMaxEnergy(max_energy, out ec);
            // 设置笛卡尔空间最大速度
            double[] cart_max = new double[] { 1.0, 1.0, 1.0, 2.5, 2.5, 2.5 };
            fc.setCartesianMaxVel(cart_max, out ec);
            // 开始监控
            fc.fcMonitor(true, out ec);

            // 力控初始化
            fc.fcInit(FrameType.tool, out ec);
            // 搜索运动必须为笛卡尔阻抗控制
            fc.setControlType(1, out ec);

            // 设置绕Z轴(因为前面指定了力控坐标系为工具坐标系，所有这里是工具Z轴)的正弦搜索运动
            fc.setSineOverlay(2, 6, 1, Math.PI, 1, out ec);
            // 开始力控
            fc.fcStart(out ec);
            // 叠加XZ平面莉萨如搜索运动
            fc.setLissajousOverlay(1, 5, 1, 10, 5, 0, out ec);

            Console.WriteLine("开始搜索运动, [Enter]结束控制");
            // 开始搜索运动
            fc.startOverlay(out ec);

            // 可选：暂停和重新开始搜索运动
            //fc.pauseOverlay(out ec);
            //fc.restartOverlay(out ec);


            // 按回车结束力控
            waitEnter();
            // 停止搜索运动
            fc.stopOverlay(out ec);

            // 监控参数恢复到默认值
            fc.fcMonitor(false, out ec);
            // 停止力控
            fc.fcStop(out ec);
        }

        /// <summary>
        /// 示例 - 力控终止条件
        /// </summary>
        private void Example_ForceControl_Condition(ForceControl fc)
        {
            ErrorCode ec;

            // 设置参考坐标系（可理解为外部工件坐标系）Z+ 0.1m
            Toolset toolset = new Toolset();
            toolset.reference.trans[2] = 0.1;
            robot.setToolset(toolset, out ec);

            // 设置力控坐标系为世界坐标系
            fc.fcInit(FrameType.world, out ec);
            // 力控类型为笛卡尔空间
            fc.setControlType(1, out ec);
            fc.fcStart(out ec);

            // 设置力限制
            double[] force_condition = new double[] { -20, 20, -15, 15, -15, 15 };
            // 等待时间20s
            fc.setForceCondition(force_condition, true, 20, out ec);

            // 设置长方体区域限制, isInside=false代表在这个区域内时终止等待
            // 长方体所在的坐标系，会叠加外部工件坐标系
            Frame supvFrame = new Frame();
            supvFrame.trans[2] = -0.1;
            // 长方形区域：X -0.6~0.6, Y -0.6~0.6, Z 0.2~0.3
            double[] box = new double[] { -0.6, 0.6, -0.6, 0.6, 0.2, 0.3 };
            // 等待时间20s
            fc.setPoseBoxCondition(supvFrame, box, false, 20, out ec);

            // 阻塞等待满足终止条件
            Console.WriteLine("开始等待");
            fc.waitCondition(out ec);

            Console.WriteLine("等待结束，停止力控");
            fc.fcStop(out ec);
        }

        /// <summary>
        /// 示例 - 力矩传感器标定。
        /// 注意，标定前需要通过setToolset()设置正确的末端负载(toolset.load)
        /// </summary>
        public void Example_CalibrateForceSensor()
        {
            ErrorCode ec;
            // 标定全部轴
            robot.calibrateForceSensor(true, 0, out ec);
            // 单轴(4轴)标定
            robot.calibrateForceSensor(false, 3, out ec);
        }

        /// <summary>
        /// 示例 - 力控指令
        /// </summary>
        public void Example_ForceControl()
        {
            var force_control = robot.forceControl();
            ErrorCode ec;

            // 读取末端力矩信息
            double[] joint_torque = new double[6], external_torque = new double[6];
            double[] cart_torque = new double[3], cart_force = new double[3];
            force_control.getEndTorque(FrameType.flange, ref joint_torque, ref external_torque,
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

            // 自动模式下上电
            robot.setOperateMode(OperateMode.automatic, out ec);
            robot.setPowerState(true, out ec);

            MoveCommand drag = new MoveCommand();

            // 先运动到拖拽位姿，这里用的是xMateER3拖拽位姿
            drag.jointTarget.joints = new List<double> { 0, Math.PI / 6, Math.PI / 3, 0, Math.PI / 2, 0 };
            String cmd_id = "";
            robot.moveAppend(MoveCommand.Type.MoveAbsJ, new List<MoveCommand> { drag }, ref cmd_id, out ec);
            robot.moveStart(out ec);
            waitRobot();

            // 运行一个力控指令示例
            Example_ForceControl_JointSpace(force_control);
        }

        /// <summary>
        /// 示例：同步时间并开始记录
        /// </summary>
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
            cts = new CancellationTokenSource();
            recordThread = new Thread(new ThreadStart(() => readStateDate(cts.Token)));
            recordThread.IsBackground = true;
            recordThread.Start();
            return "";
        }

        /// <summary>
        /// 停止记录
        /// </summary>
        /// <returns></returns>
        string ExampleInterface.Demo_StopRecordingPos()
        {
            if (cts != null) { cts.Cancel(); }
            if (recordThread != null) { recordThread.Join(); }
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
                    joints = new List<double> { 0, 0, 0, 0, 0, 0 },
                    external = new List<double> { -0.2 }

                }
            }, p1 = new MoveCommand()
            {
                jointTarget = new JointPosition()
                {
                    joints = new List<double> { 1.5, 1.5, 1.5, 1.5, 1.5, 1.5 },
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

        private static void ClearCartConf(CartesianPosition p)
        {
            p.confData = new List<int>();
        }

        private static double ClampExt(double v, double extMinM, double extMaxM)
        {
            return Math.Max(extMinM, Math.Min(extMaxM, v));
        }

        private static string EcMessage(ErrorCode ec, string where)
        {
            return ec.value != 0 ? $"{where}: {ec.message}" : "";
        }

        private bool FindEnabledMechUnit(out string mechUnit, out string fixedName, out ErrorCode ec)
        {
            mechUnit = "";
            fixedName = "";
            ec = new ErrorCode(0, "");
            string[] units = { "u1", "u2", "u3", "u4", "u5", "u6" };
            foreach (string u in units)
            {
                bool enabled = false;
                robot.getMechUnit(u, "enable", ref enabled, out ec);
                if (ec.value != 0)
                {
                    continue;
                }
                if (!enabled)
                {
                    continue;
                }
                mechUnit = u;
                fixedName = u;
                string nameFromCtrl = "";
                robot.getMechUnit(u, "fixed_name", ref nameFromCtrl, out ec);
                if (ec.value == 0 && !string.IsNullOrEmpty(nameFromCtrl))
                {
                    fixedName = nameFromCtrl;
                }
                else
                {
                    ec = new ErrorCode(0, "");
                }
                return true;
            }
            return false;
        }

        private string FirstExtAxisName(string mechUnit, out ErrorCode ec)
        {
            ec = new ErrorCode(0, "");
            string[] axes = Array.Empty<string>();
            robot.getMechUnit(mechUnit, "axes_info", ref axes, out ec);
            if (ec.value == 0 && axes != null && axes.Length > 0)
            {
                return axes[0];
            }
            ec = new ErrorCode(0, "");
            return "axis1";
        }

        private static MoveCommand MakeWait5s()
        {
            return new MoveCommand { args = new List<double> { 5.0 } };
        }

        /// <summary>
        /// 附加轴示例：扫描 u1~u6、Jog、带 external 运动，3 轮后回原点，每轮进行movej/l/c附加轴，且每个运动间隔5s。
        /// </summary>
        /// <param name="log">写入 GUI 执行信息</param>
        /// <returns>空字符串表示成功，否则为错误信息</returns>
        public string Example_MoveWithExtAxis(Action<string> log)
        {
            void Step(string msg)
            {
                log(msg);
                Console.WriteLine(msg);
            }

            const double kExtStepM = 0.02;
            const double kStepZ = 0.010;
            const double kStepXy = 0.008;
            const int kCartSpeed = 200;
            const int kCartZone = 50;
            const int kCycleCount = 3;
            double[] qWork =
            {
                0, Math.PI / 6, -Math.PI / 2, 0, -Math.PI / 3, 0
            };

            ErrorCode ec;
            Step("扫描机械单元 u1~u6 ...");
            if (!FindEnabledMechUnit(out string mechUnit, out string fixedName, out ec))
            {
                return "未找到已启用的附加轴机械单元 (u1~u6)，请在控制器中启用附加轴";
            }
            Step($"已启用: {mechUnit}, Jog fixed_name={fixedName}");

            string axisName = FirstExtAxisName(mechUnit, out ec);
            string err = EcMessage(ec, "getMechUnit(axes_info)");
            if (err.Length > 0)
            {
                return err;
            }

            double maxSpeed = 0;
            robot.getExtAxisInfo(axisName, "max_speed", ref maxSpeed, out ec);
            err = EcMessage(ec, "getExtAxisInfo(max_speed)");
            if (err.Length > 0)
            {
                return err;
            }

            double softLower = 0, softUpper = 0;
            robot.getExtAxisInfo(axisName, "soft_limit_lower", ref softLower, out ec);
            err = EcMessage(ec, "getExtAxisInfo(soft_limit_lower)");
            if (err.Length > 0)
            {
                return err;
            }
            robot.getExtAxisInfo(axisName, "soft_limit_upper", ref softUpper, out ec);
            err = EcMessage(ec, "getExtAxisInfo(soft_limit_upper)");
            if (err.Length > 0)
            {
                return err;
            }

            Step($"{axisName} 行程限位(mm): [{softLower}, {softUpper}] max_speed={maxSpeed}");

            Step("切换手动模式并 Jog 附加轴 ...");
            robot.setOperateMode(OperateMode.manual, out ec);
            err = EcMessage(ec, "setOperateMode(manual)");
            if (err.Length > 0)
            {
                return err;
            }
            robot.setPowerState(true, out ec);
            err = EcMessage(ec, "setPowerState(Jog)");
            if (err.Length > 0)
            {
                return err;
            }

            robot.startJogWithExt(JogSpace.jointSpace, 0.1, 5.0, 0, true, fixedName, true, out ec);
            err = EcMessage(ec, "startJogWithExt");
            if (err.Length > 0)
            {
                return err;
            }
            Thread.Sleep(5000);
            robot.stop(out ec);
            err = EcMessage(ec, "stop(Jog)");
            if (err.Length > 0)
            {
                return err;
            }

            Step("Jog 完成，切换自动模式 ...");
            robot.setOperateMode(OperateMode.automatic, out ec);
            err = EcMessage(ec, "setOperateMode(automatic)");
            if (err.Length > 0)
            {
                return err;
            }
            robot.setPowerState(true, out ec);
            err = EcMessage(ec, "setPowerState(auto)");
            if (err.Length > 0)
            {
                return err;
            }

            Toolset toolset = robot.toolset(out ec);
            err = EcMessage(ec, "toolset");
            if (err.Length > 0)
            {
                return err;
            }
            robot.setToolset(toolset, out ec);
            err = EcMessage(ec, "setToolset");
            if (err.Length > 0)
            {
                return err;
            }
            robot.setDefaultSpeed(kCartSpeed, out ec);
            robot.setDefaultZone(kCartZone, out ec);
            robot.setDefaultConfOpt(false, out ec);
            robot.setMotionControlMode(MotionControlMode.NrtCommand, out ec);
            err = EcMessage(ec, "setMotionControlMode");
            if (err.Length > 0)
            {
                return err;
            }

            double extLoM = softLower / 1000.0;
            double extHiM = softUpper / 1000.0;
            double extMinM = extLoM + 0.01;
            double extMaxM = extHiM - 0.01;
            double extOriginM = extMinM;

            Step("计算基准位姿 calcFk ...");
            CartesianPosition pRef = robot.calcFk(qWork, toolset, out ec);
            err = EcMessage(ec, "calcFk");
            if (err.Length > 0)
            {
                return err;
            }
            Step($"导轨原点 external(m)={extOriginM:F3}，共 {kCycleCount} 轮");

            MoveCommand wait5s = MakeWait5s();

            for (int cycle = 1; cycle <= kCycleCount; ++cycle)
            {
                double ext1 = ClampExt(extOriginM + kExtStepM, extMinM, extMaxM);
                double ext2 = ClampExt(extOriginM + 2.0 * kExtStepM, extMinM, extMaxM);
                double ext3 = ClampExt(extOriginM + 3.0 * kExtStepM, extMinM, extMaxM);
                Step($"----- 第 {cycle}/{kCycleCount} 轮 导轨(m): {extOriginM:F3}->{ext1:F3}->{ext2:F3}->{ext3:F3}->{extOriginM:F3} -----");

                CartesianPosition pJ = new CartesianPosition
                {
                    trans = (double[])pRef.trans.Clone(),
                    rpy = (double[])pRef.rpy.Clone(),
                    external = new List<double> { ext1 }
                };
                ClearCartConf(pJ);

                CartesianPosition pL = new CartesianPosition
                {
                    trans = new double[] { pRef.trans[0], pRef.trans[1], pRef.trans[2] - kStepZ },
                    rpy = (double[])pRef.rpy.Clone(),
                    external = new List<double> { ext2 }
                };
                ClearCartConf(pL);

                CartesianPosition pCTgt = new CartesianPosition
                {
                    trans = new double[] { pRef.trans[0] + kStepXy, pRef.trans[1], pRef.trans[2] },
                    rpy = (double[])pRef.rpy.Clone(),
                    external = new List<double> { ext3 }
                };
                ClearCartConf(pCTgt);

                CartesianPosition pCAux = new CartesianPosition
                {
                    trans = new double[] { pRef.trans[0], pRef.trans[1] + kStepXy, pRef.trans[2] },
                    rpy = (double[])pRef.rpy.Clone(),
                    external = new List<double> { ext3 }
                };
                ClearCartConf(pCAux);

                CartesianPosition pHome = new CartesianPosition
                {
                    trans = (double[])pRef.trans.Clone(),
                    rpy = (double[])pRef.rpy.Clone(),
                    external = new List<double> { extOriginM }
                };
                ClearCartConf(pHome);

                MoveCommand absJ = new MoveCommand
                {
                    speed = kCartSpeed,
                    zone = kCartZone,
                    jointTarget = new JointPosition
                    {
                        joints = new List<double>(qWork),
                        external = new List<double> { ext1 }
                    }
                };
                MoveCommand j = new MoveCommand { speed = kCartSpeed, zone = kCartZone, cartTarget = pJ };
                MoveCommand l = new MoveCommand
                {
                    speed = kCartSpeed,
                    zone = kCartZone,
                    cartTarget = pL,
                    customInfo = "ext_axis_demo"
                };
                MoveCommand c = new MoveCommand
                {
                    speed = kCartSpeed,
                    zone = kCartZone,
                    cartTarget = pCTgt,
                    auxPoint = pCAux
                };
                MoveCommand home = new MoveCommand { speed = kCartSpeed, zone = kCartZone, cartTarget = pHome };

                string cmdId = "";
                robot.moveReset(out ec);
                err = EcMessage(ec, "moveReset");
                if (err.Length > 0)
                {
                    return err;
                }

                robot.moveAppend(MoveCommand.Type.MoveAbsJ, absJ, ref cmdId, out ec);
                err = EcMessage(ec, $"第{cycle}轮 MoveAbsJ");
                if (err.Length > 0)
                {
                    return err;
                }
                robot.moveAppend(MoveCommand.Type.MoveWait, wait5s, ref cmdId, out ec);
                err = EcMessage(ec, $"第{cycle}轮 MoveWait");
                if (err.Length > 0)
                {
                    return err;
                }
                robot.moveAppend(MoveCommand.Type.MoveJ, j, ref cmdId, out ec);
                err = EcMessage(ec, $"第{cycle}轮 MoveJ");
                if (err.Length > 0)
                {
                    return err;
                }
                robot.moveAppend(MoveCommand.Type.MoveWait, wait5s, ref cmdId, out ec);
                robot.moveAppend(MoveCommand.Type.MoveL, l, ref cmdId, out ec);
                err = EcMessage(ec, $"第{cycle}轮 MoveL");
                if (err.Length > 0)
                {
                    return err;
                }
                robot.moveAppend(MoveCommand.Type.MoveWait, wait5s, ref cmdId, out ec);
                robot.moveAppend(MoveCommand.Type.MoveC, c, ref cmdId, out ec);
                err = EcMessage(ec, $"第{cycle}轮 MoveC");
                if (err.Length > 0)
                {
                    return err;
                }
                robot.moveAppend(MoveCommand.Type.MoveWait, wait5s, ref cmdId, out ec);
                robot.moveAppend(MoveCommand.Type.MoveL, home, ref cmdId, out ec);
                err = EcMessage(ec, $"第{cycle}轮 回原点 MoveL");
                if (err.Length > 0)
                {
                    return err;
                }

                Step($"第 {cycle} 轮 moveStart ...");
                robot.moveStart(out ec);
                err = EcMessage(ec, $"第{cycle}轮 moveStart");
                if (err.Length > 0)
                {
                    return err;
                }
                waitRobot();
                Step($"第 {cycle} 轮完成");
            }

            return "";
        }
        /// <summary>
        /// 带导轨的运动
        /// </summary>
        public void Example_MoveWithRail()
        {
            ErrorCode ec;
            bool isEnabled = false;
            // 读取导轨参数，支持的参数名和数据类型同setRailParameter
            robot.getRailParameter("enable", ref isEnabled, out ec);
            if (!isEnabled)
            {
                Console.WriteLine("未开启导轨");
                return;
            }

            // 打开关闭导轨，设置导轨参数
            // 设置导轨参数和基坐标系需要重启控制器生效, 这里仅展示接口调用方法
            robot.setRailParameter("enable", true, out ec); // 打开导轨功能
            robot.setRailParameter("maxAcc", 10, out ec); // 设置最大加速度10m/s^2
            double[] rail_soft_limit = new double[] { -0.87, 0.89 };
            robot.setRailParameter("softLimit", rail_soft_limit, out ec); // 设置软限位为+-0.8m
            robot.setRailParameter("reductionRatio", 1.0, out ec); // 设置减速比
            Frame rail_frame = new Frame();
            robot.getRailParameter("baseFrame", ref rail_frame, out ec); // 读取导轨基坐标系

            MoveCommand absj = new MoveCommand()
            {
                jointTarget = new JointPosition()
                {
                    joints = new List<double>() { 0, Math.PI / 6, -Math.PI / 2, 0, -Math.PI / 3, 0 },
                    external = new List<double>() { 0.1 }// 导轨运动到0.1m
                }
            };
            CartesianPosition p0 = new CartesianPosition()
            {
                trans = new double[] { 0.56, 0.136, 0.416 },
                rpy = new double[] { Math.PI, 0, Math.PI },
                external = new List<double>() { 0.02 } // 导轨运动到0.02m, 下同
            };
            CartesianPosition p1 = new CartesianPosition()
            {
                trans = new double[] { 0.56, 0.136, 0.3 },
                rpy = new double[] { Math.PI, 0, Math.PI },
                external = new List<double> { -0.04 }
            };
            MoveCommand j = new MoveCommand()
            {
                cartTarget = p0
            },
            l = new MoveCommand()
            {
                cartTarget = p1,
                customInfo = "hello" // 加一个自定义信息，将在运动信息反馈时返回出来
            },
            c = new MoveCommand()
            {
                cartTarget = p1,
                auxPoint = p0
            };

            string cmdId = "";
            robot.moveAppend(MoveCommand.Type.MoveAbsJ, absj, ref cmdId, out ec);
            robot.moveAppend(MoveCommand.Type.MoveJ, j, ref cmdId, out ec);
            robot.moveAppend(MoveCommand.Type.MoveL, l, ref cmdId, out ec);
            robot.moveAppend(MoveCommand.Type.MoveAbsJ, absj, ref cmdId, out ec);
            robot.moveAppend(MoveCommand.Type.MoveC, c, ref cmdId, out ec);
            robot.moveStart(out ec);
        }


        public void Example_ModbusRTU_EndTool()
        {
            ErrorCode ec;

            // 打开XMC或XMS机型末端485
            robot.setxPanelRS485(xPanelOpt.Vout.supply24v, true, out ec);

            // 寄存器读写
            int[] init_set = new int[]{ 0xA5 };//初始化数据
            robot.XPRWModbusRTUReg(1, 0x10, 0x0100, "int16", 1, ref init_set, false, out ec);

            // 线圈或离散输入读写
            bool[] bool_data = new bool[]{ false };
            robot.XPRWModbusRTUCoil(0x01, 0x01, 0x0001, 1, ref bool_data, false, out ec);

            // 数据透传
            Byte[] send_data = new Byte[]{ 0x01, 0x06, 0x01, 0x00, 0x00, 0xA5, 0x48, 0x4D }; //初始化的透传数据
            Byte[] rev_data = Array.Empty<byte>();// 接收透传数据
            robot.XPRS485SendData(send_data.Length, rev_data.Length, send_data, ref rev_data, out ec);
        }
    }
}
