using rokae.clr;
using System.Drawing;
using System.Runtime.CompilerServices;
using xCoreSDK_CSharp.examples;
using EventInfos = System.Collections.Generic.Dictionary<System.String, System.Object>;

namespace xCoreSDK_CSharp
{
    public partial class MainForm : Form
    {
        // 本示例以连接协作6轴机器人为例
        // 如需连接其它构型，请自行修改
        //private xMateRobotDemo robotDemo = new xMateRobotDemo();
        private ExampleInterface robotDemo;

        public ExampleInterface RobotDemo { get { return robotDemo; } }

        private bool isConnected = false;

        private ErrorCode ec = new ErrorCode(0, "");
        JogSpace JogSpaceOpt = JogSpace.world;
        double JogStep = 5000;
        double JogRate = 0.2;
        int selectModelIndex = -1;

        public MainForm()
        {
            InitializeComponent();
        }

        private void printMoveExecutionInfo(EventInfos info)
        {
            var _cmdID = (String)info["cmdID"]; // 指令ID
            var _idx = (int)info["wayPointIndex"]; // 路点下标, 从0开始标号
            var _err = (ErrorCode)info["error"]; // 错误信息
            var _remark = (string)info["remark"]; // 警告信息（相近定位）

            string toShow = "";
            if (_err.value != 0)
            {
                toShow = string.Format($"ID: {_cmdID}:{_idx} ,Err: {_err.value}:{_err.message}");
            }
            // 是否达到目标点
            if ((bool)info["reachTarget"])
            {
                toShow = string.Format($"ID: {_cmdID}:{_idx} reach target");

            }
            if (_remark.Length > 0)
            {
                toShow += string.Format($" Warning: {_remark}");
            }
            if (info.ContainsKey("customInfo"))
            {
                var custom_info = (String)info["customInfo"];
                if (custom_info.Length > 0)
                {
                    toShow += " 自定义:" + custom_info;
                }
            }

            if (richTextBoxMoveInfo.InvokeRequired)
            {
                richTextBoxMoveInfo.Invoke(new Action<string>(info =>
                {
                    AppendRichBox(this.richTextBoxMoveInfo, info);
                }), toShow);
                return;
            }
            AppendRichBox(richTextBoxMoveInfo, toShow);

        }

        private void buttonJog_Stop(object sender, MouseEventArgs e)
        {
            robotDemo.RobotInst.stop(out ec);
            printErrorCode("停止Jog", ec);
        }

        private void buttonConnect_Click(object sender, EventArgs e)
        {
            if (!isConnected)
            {
                if(selectModelIndex != -1 && selectModelIndex != comboBoxModelClass.SelectedIndex)
                {
                    robotDemo = null;
                    selectModelIndex = -1;
                }
                if (selectModelIndex == -1)
                {
                    if (comboBoxModelClass.SelectedIndex == 0)
                    {
                        robotDemo = new xMateRobotDemo();
                    }
                    else if (comboBoxModelClass.SelectedIndex == 1)
                    {
                        robotDemo = new xMateErProRobotDemo();
                    }
                    else if (comboBoxModelClass.SelectedIndex == 2)
                    {
                        robotDemo = new StandardRobotDemo();
                    }
                    else if(comboBoxModelClass.SelectedIndex == 3)
                    {
                        robotDemo = new PCB4RobotDemo();
                    }
                    selectModelIndex = comboBoxModelClass.SelectedIndex;
                }
                if (robotDemo == null)
                {
                    PrintExecInfo("请先选择机型类别");
                    return;
                }
                string remoteIp = textBoxIpAddr.Text;
                string localIp = textBoxLocalIp.Text;
                try
                {
                    robotDemo.RobotInst.connectToRobot(remoteIp, localIp);
                }
                catch (Exception ex)
                {
                    PrintExecInfo(ex.Message);
                    return;
                }
                EnableControls(this);
                if (comboBoxModelClass.SelectedIndex == 2 || comboBoxModelClass.SelectedIndex == 3)
                {
                    DisableControls(groupBoxDrag);
                }
                ErrorCode _ec;
                var robotInfo = robotDemo.RobotInst.robotInfo(out _ec);
                textBoxModelName.Text = robotInfo.type;
                textBoxXcoreVer.Text = robotInfo.version;
                textBoxSdkVer.Text = robotDemo.RobotInst.sdkVersion();
                isConnected = true;
                buttonConnect.Text = "断开";

                PrintExecInfo("连接到机器人");
                checkBoxSwitchPower.Checked = (robotDemo.RobotInst.powerState(out _ec) == rokae.clr.PowerState.on);
                checkBoxSwitchMode.Checked = (robotDemo.RobotInst.operateMode(out _ec) == OperateMode.automatic);

                buttonStatusFresh_Click(sender, e);
                buttonMoveUpdate_Click(sender, e);

                robotDemo.RobotInst.setEventWatcher(Event.moveExecution, printMoveExecutionInfo, out _ec);
                printErrorCode("监听运动事件", _ec);
                return;
            }

            if (robotDemo != null)
            {
                robotDemo.RobotInst.disconnectFromRobot(out ErrorCode discEc);
            }
            isConnected = false;
            PrintExecInfo("已断开连接");
            buttonConnect.Text = "连接";
        }

        private void AppendRichBox(RichTextBox box, string content)
        {
            if (string.IsNullOrEmpty(content))
            {
                return;
            }

            box.AppendText(content + System.Environment.NewLine);
            box.ScrollToCaret();
        }

        private void PrintExecInfo(string content)
        {
            if (richTextBoxExecInfo.InvokeRequired)
            {
                richTextBoxExecInfo.Invoke(new Action<string>(info =>
                {
                    AppendRichBox(this.richTextBoxExecInfo, info);
                }), content);
                return;
            }

            AppendRichBox(richTextBoxExecInfo, content);
        }
        private void printErrorCode(string operate, ErrorCode _ec)
        {
            if (_ec.value != 0)
            {
                PrintExecInfo(operate + ": " + _ec.message);
            }
        }

        private void comboBoxJogSpace_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (comboBoxJogSpace.SelectedIndex)
            {
                case 0: JogSpaceOpt = JogSpace.world; break;
                case 1: JogSpaceOpt = JogSpace.flange; break;
                case 2: JogSpaceOpt = JogSpace.baseFrame; break;
                case 3: JogSpaceOpt = JogSpace.toolFrame; break;
                case 4: JogSpaceOpt = JogSpace.wobjFrame; break;
                case 5: JogSpaceOpt = JogSpace.jointSpace; break;
            }
        }

        private void comboBoxJogStep_SelectedIndexChanged_1(object sender, EventArgs e)
        {
            switch (comboBoxJogStep.SelectedIndex)
            {
                case 0: JogStep = 5000; break;
                case 1: JogStep = 10; break;
                case 2: JogStep = 1; break;
                case 3: JogStep = 0.1; break;
                case 4: JogStep = 0.01; break;
            }
        }

        private void buttonJog_MouseDown(object sender, MouseEventArgs e)
        {
            string btnName = (sender as Button).Name;
            Boolean dir = false;
            int idx = 0;
            if (btnName == "buttonJog1Neg") { dir = false; idx = 0; }
            if (btnName == "buttonJog1Pos") { dir = true; idx = 0; }
            if (btnName == "buttonJog2Neg") { dir = false; idx = 1; }
            if (btnName == "buttonJog2Pos") { dir = true; idx = 1; }
            if (btnName == "buttonJog3Neg") { dir = false; idx = 2; }
            if (btnName == "buttonJog3Pos") { dir = true; idx = 2; }
            if (btnName == "buttonJog4Neg") { dir = false; idx = 3; }
            if (btnName == "buttonJog4Pos") { dir = true; idx = 3; }
            if (btnName == "buttonJog5Neg") { dir = false; idx = 4; }
            if (btnName == "buttonJog5Pos") { dir = true; idx = 4; }
            if (btnName == "buttonJog6Neg") { dir = false; idx = 5; }
            if (btnName == "buttonJog6Pos") { dir = true; idx = 5; }
            if (btnName == "buttonJog7Neg") { dir = false; idx = 6; }
            if (btnName == "buttonJog7Pos") { dir = true; idx = 6; }
            robotDemo.Demo_JogRobot(JogSpaceOpt, JogRate, JogStep, dir, idx, ref ec);
            printErrorCode("Jog", ec);
        }

        private void trackBarJogSpeed_ValueChanged(object sender, EventArgs e)
        {
            updateJogRate();
        }
        private void updateJogRate()
        {
            JogRate = (double)trackBarJogSpeed.Value * 0.01;
            labelJogRate.Text = trackBarJogSpeed.Value.ToString() + "%";
        }

        private void buttonStatusFresh_Click(object sender, EventArgs e)
        {
            var jnt = robotDemo.RobotInst.jointPos(out ec);
            printErrorCode("获取当前位置", ec);
            if (ec.value == 0)
            {
                textBoxCurrJnt1.Text = string.Format("{0:0.000}", jnt[0] * 180.0 / Math.PI);
                textBoxCurrJnt2.Text = string.Format("{0:0.000}", jnt[1] * 180.0 / Math.PI);
                textBoxCurrJnt3.Text = string.Format("{0:0.000}", jnt[2] * 180.0 / Math.PI);
                textBoxCurrJnt4.Text = string.Format("{0:0.000}", jnt[3] * 180.0 / Math.PI);
                if(jnt.Length > 4)
                {
                    textBoxCurrJnt5.Text = string.Format("{0:0.000}", jnt[4] * 180.0 / Math.PI);
                    textBoxCurrJnt6.Text = string.Format("{0:0.000}", jnt[5] * 180.0 / Math.PI);
                }
            }
            var cart = robotDemo.RobotInst.posture(CoordinateType.endInRef, out ec);
            if (ec.value == 0)
            {
                textBoxCurrX.Text = string.Format("{0:0.000}", cart[0] * 1000.0);
                textBoxCurrY.Text = string.Format("{0:0.000}", cart[1] * 1000.0);
                textBoxCurrZ.Text = string.Format("{0:0.000}", cart[2] * 1000.0);
                textBoxCurrA.Text = string.Format("{0:0.000}", cart[3] * 180.0 / Math.PI);
                textBoxCurrB.Text = string.Format("{0:0.000}", cart[4] * 180.0 / Math.PI);
                textBoxCurrC.Text = string.Format("{0:0.000}", cart[5] * 180.0 / Math.PI);
            }
        }

        private void trackBarMove_ValueChanged(object sender, EventArgs e)
        {
            updateMoveParameter();
        }
        private void updateMoveParameter()
        {
            labelSpeedValue.Text = string.Format("{0}mm/s", trackBarMoveSpeed.Value);
            labelZoneValue.Text = string.Format("{0}mm", trackBarMoveZone.Value);
        }

        private void buttonMoveUpdate_Click(object sender, EventArgs e)
        {
            var jnt = robotDemo.RobotInst.jointPos(out ec);
            printErrorCode("获取当前位置", ec);
            if (ec.value == 0)
            {
                textBoxMoveToJnt1.Text = string.Format("{0:0.000}", jnt[0] * 180.0 / Math.PI);
                textBoxMoveToJnt2.Text = string.Format("{0:0.000}", jnt[1] * 180.0 / Math.PI);
                textBoxMoveToJnt3.Text = string.Format("{0:0.000}", jnt[2] * 180.0 / Math.PI);
                textBoxMoveToJnt4.Text = string.Format("{0:0.000}", jnt[3] * 180.0 / Math.PI);
                if(jnt.Length > 4)
                {
                    textBoxMoveToJnt5.Text = string.Format("{0:0.000}", jnt[4] * 180.0 / Math.PI);
                    textBoxMoveToJnt6.Text = string.Format("{0:0.000}", jnt[5] * 180.0 / Math.PI);
                }
            }
            var cart = robotDemo.RobotInst.posture(CoordinateType.endInRef, out ec);
            if (ec.value == 0)
            {
                textBoxMoveToX.Text = string.Format("{0:0.000}", cart[0] * 1000.0);
                textBoxMoveToY.Text = string.Format("{0:0.000}", cart[1] * 1000.0);
                textBoxMoveToZ.Text = string.Format("{0:0.000}", cart[2] * 1000.0);
                textBoxMoveToA.Text = string.Format("{0:0.000}", cart[3] * 180.0 / Math.PI);
                textBoxMoveToB.Text = string.Format("{0:0.000}", cart[4] * 180.0 / Math.PI);
                textBoxMoveToC.Text = string.Format("{0:0.000}", cart[5] * 180.0 / Math.PI);
            }
        }

        private void buttonStartMove_Click(object sender, EventArgs e)
        {

            robotDemo.RobotInst.moveStart(out ec);
            printErrorCode("开始运动", ec);
        }

        private void buttonPauseMove_Click(object sender, EventArgs e)
        {
            robotDemo.RobotInst.stop(out ec);
            printErrorCode("暂停运动", ec);
        }

        private void trackBarSpeedRatio_ValueChanged(object sender, EventArgs e)
        {
            labelSpeedRatioValue.Text = string.Format("{0}%", trackBarSpeedRatio.Value);
            robotDemo.RobotInst.adjustSpeedOnline((double)trackBarSpeedRatio.Value * 0.01, out ec);
            printErrorCode("调整速度", ec);
        }

        private void buttonMoveAppend_Click(object sender, EventArgs e)
        {
            var moveType = MoveCommand.Type.MoveAbsJ;
            double[] trans_m = new double[]
            {
                double.Parse(textBoxMoveToX.Text),
                double.Parse(textBoxMoveToY.Text),
                double.Parse(textBoxMoveToZ.Text)
            };
            double[] rpy_deg = new double[]
            {
                double.Parse(textBoxMoveToA.Text),
                double.Parse(textBoxMoveToB.Text),
                double.Parse(textBoxMoveToC.Text)
            };
            double[] joint_deg = new double[]
            {
                double.Parse(textBoxMoveToJnt1.Text),
                double.Parse(textBoxMoveToJnt2.Text),
                double.Parse(textBoxMoveToJnt3.Text),
                double.Parse(textBoxMoveToJnt4.Text),
                double.Parse(textBoxMoveToJnt5.Text),
                double.Parse(textBoxMoveToJnt6.Text),
            };


            MoveCommand cmd = new MoveCommand
            {
                speed = trackBarMoveSpeed.Value,
                zone = trackBarMoveZone.Value,
                cartTarget = new CartesianPosition
                {
                    trans = new double[] { trans_m[0] / 1000.0, trans_m[1] / 1000.0, trans_m[2] / 1000.0 },
                    rpy = new double[] { rpy_deg[0] / 180.0 * Math.PI, rpy_deg[1] / 180.0 * Math.PI, rpy_deg[2] / 180.0 * Math.PI, }
                },
                jointTarget = new JointPosition
                {
                    joints = new List<double>
                    {
                       joint_deg[0]/ 180.0 * Math.PI,
                       joint_deg[1] / 180.0 * Math.PI,
                       joint_deg[2] / 180.0 * Math.PI,
                       joint_deg[3] / 180.0 * Math.PI,
                       joint_deg[4] / 180.0 * Math.PI,
                       joint_deg[5]/ 180.0 * Math.PI,
                    }
                }
            };
            if (radioButtonJ.Checked)
            {
                moveType = MoveCommand.Type.MoveJ;

            }
            else if (radioButtonL.Checked)
            {
                moveType = MoveCommand.Type.MoveL;
            }
            string cmdId = "";
            robotDemo.RobotInst.moveAppend(moveType, new List<MoveCommand> { cmd }, ref cmdId, out ec);
            printErrorCode("添加指令", ec);

            if (ec.value != 0)
            {
                return;
            }
            string cmdStr = "";
            if (moveType == MoveCommand.Type.MoveL || moveType == MoveCommand.Type.MoveJ)
            {
                cmdStr = string.Format($"{cmdId}: {moveType.ToString()} [{trans_m[0]}, {trans_m[1]}, {trans_m[2]}, {rpy_deg[0]}, {rpy_deg[1]}, {rpy_deg[2]}], v{cmd.speed}, z{cmd.zone}");
            }
            else
            {
                cmdStr = string.Format($"{cmdId}: {moveType} [{joint_deg[0]}, {joint_deg[1]}, {joint_deg[2]}, {joint_deg[3]}, {joint_deg[4]}, {joint_deg[5]}], v{cmd.speed}, z{cmd.zone}");
            }

            if (richTextBoxCmdAppend.InvokeRequired)
            {
                richTextBoxCmdAppend.Invoke(new Action<string>(info =>
                {
                    AppendRichBox(this.richTextBoxCmdAppend, info);
                }), cmdStr);
                return;
            }
            AppendRichBox(richTextBoxCmdAppend, cmdStr);
        }

        private void DisableControls(Control con)
        {
            foreach (Control c in con.Controls)
            {
                DisableControls(c);
            }
            con.Enabled = false;
        }

        private void EnableControls(Control con)
        {
            foreach (Control c in con.Controls)
            {
                EnableControls(c);
            }
            con.Enabled = true;
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            textBoxIpAddr.Text = "192.168.0.160";

            updateJogRate();
            updateMoveParameter();
            comboBoxJogSpace.SelectedIndex = 0;
            comboBoxJogStep.SelectedIndex = 0;
            comboBoxModelClass.SelectedIndex = 0;
            radioButtonAbsj.Checked = true;
            radioButtonDragFree.Checked = true;
            radioButtonDragCart.Checked = true;

            textBoxMoveToX.Text = "0";
            textBoxMoveToY.Text = "0";
            textBoxMoveToZ.Text = "0";
            textBoxMoveToA.Text = "0";
            textBoxMoveToB.Text = "0";
            textBoxMoveToC.Text = "0";
            textBoxMoveToJnt1.Text = "0";
            textBoxMoveToJnt2.Text = "0";
            textBoxMoveToJnt3.Text = "0";
            textBoxMoveToJnt4.Text = "0";
            textBoxMoveToJnt5.Text = "0";
            textBoxMoveToJnt6.Text = "0";

            DisableControls(panelBottomBar);
            DisableControls(groupBoxCurr);
            DisableControls(groupBoxDrag);
            DisableControls(groupBoxMotion);
            DisableControls(groupBoxJog);
        }

        private void buttonMoveReset_Click(object sender, EventArgs e)
        {
            robotDemo.RobotInst.moveReset(out ec);
            printErrorCode("运动重置", ec);
            if (ec.value == 0)
            {
                richTextBoxCmdAppend.Clear();
                richTextBoxMoveInfo.Clear();
            }

        }

        private void buttonClearLog_Click(object sender, EventArgs e)
        {
            richTextBoxExecInfo.Clear();
        }

        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (isConnected && robotDemo != null)
            {
                robotDemo.RobotInst.disconnectFromRobot(out ErrorCode ec);
            }        }

        private void checkBoxDrag_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBoxDrag.Checked)
            {
                checkBoxDrag.Text = "关闭";
            }
            else
            {
                checkBoxDrag.Text = "打开";
            }
        }

        private void radioButtonDragJnt_CheckedChanged(object sender, EventArgs e)
        {
            if (radioButtonDragJnt.Checked)
            {
                radioButtonDragFree.Checked = true;
            }
        }

        private void checkBoxSwitchMode_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBoxSwitchMode.Checked)
            {
                checkBoxSwitchMode.Text = "手动";
            }
            else
            {
                checkBoxSwitchMode.Text = "自动";
            }
        }

        private void checkBoxSwitchPower_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBoxSwitchPower.Checked)
            {
                checkBoxSwitchPower.Text = "下电";
            }
            else
            {
                checkBoxSwitchPower.Text = "上电";
            }
        }

        private void checkBoxSwitchMode_MouseClick(object sender, MouseEventArgs e)
        {
            bool isChecked = checkBoxSwitchMode.Checked;
            if (isChecked)
            {
                robotDemo.RobotInst.setOperateMode(OperateMode.automatic, out ec);
            }
            else
            {
                robotDemo.RobotInst.setOperateMode(OperateMode.manual, out ec);
            }
            printErrorCode("切换模式", ec);
            if (ec.value != 0)
            {
                checkBoxSwitchMode.Checked = !isChecked;
            }

        }

        private void checkBoxSwitchPower_MouseClick(object sender, MouseEventArgs e)
        {
            bool isChecked = checkBoxSwitchPower.Checked;
            robotDemo.RobotInst.setPowerState(isChecked, out ErrorCode errr);
            printErrorCode("上下电", errr);
            if (ec.value != 0)
            {
                checkBoxSwitchPower.Checked = !isChecked;
            }
        }


        private void buttonExtAxis_Click(object sender, EventArgs e)
        {
            if (!isConnected)
            {
                PrintExecInfo("请先连接机器人");
                return;
            }
            var demo = robotDemo as xMateRobotDemo;
            if (demo == null)
            {
                PrintExecInfo("附加轴示例仅支持协作六轴机型");
                return;
            }

            buttonExtAxis.Enabled = false;
            PrintExecInfo("======== 附加轴示例开始 ========");
            try
            {
                string result = demo.Example_MoveWithExtAxis(PrintExecInfo);
                if (string.IsNullOrEmpty(result))
                {
                    PrintExecInfo("======== 附加轴示例完成 ========");
                    buttonStatusFresh_Click(sender, e);
                }
                else
                {
                    PrintExecInfo("附加轴示例失败: " + result);
                }
            }
            catch (Exception ex)
            {
                PrintExecInfo("附加轴示例异常: " + ex.Message);
            }
            finally
            {
                buttonExtAxis.Enabled = true;
            }
        }

        private void checkBoxDrag_MouseClick(object sender, MouseEventArgs e)
        {
            if (checkBoxDrag.Checked)
            {
                var space = DragOpt.Space.cartesianSpace;
                if (radioButtonDragJnt.Checked)
                {
                    space = DragOpt.Space.jointSpace;
                }
                var type = DragOpt.Type.freely;
                if (radioButtonDragTrans.Checked)
                {
                    type = DragOpt.Type.translationOnly;
                }
                else if (radioButtonDragRot.Checked)
                {
                    type = DragOpt.Type.rotationOnly;
                }
                robotDemo.CobotInst.enableDrag(space, type, false, out ec);
                printErrorCode("打开拖动", ec);
                if (ec.value == 0)
                {
                    PrintExecInfo("拖动已打开");
                }
                else
                {
                    checkBoxDrag.Checked = false;
                }
                return;
            }
            robotDemo.CobotInst.disableDrag(out ec);
            printErrorCode("关闭拖动", ec);
            if (ec.value == 0)
            {
                PrintExecInfo("拖动已关闭");
            }
            else
            {
                checkBoxDrag.Checked = true;
            }
        }
    }
}