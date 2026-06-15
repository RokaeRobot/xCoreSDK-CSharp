using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using xCoreSDK_CSharp.examples;
using rokae.clr;
using System.Reflection.Metadata;

namespace xCoreSDK_CSharp
{
    public partial class ApplicationForm : Form, FormInterface
    {
        ExampleInterface robotDemo;
        internal static ApplicationForm appFormInstance;
        internal string InfoBox
        {
            get { return richTextBoxExecInfo.Text.ToString(); }
            set
            {
                if (richTextBoxExecInfo.InvokeRequired)
                {
                    richTextBoxExecInfo.Invoke(new Action<string>(info =>
                    {
                        AppendRichBox(this.richTextBoxExecInfo, info);
                    }), value);
                    return;
                }

                AppendRichBox(richTextBoxExecInfo, value);
            }
        }

        public ApplicationForm()
        {
            InitializeComponent();
            appFormInstance = this;
        }

        void FormInterface.setRobotInstance(ExampleInterface robotInstance)
        {
            robotDemo = robotInstance;
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

        private void buttonRunExample_Click(object sender, EventArgs e)
        {
            PrintExecInfo($"运行示例指令 {robotDemo.Demo_JointMove()}");
        }

        private void buttonSyncTime_Click(object sender, EventArgs e)
        {
            PrintExecInfo($"开始记录 {robotDemo.Demo_StartRecordingPos()}");
        }

        private void buttonStopRec_Click(object sender, EventArgs e)
        {
            PrintExecInfo($"停止记录 {robotDemo.Demo_StopRecordingPos()}");
        }

        private void buttonClearLog_Click(object sender, EventArgs e)
        {
            richTextBoxExecInfo.Clear();
        }
    }
}
