namespace xCoreSDK_CSharp
{
    partial class ApplicationForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            groupBoxRecPos = new GroupBox();
            buttonStopRec = new Button();
            buttonSyncTime = new Button();
            buttonRunExample = new Button();
            groupBoxExecInfo = new GroupBox();
            buttonClearLog = new Button();
            richTextBoxExecInfo = new RichTextBox();
            groupBoxRecPos.SuspendLayout();
            groupBoxExecInfo.SuspendLayout();
            SuspendLayout();
            // 
            // groupBoxRecPos
            // 
            groupBoxRecPos.Controls.Add(buttonStopRec);
            groupBoxRecPos.Controls.Add(buttonSyncTime);
            groupBoxRecPos.Controls.Add(buttonRunExample);
            groupBoxRecPos.Font = new Font("Microsoft YaHei UI", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            groupBoxRecPos.Location = new Point(21, 16);
            groupBoxRecPos.Margin = new Padding(2);
            groupBoxRecPos.Name = "groupBoxRecPos";
            groupBoxRecPos.Padding = new Padding(2);
            groupBoxRecPos.Size = new Size(291, 69);
            groupBoxRecPos.TabIndex = 11;
            groupBoxRecPos.TabStop = false;
            groupBoxRecPos.Text = "点位记录";
            // 
            // buttonStopRec
            // 
            buttonStopRec.Font = new Font("Microsoft YaHei UI", 9F, FontStyle.Regular, GraphicsUnit.Point);
            buttonStopRec.Location = new Point(199, 28);
            buttonStopRec.Margin = new Padding(2);
            buttonStopRec.Name = "buttonStopRec";
            buttonStopRec.Size = new Size(79, 24);
            buttonStopRec.TabIndex = 14;
            buttonStopRec.Text = "停止记录";
            buttonStopRec.UseVisualStyleBackColor = true;
            buttonStopRec.Click += buttonStopRec_Click;
            // 
            // buttonSyncTime
            // 
            buttonSyncTime.Font = new Font("Microsoft YaHei UI", 9F, FontStyle.Regular, GraphicsUnit.Point);
            buttonSyncTime.Location = new Point(12, 28);
            buttonSyncTime.Margin = new Padding(2);
            buttonSyncTime.Name = "buttonSyncTime";
            buttonSyncTime.Size = new Size(79, 24);
            buttonSyncTime.TabIndex = 13;
            buttonSyncTime.Text = "开始记录";
            buttonSyncTime.UseVisualStyleBackColor = true;
            buttonSyncTime.Click += buttonSyncTime_Click;
            // 
            // buttonRunExample
            // 
            buttonRunExample.Font = new Font("Microsoft YaHei UI", 9F, FontStyle.Regular, GraphicsUnit.Point);
            buttonRunExample.Location = new Point(106, 28);
            buttonRunExample.Margin = new Padding(2);
            buttonRunExample.Name = "buttonRunExample";
            buttonRunExample.Size = new Size(79, 24);
            buttonRunExample.TabIndex = 11;
            buttonRunExample.Text = "运行示例";
            buttonRunExample.UseVisualStyleBackColor = true;
            buttonRunExample.Click += buttonRunExample_Click;
            // 
            // groupBoxExecInfo
            // 
            groupBoxExecInfo.Controls.Add(buttonClearLog);
            groupBoxExecInfo.Controls.Add(richTextBoxExecInfo);
            groupBoxExecInfo.Font = new Font("Microsoft YaHei UI", 10.5F, FontStyle.Regular, GraphicsUnit.Point);
            groupBoxExecInfo.Location = new Point(345, 16);
            groupBoxExecInfo.Margin = new Padding(2);
            groupBoxExecInfo.Name = "groupBoxExecInfo";
            groupBoxExecInfo.Padding = new Padding(2);
            groupBoxExecInfo.Size = new Size(321, 231);
            groupBoxExecInfo.TabIndex = 12;
            groupBoxExecInfo.TabStop = false;
            groupBoxExecInfo.Text = "执行信息";
            // 
            // buttonClearLog
            // 
            buttonClearLog.Font = new Font("Microsoft YaHei UI", 9F, FontStyle.Regular, GraphicsUnit.Point);
            buttonClearLog.Location = new Point(229, 196);
            buttonClearLog.Margin = new Padding(2);
            buttonClearLog.Name = "buttonClearLog";
            buttonClearLog.Size = new Size(79, 24);
            buttonClearLog.TabIndex = 1;
            buttonClearLog.Text = "清空";
            buttonClearLog.UseVisualStyleBackColor = true;
            buttonClearLog.Click += buttonClearLog_Click;
            // 
            // richTextBoxExecInfo
            // 
            richTextBoxExecInfo.Font = new Font("Microsoft YaHei UI", 9F, FontStyle.Regular, GraphicsUnit.Point);
            richTextBoxExecInfo.Location = new Point(11, 22);
            richTextBoxExecInfo.Margin = new Padding(2);
            richTextBoxExecInfo.Name = "richTextBoxExecInfo";
            richTextBoxExecInfo.Size = new Size(298, 167);
            richTextBoxExecInfo.TabIndex = 0;
            richTextBoxExecInfo.Text = "";
            // 
            // ApplicationForm
            // 
            AutoScaleDimensions = new SizeF(10F, 21F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(688, 275);
            Controls.Add(groupBoxExecInfo);
            Controls.Add(groupBoxRecPos);
            Margin = new Padding(2);
            Name = "ApplicationForm";
            Text = "ApplicationForm";
            groupBoxRecPos.ResumeLayout(false);
            groupBoxExecInfo.ResumeLayout(false);
            ResumeLayout(false);
        }

        #endregion
        private GroupBox groupBoxRecPos;
        private Button buttonStopRec;
        private Button buttonSyncTime;
        private Button buttonConfigNtp;
        private Button buttonRunExample;
        private GroupBox groupBoxExecInfo;
        private Button buttonClearLog;
        private RichTextBox richTextBoxExecInfo;
    }
}